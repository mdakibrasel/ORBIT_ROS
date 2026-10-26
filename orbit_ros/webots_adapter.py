"""Webots adapter: the ORBIT-ROS task world and calibration bay run inside a simulator.

`WebotsWorld2D` is a `World2D` whose world lives in a simulator. Agents keep calling `world.execute(tool, args)` and reading
`world.x / y / yaw / collisions`, so no planner, verifier or recovery code changes. The simulator sits behind the small
`SimBackend` interface, so everything here is unit-tested without Webots (`FakeSim`); Webots is imported only inside `WebotsSim`.

Two motion modes
* grid        After every move_forward / turn the robot is teleported to the resulting cell and yaw (grid-faithful, stage C1).
              Logic is identical to World2D, so any disagreement comes from world generation or the perception mapping.
* continuous  Real wheel motion on encoder odometry (`orbit_ros.continuous.DiffDrive`): lidar-gated cell steps, stall and contact
              detection, physical room walls. The agent's pose is read back from the simulator.

Both the task world and the calibration bay (the identifier's real fixture runs, via `embodiment_runtime.BAY_HOOK`) run in the
simulator. Dry runs under hypotheses stay pure Python, because they are predictions, not robot motion.

Status: the adapter logic, FakeSim and the control loops are unit-tested. `WebotsSim` / `WebotsPlant` were written from the Webots R2025a
reference pages and have not been run in Webots by the author: start with `--mode probe`.
"""
from __future__ import annotations
import math
from .world2d import World2D, Obj, to_robot_frame
from .continuous import DiffDrive, FakePlant, Plant, wrap

LABEL_COLORS = {"red box": (0.85, 0.1, 0.1), "blue box": (0.1, 0.2, 0.85), "green box": (0.1, 0.65, 0.2),
                "red shelf": (0.7, 0.2, 0.2), "blue shelf": (0.2, 0.3, 0.7), "red chair": (0.9, 0.4, 0.4),
                "calibration marker": (0.9, 0.8, 0.1)}
OBJ_SIZE, OBJ_HEIGHT = 0.6, 0.6      # metres; a cell is 1 m, so 0.2 m of clearance remains on each side
WALL_T, WALL_H = 0.05, 0.5

def label_color(label: str) -> tuple:
    if label in LABEL_COLORS: return LABEL_COLORS[label]
    h = (sum(ord(c) * (i + 1) for i, c in enumerate(label)) % 255) / 255.0
    return (h, 0.5, 1.0 - h)

def solid_vrml(obj_id: str, label: str, x: int, y: int) -> str:
    """One obstacle: a plain Solid (WoodenBox cannot be recognised) with `model` and `recognitionColors`."""
    r, g, b = label_color(label); s, hgt = OBJ_SIZE, OBJ_HEIGHT
    return (f'Solid {{ translation {x} {y} {hgt / 2} name "obj_{obj_id}" model "{label}" recognitionColors [ {r} {g} {b} ] '
            f'children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 1 metalness 0 }} '
            f'geometry Box {{ size {s} {s} {hgt} }} }} ] boundingObject Box {{ size {s} {s} {hgt} }} }}')

def look_at_rotation(pos, target) -> list:
    """Webots Viewpoint rotation (axis-angle) that looks from `pos` at `target`. Webots R2022a+ viewpoints look along their local +x
    with +z up, so the rotation is Rz(yaw) * Ry(pitch), pitch > 0 tilting the view down. Display only: no effect on any sensor."""
    dx, dy, dz = (target[i] - pos[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]
    ang = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if ang < 1e-9: return [0.0, 0.0, 1.0, 0.0]
    if abs(ang - math.pi) < 1e-6:                                # 180 deg: axis from the diagonal
        ax = [math.sqrt(max(0.0, (R[i][i] + 1) / 2)) for i in range(3)]
        if ax[0] > 1e-6: ax[1], ax[2] = math.copysign(ax[1], R[0][1]), math.copysign(ax[2], R[0][2])
        elif ax[1] > 1e-6: ax[2] = math.copysign(ax[2], R[1][2])
        return [*ax, ang]
    k = 2 * math.sin(ang)
    return [(R[2][1] - R[1][2]) / k, (R[0][2] - R[2][0]) / k, (R[1][0] - R[0][1]) / k, ang]

def room_view(w: int, h: int, pitch_deg: float = 50.0) -> tuple:
    """(position, rotation) of a 3D overview from the south side of a w x h room: x to the right, y away from the viewer, as on the map."""
    target = ((w - 1) / 2, (h - 1) / 2, 0.0)
    d = max(w, 1.4 * h) + 1.5
    p = math.radians(pitch_deg)
    pos = (target[0], target[1] - d * math.cos(p), d * math.sin(p))
    return pos, look_at_rotation(pos, target)

def wall_specs(w: int, h: int) -> list:
    """Four room walls around cells 0..w-1 x 0..h-1 as (name, cx, cy, size_x, size_y). No recognitionColors: the camera never sees them."""
    t = WALL_T
    return [("wall_n", (w - 1) / 2, h - 0.5 + t / 2, w + 2 * t, t), ("wall_s", (w - 1) / 2, -0.5 - t / 2, w + 2 * t, t),
            ("wall_w", -0.5 - t / 2, (h - 1) / 2, t, h + 2 * t), ("wall_e", w - 0.5 + t / 2, (h - 1) / 2, t, h + 2 * t)]

def wall_vrml(name, cx, cy, sx, sy) -> str:
    return (f'Solid {{ translation {cx} {cy} {WALL_H / 2} name "{name}" children [ Shape {{ appearance PBRAppearance {{ baseColor 0.93 0.9 0.84 roughness 0.8 metalness 0 }} '
            f'geometry Box {{ size {sx} {sy} {WALL_H} }} }} ] boundingObject Box {{ size {sx} {sy} {WALL_H} }} }}')

# ---------------------------------------------------------------------------------------------------------------
class SimBackend:
    """What the adapter needs from a simulator. Cells are integer (x, y); yaw in degrees, 0 = +x, CCW positive."""
    camera_width = 160
    active = None                                   # token of the world currently loaded
    def load_scene(self, w: int, h: int, objects: list, start: tuple, yaw: int): raise NotImplementedError
    def set_robot_pose(self, x: int, y: int, yaw: int): raise NotImplementedError
    def robot_pose(self) -> tuple: raise NotImplementedError                 # (x_m, y_m, yaw_deg)
    def object_cells(self) -> list: raise NotImplementedError                # [(label, x_m, y_m)]
    def recognition(self) -> list: raise NotImplementedError                 # [{"label","f","l","px"}], camera frame, px = pixel x
    def sim_time(self) -> float: return 0.0
    def plant(self) -> Plant: raise NotImplementedError                      # wheels, encoders, lidar, contacts
    def driver(self) -> DiffDrive:
        if getattr(self, "_driver", None) is None: self._driver = DiffDrive(self.plant())
        return self._driver

class FakeSim(SimBackend):
    """Pure-Python simulator with an independent pinhole camera (pixel quantisation) and a kinematic plant."""
    def __init__(self, fov_deg=90.0, width=160, lidar_front_index=270):
        self.fov, self.camera_width = fov_deg, width
        self._objs, self._plant, self.steps = [], FakePlant(front_index=lidar_front_index), 0
    def load_scene(self, w, h, objects, start, yaw):
        self.w, self.h = w, h
        self._objs = [(o.label, float(o.x), float(o.y)) for o in objects]
        s = OBJ_SIZE / 2
        self._plant.boxes = [(float(o.x), float(o.y), s, s) for o in objects] + [(cx, cy, sx / 2, sy / 2) for _, cx, cy, sx, sy in wall_specs(w, h)]
        self.set_robot_pose(start[0], start[1], yaw)
    def set_robot_pose(self, x, y, yaw):
        self._plant.set_pose(x, y, math.radians(yaw)); self.steps += 1
        if getattr(self, "_driver", None) is not None: self._driver.reset_odom()
    def robot_pose(self):
        x, y, yaw = self._plant.truth(); return (x, y, math.degrees(yaw) % 360)
    def object_cells(self): return list(self._objs)
    def recognition(self):
        x0, y0, yaw = self._plant.truth(); out = []
        for label, ox, oy in self._objs:
            f, l = to_robot_frame(ox - x0, oy - y0, math.degrees(yaw))
            if f <= 0: continue
            right = -l / f / math.tan(math.radians(self.fov / 2))          # +1 = right image edge (pinhole: linear in tan)
            out.append({"label": label, "f": f, "l": l, "px": round(right * self.camera_width / 2 + self.camera_width / 2, 1)})
        return out
    def sim_time(self): return self.steps * 0.032
    def plant(self): return _CountingPlant(self, self._plant)

class _CountingPlant(Plant):
    """Wraps FakePlant so simulated time advances with every control step."""
    def __init__(self, sim, p): self.sim, self.p, self.dt = sim, p, p.dt
    def set_wheels(self, wl, wr): self.p.set_wheels(wl, wr)
    def step(self): self.p.step(); self.sim.steps += 1
    def encoders(self): return self.p.encoders()
    def truth(self): return self.p.truth()
    def lidar(self): return self.p.lidar()
    def contact_events(self): return self.p.contact_events()

class WebotsPlant(Plant):
    """TurtleBot3 Burger devices: 'left wheel motor', 'right wheel motor', wheel sensors, 'LDS-01' lidar, supervisor contact points."""
    def __init__(self, sim: "WebotsSim"):
        self.sim, sup = sim, sim.sup
        self.dt = sim.dt / 1000.0
        self.ml, self.mr = sup.getDevice("left wheel motor"), sup.getDevice("right wheel motor")
        self.sl, self.sr = sup.getDevice("left wheel sensor"), sup.getDevice("right wheel sensor")
        self.lid = sup.getDevice("LDS-01")
        for name, d in (("left wheel motor", self.ml), ("right wheel motor", self.mr), ("left wheel sensor", self.sl), ("right wheel sensor", self.sr), ("LDS-01", self.lid)):
            if d is None: raise RuntimeError(f"device '{name}' not found: the robot in the world must be a TurtleBot3Burger with its LDS-01 lidar (see webots/worlds/orbit_c1.wbt)")
        for m in (self.ml, self.mr): m.setPosition(float("inf")); m.setVelocity(0.0)
        self.sl.enable(sim.dt); self.sr.enable(sim.dt); self.lid.enable(sim.dt)
        self.contacts, self._touch = 0, False
        sim.step(2)                                            # sensors deliver values only after a step
    def set_wheels(self, wl, wr): self.ml.setVelocity(wl); self.mr.setVelocity(wr)
    def step(self):
        self.sim.step(1)
        touching = len(self.sim.self_node.getContactPoints(False)) > 0      # robot body only (descendants excluded: wheel-floor contacts)
        if touching and not self._touch: self.contacts += 1
        self._touch = touching
    def encoders(self): return (self.sl.getValue(), self.sr.getValue())
    def truth(self):
        x, y, yaw = self.sim.robot_pose(); return (x, y, math.radians(yaw))
    def lidar(self): return list(self.lid.getRangeImage())
    def contact_events(self): return self.contacts

class WebotsSim(SimBackend):
    """Webots R2025a backend. Only runs inside a Webots controller (needs the `controller` module)."""
    def __init__(self, robot=None, camera_name="camera", settle_steps=2):
        from controller import Supervisor                      # imported lazily: not available outside Webots
        self.sup = robot or Supervisor()
        self.dt = int(self.sup.getBasicTimeStep())
        self.self_node = self.sup.getSelf()
        self.t_field, self.r_field = self.self_node.getField("translation"), self.self_node.getField("rotation")
        self.z0 = self.t_field.getSFVec3f()[2]
        self.cam = self.sup.getDevice(camera_name)
        if self.cam is None: raise RuntimeError(f"Camera '{camera_name}' not found: check webots/worlds/orbit_c1.wbt")
        self.cam.enable(self.dt); self.cam.recognitionEnable(self.dt)
        self.camera_width = self.cam.getWidth()
        self.children = self.sup.getRoot().getField("children")
        self.nodes, self.walls, self.settle, self._plant = [], [], settle_steps, None
    def step(self, n=1):
        for _ in range(n):
            if self.sup.step(self.dt) == -1: raise RuntimeError("Webots simulation ended")
    def _add(self, vrml):
        self.children.importMFNodeFromString(-1, vrml); return self.children.getMFNode(-1)
    def load_scene(self, w, h, objects, start, yaw):
        for n in self.nodes + self.walls: n.remove()
        self.nodes, self.walls, self.w, self.h = [], [], w, h
        for o in objects: self.nodes.append(self._add(solid_vrml(o.id, o.label, o.x, o.y)))
        for spec in wall_specs(w, h): self.walls.append(self._add(wall_vrml(*spec)))
        self._frame_view(w, h)
        self.set_robot_pose(start[0], start[1], yaw)
    def _frame_view(self, w, h):
        """v0.15.1: point the 3D view (DEF ORBIT_VIEW Viewpoint in the world file) at the room just built. Display only."""
        vp = self.sup.getFromDef("ORBIT_VIEW")
        if vp is None: return
        pos, rot = room_view(w, h)
        try:
            vp.getField("position").setSFVec3f(list(pos)); vp.getField("orientation").setSFRotation(rot)
        except Exception:                                      # an older world without the fields: keep Webots' own view
            pass
    def set_robot_pose(self, x, y, yaw):
        self.t_field.setSFVec3f([float(x), float(y), self.z0])
        self.r_field.setSFRotation([0.0, 0.0, 1.0, math.radians(yaw % 360)])
        self.self_node.resetPhysics()                          # inertia is not reset automatically after a teleport
        self.step(self.settle)                                 # setters take effect at the end of the step
        if getattr(self, "_driver", None) is not None: self._driver.reset_odom()
    def robot_pose(self):
        p = self.self_node.getPosition(); R = self.self_node.getOrientation()   # R is row-major 3x3
        return (p[0], p[1], math.degrees(math.atan2(R[3], R[0])) % 360)
    def object_cells(self):
        return [(n.getField("model").getSFString(), n.getPosition()[0], n.getPosition()[1]) for n in self.nodes]
    def recognition(self):
        out = []
        for ro in self.cam.getRecognitionObjects():
            pos, pix = ro.getPosition(), ro.getPositionOnImage()   # camera frame: x forward, y left (ENU); verified by --mode probe
            out.append({"label": ro.getModel(), "f": pos[0], "l": pos[1], "px": float(pix[0])})
        return out
    def sim_time(self): return self.sup.getTime()
    def plant(self):
        if self._plant is None: self._plant = WebotsPlant(self)
        return self._plant

# ---------------------------------------------------------------------------------------------------------------
class WebotsWorld2D(World2D):
    """A World2D whose world is mirrored into (grid) or driven through (continuous) a simulator. Build with `adopt`."""
    @classmethod
    def adopt(cls, world: World2D, sim: SimBackend, perception: str = "camera", motion: str = "grid"):
        if perception not in ("camera", "truth"): raise ValueError(perception)
        if motion not in ("grid", "continuous"): raise ValueError(motion)
        w = object.__new__(cls)
        w.__dict__.update(world.__dict__)
        w._sim, w._perception, w._motion, w._token = sim, perception, motion, object()
        w.sync_error_max, w.sync_checks, w.view_calls, w.view_mismatch = 0.0, 0, 0, 0
        w.contact_collisions, w.odom_error_max, w.stop_reasons = 0, 0.0, []
        w.objects = list(world.objects)
        w._load()
        w._cells = {(round(x), round(y)) for _, x, y in sim.object_cells()}   # obstacles as the SIMULATOR holds them
        return w

    def _load(self):
        self._sim.load_scene(self.w, self.h, self.objects, (self.x, self.y), self.yaw)
        self._sim.active = self._token
        self._check_sync()
    def _ensure(self):
        """The calibration bay and the task world share one simulator: reload this world if another one was loaded since."""
        if self._sim.active is not self._token: self._load()

    # sync -----------------------------------------------------------------------------------------------------
    def _check_sync(self):
        x, y, yaw = self._sim.robot_pose()
        err = max(abs(x - self.x), abs(y - self.y), abs(((yaw - self.yaw) + 180) % 360 - 180) / 90.0)
        self.sync_error_max, self.sync_checks = max(self.sync_error_max, err), self.sync_checks + 1
    def _push(self):
        self._sim.set_robot_pose(self.x, self.y, self.yaw); self._check_sync()

    # geometry / pose / camera ----------------------------------------------------------------------------------
    def occupied(self, x, y): return (x, y) in self._cells
    def pose(self):
        self._ensure()
        sx, sy, syaw = self._sim.robot_pose()                    # the agent reads the true pose, rounded to cells
        return {"x": int(round(sx)), "y": int(round(sy)), "yaw_deg": int(round(syaw / 90.0)) % 4 * 90, "room_w": self.w, "room_h": self.h}

    @staticmethod
    def views_match(a, b, tol_x=0.05, tol_size=0.03):
        """Same labels, and image_x / apparent_size within tolerance (pixel quantisation is expected)."""
        if sorted(o["label"] for o in a) != sorted(o["label"] for o in b): return False
        for o in a:
            m = [q for q in b if q["label"] == o["label"]]
            if not any(abs(q["image_x"] - o["image_x"]) <= tol_x and abs(q["apparent_size"] - o["apparent_size"]) <= tol_size for q in m): return False
        return True

    def describe_view(self):
        self._ensure()
        truth = World2D.describe_view(self)
        if self._perception == "truth": return truth
        out = self._camera_view()
        self.view_calls += 1; self.view_mismatch += 0 if self.views_match(out, truth) else 1
        return out

    def _camera_view(self):
        seen, half = [], self._sim.camera_width / 2
        for r in self._sim.recognition():
            f, l = r["f"], r["l"]; d = math.hypot(f, l); b = math.degrees(math.atan2(l, f))
            if f <= 0 or d > self.range or abs(b) > self.fov / 2 + 1e-9: continue
            ix = max(-1.0, min(1.0, (r["px"] - half) / half))
            if self.mirrored: ix = -ix                           # Webots has no mirror switch: negate in the driver
            seen.append((r["label"], d, b, ix))
        out = []
        for i, (label, d, b, ix) in enumerate(seen):
            if any(j != i and d2 < d and abs(b2 - b) < 1.0 for j, (_, d2, b2, _) in enumerate(seen)): continue   # occluded, as in World2D
            out.append({"label": label, "image_x": round(ix, 3), "apparent_size": round(1 / d, 3)})
        return out

    # tools ----------------------------------------------------------------------------------------------------
    def execute(self, tool, args):
        self._ensure()
        if self._motion == "continuous" and tool in ("move_forward", "turn"): return self._execute_continuous(tool, args)
        res = super().execute(tool, args)
        if tool in ("move_forward", "turn"):
            self._push()
            if res.get("data") is not None: res["data"] = self.pose()   # report the pose as read back from the simulator
        return res

    def _adopt_truth(self):
        """Continuous mode: the logical cell and yaw follow the physical robot."""
        x, y, yaw = self._sim.robot_pose()
        self.x, self.y, self.yaw = int(round(x)), int(round(y)), int(round(yaw / 90.0)) % 4 * 90
        self.odom_error_max = max(self.odom_error_max, self._sim.driver().odom_error())

    def _execute_continuous(self, tool, args):
        self.tool_calls += 1
        for item in list(self.drift_schedule):
            if self.tool_calls >= item[0]:
                for k, v in item[1].items(): setattr(self, k, v)
                self.drift_schedule.remove(item)
        def bad(msg):
            self.invalid_calls += 1; return {"ok": False, "message": msg, "data": None}
        if self.stopped: return bad("robot is stopped")
        drv = self._sim.driver(); c0 = self._sim.plant().contact_events()
        if tool == "turn":
            d, deg = args.get("direction"), args.get("degrees")
            if d not in ("left", "right") or deg not in (90, 180, 270): return bad("turn needs direction left|right, degrees 90|180|270")
            signed = deg if d == "left" else -deg
            if self.swapped: signed = -signed
            drv.rotate(int(abs(signed) * self.turn_gain / 90 + 0.5) * 90 * (1 if signed > 0 else -1))
            self._adopt_truth()
            self.contact_collisions += self._sim.plant().contact_events() - c0
            return {"ok": True, "message": "turned", "data": self.pose()}
        n = args.get("distance")
        if not isinstance(n, int) or isinstance(n, bool) or not 0 <= n <= 12: return bad("move_forward needs integer distance 0..12")
        n_commanded, n = n, int(n * self.move_gain + 0.5)
        k, why = drv.drive_cells(n)
        self._adopt_truth()
        self.contact_collisions += self._sim.plant().contact_events() - c0
        self.stop_reasons.append(why)
        if k < n:
            self.collisions += 1            # same meaning as World2D: an attempted move into an obstacle or wall
            return {"ok": False, "message": f"blocked after {k} m", "data": self.pose()}
        return {"ok": True, "message": f"moved {n_commanded} m", "data": self.pose()}

def calibrate_lidar(sim: SimBackend, tol=0.12) -> int:
    """Find which lidar ray points forward: put one obstacle two cells ahead (face at 1.7 m) and take the nearest ray."""
    w = WebotsWorld2D.adopt(World2D(9, 7, (4, 3), 0, [Obj("c", "calibration marker", 6, 3)]), sim)
    rs = sim.plant().lidar()
    finite = [(v, i) for i, v in enumerate(rs) if v == v and v != float("inf")]
    if not finite: raise RuntimeError("lidar sees nothing: is the LDS-01 enabled and the world loaded?")
    v, idx = min(finite)
    if abs(v - 1.7) > tol: raise RuntimeError(f"lidar calibration: nearest obstacle at {v:.2f} m, expected 1.70 m (marker face)")
    sim.driver().front_idx = idx
    return idx
