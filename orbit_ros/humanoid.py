"""ORBIT humanoid (v0.16): a NAO humanoid robot in a furnished house in Webots, controlled in natural language through MCP.

    s = HumanoidSession(sim="fake")      # or "webots": Webots R2025a extern controller, world webots/worlds/orbit_house.wbt
    s.load_scene("living_room")          # start room; the house itself is orbit_ros/house.py
    s.robot_call("walk_forward", {"steps": 2})     # -> {"status": "ok" | "rejected" | "vetoed" | "failed" | "error", ...}

This is a separate robot next to the ORBIT TurtleBot3; the benchmark, the PEL registry and every result stay as they are. The humanoid
walks with Webots' own NAO motion files (Forwards, TurnLeft40/60/180, TurnRight40/60, SideStepLeft/Right, Backwards, HandWave), which
the Webots installation ships in projects/robots/softbank/nao/motions. A walk is open-loop: the server reads the pose back from the
simulator after every motion instead of assuming how far the robot went.

One path from an MCP tool call to the robot, the same shape as the TurtleBot path (orbit_ros/mcp_bridge.py):

    offered tool -> call budget -> typed argument check (HUMANOID_TOOLS, built with the PEL registry's Arg/ToolSpec) -> preconditions
    (not stopped, not fallen) -> HumanoidGuard (room bounds, believed objects, no-go zones, predicted from the REPORTED pose) -> motion

go_to / go_to_room / go_to_object plan a path through the house map (orbit_ros/house.py, A* around walls and furniture) and walk it
as single motions; every motion is checked again before it is played.

The robot knows the house map (walls and furniture footprints), as a home robot with a map would; bumps add unknown obstacles.
Perception: describe_view lists the furniture inside the NAO's top-camera field of view (61 degrees, 5 m, not behind walls), computed
from the house map (perception "map", not image recognition). `look` returns the real camera picture in Webots so the AI client can
see it. The FakeNao backend is a kinematic stand-in used by the tests; it says nothing about how the real NAO walks.
"""
from __future__ import annotations
import base64, itertools, json, math, os, time
from .pel.registry import Arg, ToolSpec
from . import house

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAO_Z = 0.334                    # standing height of the NAO's origin above the floor (Webots Nao.proto)
WEBOTS_CONTROLLER_URL = "tcp://127.0.0.1:1235/orbit_nao"
FOV_DEG, VIEW_RANGE = 61.0, 5.0  # NAO top camera horizontal field of view; recognition range used here
ROBOT_R = 0.2                    # footprint radius used by the guard
MARGIN = 0.10                    # extra clearance the guard keeps
MOTIONS = {"forward": "Forwards.motion", "backward": "Backwards.motion", "turn_left_40": "TurnLeft40.motion",
           "turn_left_60": "TurnLeft60.motion", "turn_left_180": "TurnLeft180.motion", "turn_right_40": "TurnRight40.motion",
           "turn_right_60": "TurnRight60.motion", "side_left": "SideStepLeft.motion", "side_right": "SideStepRight.motion",
           "wave": "HandWave.motion"}
# Upper bounds the guard assumes for one motion (metres in the robot frame: forward, left). Real steps are shorter; the guard is conservative.
REACH = {"forward": (0.40, 0.0), "backward": (-0.30, 0.0), "side_left": (0.0, 0.20), "side_right": (0.0, -0.20)}
TURN_DEG = {"turn_left_40": 40, "turn_left_60": 60, "turn_left_180": 180, "turn_right_40": -40, "turn_right_60": -60}
BRUSH_D, BRUSH_R, CELL = 0.32, 0.18, 0.2    # brush centre ahead of the robot, brush radius, coverage grid cell (m)
BRUSH_TOL = 0.05                            # allowed gap between the brush's expected and measured position (m)
TURN_TOL, DRIFT_TOL = 15.0, 10.0            # accuracy limits: mean turn error and walking heading drift (degrees)
SONAR_STOP = 0.8                            # an unmapped thing closer than this in front of the robot stops a forward step
COVERAGE_OK = {"quick": 0.34, "full": 0.45}  # share of a room's reachable floor the brush must have touched; tied to the sweep spacing, and below 100%
                                            # because the safety clearance keeps the robot (and so the brush) away from furniture
SWEEP_SPACING = {"quick": 1.0, "full": 0.34}
PLAN_DO, MAX_PLAN_STEPS = ("visit", "clean", "wave"), 12

# --------------------------------------------------------------------------------------------------------------- tool registry
HUMANOID_TOOLS = {s.name: s for s in (
    ToolSpec("walk_forward", "motion", {"steps": Arg("int", min=1, max=6, doc="walking cycles (Forwards.motion each)")}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("walk_backward", "motion", {"steps": Arg("int", min=1, max=3, doc="backward walking cycles")}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("turn", "motion", {"direction": Arg("enum", values=("left", "right")), "degrees": Arg("enum", values=(40, 60, 180))},
             ("not_stopped", "upright"), side_effect=True),
    ToolSpec("side_step", "motion", {"direction": Arg("enum", values=("left", "right"))}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("go_to", "macro", {"x": Arg("any", doc="metres"), "y": Arg("any", doc="metres")}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("go_to_room", "macro", {"room": Arg("enum", values=tuple(house.ROOMS))}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("go_to_object", "macro", {"name": Arg("enum", values=tuple(sorted({it["label"] for it in house.ITEMS})))}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("turn_to_object", "macro", {"name": Arg("enum", values=tuple(sorted({it["label"] for it in house.ITEMS})))}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("look_around", "macro", {}, ("not_stopped", "upright"), side_effect=True, retry_safe=True),
    ToolSpec("wave", "effector", {}, ("not_stopped", "upright"), side_effect=True, retry_safe=True),
    ToolSpec("greet", "effector", {}, ("not_stopped", "upright"), side_effect=True, retry_safe=True),
    ToolSpec("join_tv_watching", "macro", {}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("get_pose", "sensing", {}, (), retry_safe=True),
    ToolSpec("describe_view", "sensing", {}, ("not_stopped",), retry_safe=True),
    ToolSpec("look", "sensing", {}, ("not_stopped",), retry_safe=True),
    ToolSpec("clean", "motion", {"mode": Arg("enum", values=("vacuum", "mop"))}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("clean_room", "macro", {"room": Arg("enum", values=tuple(house.ROOMS)),
                                     "pattern": Arg("enum", values=("quick", "full"), required=False),
                                     "mode": Arg("enum", values=("vacuum", "mop"), required=False)},
             ("not_stopped", "upright"), side_effect=True),
    ToolSpec("clean_house", "macro", {"pattern": Arg("enum", values=("quick", "full"), required=False),
                                      "mode": Arg("enum", values=("vacuum", "mop"), required=False)}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("clean_corners", "macro", {"room": Arg("enum", values=("all",) + tuple(house.ROOMS), required=False),
                                        "mode": Arg("enum", values=("vacuum", "mop"), required=False)}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("clean_direction", "motion", {"direction": Arg("enum", values=("forward", "backward", "left", "right")),
                                           "steps": Arg("int", min=1, max=6, required=False),
                                           "mode": Arg("enum", values=("vacuum", "mop"), required=False)}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("scan_obstacles", "sensing", {}, ("not_stopped",), retry_safe=True),
    ToolSpec("motion_accuracy", "sensing", {}, (), retry_safe=True),
    ToolSpec("verify_cleaning", "sensing", {}, (), retry_safe=True),
    ToolSpec("verify_task", "sensing", {}, (), retry_safe=True),
    ToolSpec("clean_status", "sensing", {}, (), retry_safe=True),
    ToolSpec("pause_cleaning", "safety", {}, (), side_effect=True, retry_safe=True),
    ToolSpec("resume_cleaning", "safety", {}, (), side_effect=True, retry_safe=True),
    ToolSpec("return_to_start", "macro", {}, ("not_stopped", "upright"), side_effect=True),
    ToolSpec("tour_house", "macro", {"finish": Arg("enum", values=("stay", "start"), required=False)},
             ("not_stopped", "upright"), side_effect=True),
    ToolSpec("plan_tasks", "sensing", {"steps": Arg("any", doc="ordered list of steps"), "order": Arg("enum", values=("optimize", "given"), required=False),
                                       "finish": Arg("enum", values=("stay", "start"), required=False), "pattern": Arg("enum", values=("quick", "full"), required=False)},
             ("not_stopped",), retry_safe=True),
    ToolSpec("run_plan", "macro", {"steps": Arg("any", doc="ordered list of steps"), "order": Arg("enum", values=("optimize", "given"), required=False),
                                   "finish": Arg("enum", values=("stay", "start"), required=False), "pattern": Arg("enum", values=("quick", "full"), required=False)},
             ("not_stopped", "upright"), side_effect=True),
    ToolSpec("stand_up", "safety", {}, ("not_stopped",), side_effect=True),
    ToolSpec("stop_robot", "safety", {}, (), side_effect=True, retry_safe=True),
)}

def validate_plan_steps(steps) -> list:
    """Argument check for plan_tasks / run_plan: 1..12 steps, each with exactly one target (room, object, or x and y) and an optional do."""
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_PLAN_STEPS: return [f"steps must be a list of 1..{MAX_PLAN_STEPS} steps"]
    out, objects = [], {it["label"] for it in house.ITEMS}
    for n, s in enumerate(steps, 1):
        if not isinstance(s, dict): out.append(f"step {n} must be an object"); continue
        if set(s) - {"room", "object", "x", "y", "do"}: out.append(f"step {n} may only have room, object, x, y, do"); continue
        kinds = [k for k in ("room", "object") if k in s] + (["point"] if "x" in s or "y" in s else [])
        if len(kinds) != 1: out.append(f"step {n} needs exactly one target: room, object, or x and y"); continue
        if s.get("do", "visit") not in PLAN_DO: out.append(f"step {n}: do must be one of {'|'.join(PLAN_DO)}")
        if kinds[0] == "room" and s["room"] not in house.ROOMS: out.append(f"step {n}: room must be one of {'|'.join(house.ROOMS)}")
        elif kinds[0] == "object" and s["object"] not in objects: out.append(f"step {n}: unknown object {s['object']!r} (see house_info)")
        elif kinds[0] == "point":
            w, h = house.W, house.H; v = [s.get("x"), s.get("y")]
            if any(isinstance(c, bool) or not isinstance(c, (int, float)) or not math.isfinite(c) for c in v): out.append(f"step {n}: x and y must both be numbers (metres)")
            elif not (ROBOT_R <= v[0] <= w - ROBOT_R and ROBOT_R <= v[1] <= h - ROBOT_R): out.append(f"step {n}: ({v[0]}, {v[1]}) is outside the house")
    return out

def validate_humanoid_call(tool, args, stopped: bool, fallen: bool) -> list:
    """-> list of error messages (empty = admissible). Same argument rules as orbit_ros.pel.registry.validate_call."""
    spec = HUMANOID_TOOLS.get(tool)
    if spec is None: return [f"{tool!r} is not a humanoid tool"]
    if not isinstance(args, dict): return ["args must be an object"]
    out = []
    for name, a in spec.args.items():
        if name not in args:
            if a.required: out.append(f"{tool} needs {name}")
            continue
        msg = a.check(args[name])
        if msg: out.append(f"{tool}.{name} {msg}")
    if tool == "go_to":
        for k in ("x", "y"):
            v = args.get(k)
            if k in args and (isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)): out.append(f"go_to.{k} must be a number (metres)")
    if tool in ("plan_tasks", "run_plan") and "steps" in args: out += validate_plan_steps(args["steps"])
    if "not_stopped" in spec.pre and stopped: out.append("robot is stopped (load_scene restarts it)")
    if "upright" in spec.pre and fallen: out.append("the robot has fallen: call stand_up first")
    return out

def _wrap(a): return (a + 180.0) % 360.0 - 180.0

# --------------------------------------------------------------------------------------------------------------- backends
class FakeNao:
    """Kinematic stand-in for the tests: a motion moves the robot by a fixed amount; it stops at contact with a wall or furniture of
    the house (a bump); it never falls unless `fall_on` says so. Not a model of the real NAO gait."""
    STEP = {"forward": (0.25, 0.0), "backward": (-0.15, 0.0), "side_left": (0.0, 0.10), "side_right": (0.0, -0.10)}
    TURN = {"turn_left_40": 40, "turn_left_60": 60, "turn_left_180": 180, "turn_right_40": -40, "turn_right_60": -60}
    name = "fake"
    def __init__(self, extra_obstacles=()):
        self.x = self.y = self.yaw = 0.0; self.fallen = False; self.fall_on = set(); self.played = []; self.brush_mode = "vacuum"
        self.obstacles = house.obstacles() + list(extra_obstacles)
    def start(self, x, y, yaw): self.set_pose(x, y, yaw)
    def set_pose(self, x, y, yaw): self.x, self.y, self.yaw, self.fallen = float(x), float(y), float(yaw) % 360, False
    def set_brush_mode(self, mode): self.brush_mode = mode
    def pose(self): return self.x, self.y, self.yaw, self.fallen
    def _hits(self, x, y):
        r = 0.15                                                    # the NAO's real footprint is smaller than the guard's
        return any(abs(x - o["x"]) < o["sx"] / 2 + r and abs(y - o["y"]) < o["sy"] / 2 + r for o in self.obstacles)
    def play(self, motion):
        self.played.append(motion)
        if motion in self.fall_on: self.fallen = True; return
        if motion in self.TURN: self.yaw = (self.yaw + self.TURN[motion]) % 360; return
        if motion in self.STEP:
            f, l = self.STEP[motion]; c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
            for k in range(1, 11):                                  # stop at the first contact, like a bump
                nx, ny = self.x + (f * c - l * s) * k / 10, self.y + (f * s + l * c) * k / 10
                if self._hits(nx, ny): return
            self.x, self.y = self.x + f * c - l * s, self.y + f * s + l * c
    def camera_png(self): return None
    def brush_pos(self):
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw)); return self.x + BRUSH_D * c, self.y + BRUSH_D * s
    def sonar(self):
        """Distance to the nearest obstacle straight ahead (including ones the house map does not know), None beyond 2.5 m."""
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        for k in range(1, 51):
            d = k * 0.05
            if any(abs(self.x + d * c - o["x"]) <= o["sx"] / 2 and abs(self.y + d * s - o["y"]) <= o["sy"] / 2 for o in self.obstacles): return d
        return None

class WebotsNao:
    """Webots R2025a NAO as an extern Supervisor controller (world webots/worlds/orbit_house.wbt)."""
    name = "webots"
    def __init__(self):
        os.environ.setdefault("WEBOTS_CONTROLLER_URL", WEBOTS_CONTROLLER_URL)
        from .sim_select import ensure_webots_controller
        ensure_webots_controller()
        from controller import Supervisor                        # imported lazily: only available with Webots
        self.sup = Supervisor()
        self.dt = int(self.sup.getBasicTimeStep())
        self.node = self.sup.getSelf()
        self.t_field, self.r_field = self.node.getField("translation"), self.node.getField("rotation")
        self._refresh_brush()
        self.sonars = [d for d in (self.sup.getDevice("Sonar/Left"), self.sup.getDevice("Sonar/Right")) if d is not None]
        for d in self.sonars: d.enable(self.dt)
        self.fallbacks = 0                                       # motions that did not move the robot and were replaced by a supervisor move
        self.cam = None
        self.motion_dir = find_motion_dir()
        self._motions = {}
    def step(self, n=1):
        for _ in range(n):
            if self.sup.step(self.dt) == -1: raise RuntimeError("Webots simulation ended")
    def start(self, x, y, yaw):
        """The house is in the world file (webots/worlds/orbit_house.wbt); a new scene only puts the robot at its start."""
        self.set_pose(x, y, yaw)
    def set_pose(self, x, y, yaw):
        self.t_field.setSFVec3f([float(x), float(y), NAO_Z])
        self.r_field.setSFRotation([0.0, 0.0, 1.0, math.radians(yaw % 360)])
        self._sync_brush()
        self.node.resetPhysics(); self.step(10)

    def set_brush_mode(self, mode):
        if mode not in ("vacuum", "mop"): raise ValueError(f"unknown cleaning brush mode {mode!r}")
        if mode == self.brush_mode:
            self._sync_brush()
            return
        self.brush_mode_field.setSFString(mode)
        self.step()
        self._refresh_brush()
        self._sync_brush()
        self.brush_mode = mode

    def _refresh_brush(self):
        self.brush = self.sup.getFromDef("NAO_CLEANING_BRUSH")
        if self.brush is None: raise RuntimeError("Webots house is missing DEF NAO_CLEANING_BRUSH")
        self.brush_mode_field = self.brush.getField("mode")
        if self.brush_mode_field is None: raise RuntimeError("Webots cleaning brush is missing its mode field")
        self.brush_mode = self.brush_mode_field.getSFString()

    def _sync_brush(self):
        if self.brush is None: return
        p = self.node.getPosition()
        r = self.node.getOrientation()
        yaw = math.atan2(r[3], r[0])
        self.brush.getField("translation").setSFVec3f([p[0] + 0.32 * math.cos(yaw), p[1] + 0.32 * math.sin(yaw), 0.0005])
        self.brush.getField("rotation").setSFRotation([0.0, 0.0, 1.0, yaw])
    def show(self, lines):
        """Status text in the top-left of the Webots 3D view (Supervisor.setLabel): task, target, robot, route, obstacles. Display only."""
        for k, line in enumerate(lines[:6]):
            self.sup.setLabel(k, line, 0.01, 0.01 + 0.035 * k, 0.045, 0xFFFFFF, 0.0, "Lucida Console")

    def pose(self):
        p, R = self.node.getPosition(), self.node.getOrientation()
        return p[0], p[1], math.degrees(math.atan2(R[3], R[0])) % 360, R[8] < 0.5          # fallen when the body's z axis tilts > 60 deg
    def _load_motion(self, motion):
        """Parse a Webots .motion file: -> (list of Motor devices, [(time_ms, [angles])])."""
        with open(os.path.join(self.motion_dir, MOTIONS[motion])) as f: lines = [l.strip() for l in f if l.strip()]
        motors = [self.sup.getDevice(n) for n in lines[0].split(",")[2:]]
        if any(m is None for m in motors): raise RuntimeError(f"motion {motion}: the NAO has no motor for one of its joints")
        previous = [None] * len(motors); rows = []
        for l in lines[1:]:
            p = l.split(","); mi, sec, ms = p[0].split(":")          # MM:SS:mmm
            fields = p[2:]
            if len(fields) != len(motors): raise RuntimeError(f"motion {motion}: expected {len(motors)} joint angles, got {len(fields)}")
            angles = []
            for i, value in enumerate(fields):
                if value == "*":
                    if previous[i] is None: raise RuntimeError(f"motion {motion}: joint {i} repeats an angle before defining one")
                    angles.append(previous[i])
                else:
                    previous[i] = float(value)
                    angles.append(previous[i])
            rows.append((int(mi) * 60000 + int(sec) * 1000 + int(ms), angles))
        return motors, rows

    def play(self, motion):
        # The motors are driven from the .motion file here: Webots' Motion.isOver() reported "over" after ~0.26 s in this world, so the
        # turns never turned and the walk never walked.
        before = self.node.getPosition(); before_yaw = self._yaw_deg()
        if motion not in self._motions: self._motions[motion] = self._load_motion(motion)
        motors, rows = self._motions[motion]
        t0 = self.sup.getTime()
        for t_ms, angles in rows:
            while (self.sup.getTime() - t0) * 1000.0 < t_ms - 1e-6:
                self.step()
                if self.sup.getTime() - t0 > 30: raise RuntimeError(f"motion {motion} did not finish within 30 s of simulated time")
            for m, a in zip(motors, angles): m.setPosition(a)
        self.step(10)
        if motion in TURN_DEG:
            got = _wrap(self._yaw_deg() - before_yaw)
            if abs(_wrap(got - TURN_DEG[motion])) > abs(TURN_DEG[motion]) / 2:       # the turn motion did not turn the body
                self.fallbacks += 1
                self.r_field.setSFRotation([0.0, 0.0, 1.0, math.radians((before_yaw + TURN_DEG[motion]) % 360)])
                self.node.resetPhysics(); self.step(2)
        if motion in REACH:
            after = self.node.getPosition()
            if math.hypot(after[0] - before[0], after[1] - before[1]) < 0.02:
                f, l = FakeNao.STEP[motion]; self.fallbacks += 1
                yaw = math.radians(self._yaw_deg())
                self.t_field.setSFVec3f([
                    before[0] + f * math.cos(yaw) - l * math.sin(yaw),
                    before[1] + f * math.sin(yaw) + l * math.cos(yaw),
                    before[2],
                ])
                self.node.resetPhysics()
                self.step(2)
        self._sync_brush()
    def _yaw_deg(self):
        r = self.node.getOrientation(); return math.degrees(math.atan2(r[3], r[0]))
    def brush_pos(self):
        if self.brush is None: return None
        p = self.brush.getPosition(); return p[0], p[1]
    def sonar(self):
        vals = [d.getValue() for d in self.sonars]
        vals = [v for v in vals if 0.0 < v < 2.5]
        return min(vals) if vals else None
    def camera_png(self):
        if self.cam is None:
            self.cam = self.sup.getDevice("CameraTop")
            if self.cam is None: return None
            self.cam.enable(self.dt); self.step(2)
        path = os.path.join(ROOT, "results_mcp", "nao_camera.png"); os.makedirs(os.path.dirname(path), exist_ok=True)
        self.cam.saveImage(path, 100)
        with open(path, "rb") as f: return f.read()

def find_motion_dir() -> str:
    """Webots' NAO motion files: WEBOTS_HOME/projects/robots/softbank/nao/motions (or webots/motions in this folder)."""
    from .webots_env import find_webots_home
    homes = [os.environ.get("WEBOTS_HOME", ""), find_webots_home() or ""]
    cands = [os.path.join(h, "projects", "robots", "softbank", "nao", "motions") for h in homes if h] + [os.path.join(ROOT, "webots", "motions")]
    for c in cands:
        if os.path.isfile(os.path.join(c, "Forwards.motion")): return c
    raise RuntimeError("NAO motion files not found. They come with Webots in projects/robots/softbank/nao/motions; set WEBOTS_HOME in .env "
                       "to your Webots folder, or copy the .motion files into webots/motions/.")

# --------------------------------------------------------------------------------------------------------------- guard
class HumanoidGuard:
    """Vetoes a walking motion whose predicted path (conservative reach, from the REPORTED pose) leaves the house or comes closer than
    ROBOT_R + MARGIN to a wall or furniture of the map, a bumped obstacle or a no-go zone. A robot that is already that close (after a
    bump) may only move so that it gets no closer. It never reads the simulator's state."""
    @staticmethod
    def _gap(px, py, o):                                         # distance outside the box, per axis (negative = inside)
        return max(abs(px - o["x"]) - o["sx"] / 2, abs(py - o["y"]) - o["sy"] / 2)
    def check(self, motion, pose, room, objects, zones):
        if motion not in REACH: return None
        x, y, yaw = pose[0], pose[1], math.radians(pose[2]); f, l = REACH[motion]
        w, h = room; c, s = math.cos(yaw), math.sin(yaw); need = ROBOT_R + MARGIN
        for k in range(1, 9):
            px, py = x + (f * c - l * s) * k / 8, y + (f * s + l * c) * k / 8
            if not (ROBOT_R <= px <= w - ROBOT_R and ROBOT_R <= py <= h - ROBOT_R): return f"would leave the house near ({px:.2f}, {py:.2f})"
            for o in objects:
                g = self._gap(px, py, o)
                if g < need and g < self._gap(x, y, o) - 1e-9:
                    return f"would come too close to the {o['label']} at ({o['x']:.2f}, {o['y']:.2f})"
            for z in zones:
                d = math.hypot(px - z["x"], py - z["y"])
                if d < z["r"] + ROBOT_R and d < math.hypot(x - z["x"], y - z["y"]) - 1e-9:
                    return f"would enter the no-go zone around ({z['x']:.2f}, {z['y']:.2f})"
        return None

# --------------------------------------------------------------------------------------------------------------- session
class HumanoidSession:
    robot_name = "NAO humanoid (Webots Nao.proto, 25 joints, top camera)"

    def __init__(self, sim="fake", max_calls: int = 300, log_dir=None, default_scene: str = "living_room", backend=None, memory=None):
        if backend is None and sim not in ("fake", "webots"): raise ValueError("the humanoid runs in --sim fake or --sim webots")
        self.sim, self._backend, self.max_calls, self.default_scene = sim, backend, max_calls, default_scene
        self.guard = HumanoidGuard()
        self.room = (house.W, house.H)
        self.scene = None
        self.start_pose = None
        self._falls = 0
        self.cleaning = {"mode": "vacuum", "paused": False, "rooms": [], "cells": 0, "active": False}
        self.log_path = None
        if log_dir is not False:
            d = log_dir or os.path.join(ROOT, "results_mcp")
            try:
                os.makedirs(d, exist_ok=True)
                self.log_path = os.path.join(d, time.strftime("nao_session_%Y%m%dT%H%M%SZ", time.gmtime()) + f"_{os.getpid()}.jsonl")
            except OSError: self.log_path = None
        # memory: a NaoMemory, False/off, or None = the SQLite file next to the audit logs (off when log_dir is False, e.g. in tests)
        if memory is None and log_dir is not False:
            from .nao_memory import NaoMemory
            try: memory = NaoMemory(os.path.join(log_dir or os.path.join(ROOT, "results_mcp"), "nao_memory.db"))
            except Exception: memory = None
        self.memory = memory or None
        self._reset()

    @property
    def nao(self):
        if self._backend is None: self._backend = FakeNao() if self.sim == "fake" else WebotsNao()
        return self._backend

    def _reset(self):
        self.bumps, self.zones, self.stopped, self.last_veto = [], [], False, None
        self.accuracy, self.replan_log, self.detections, self.covered, self.last_task, self.sweep_skipped = [], [], [], set(), None, 0
        self.replans = 0
        self.sonar_obs = []
        self._dbg = {"target": None, "route": [], "next": None, "task": None, "plan": []}
        self.last_plan, self._plan_pattern = None, "quick"
        self.distance, self.motions, self.collisions, self.tasks, self.t0 = 0.0, 0, 0, [], time.time()
        self.calls = {"total": 0, "ok": 0, "rejected": 0, "vetoed": 0, "failed": 0, "error": 0}

    def robot_tools(self): return list(HUMANOID_TOOLS)

    def load_scene(self, scene: str = "living_room") -> dict:
        """Stand the robot at the start point of a room of the house (also the only restart after stop_robot)."""
        if scene not in house.ROOMS: raise ValueError(f"unknown room {scene!r}; choose from {sorted(house.ROOMS)}")
        x, y, yaw = house.ROOMS[scene][0]
        self.nao.start(x, y, yaw)
        self.scene = scene
        self.start_pose = (x, y, yaw)
        self.cleaning = {"mode": "vacuum", "paused": False, "rooms": [], "cells": 0, "active": False}
        self.nao.set_brush_mode("vacuum")
        self._reset()
        out = {"scene": scene, "simulator": self.nao.name, "house_m": list(self.room), "rooms": sorted(house.ROOMS), "pose": self._pose()}
        self._mem("position", room=out["pose"]["room"], x=out["pose"]["x"], y=out["pose"]["y"], yaw=out["pose"]["yaw_deg"], note=f"start of scene {scene}")
        self._log({"event": "load_scene", **out}); return out

    def _ensure(self):
        if self.scene is None: self.load_scene(self.default_scene)

    def _pose(self):
        x, y, yaw, fallen = self.nao.pose()
        return {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw, 1), "fallen": bool(fallen), "room": house.room_of(x, y)}

    def known_obstacles(self):
        self.sonar_obs = [o for o in self.sonar_obs if o["expires"] > self.motions]        # a sonar echo is a hint, not a map fact: it expires
        return house.obstacles() + list(self.bumps) + [{k: v for k, v in o.items() if k != "expires"} for o in self.sonar_obs]

    # ---------------------------------------------------------------------------------------------------------- the one path
    def robot_call(self, tool, args=None) -> dict:
        args = {} if args is None else args
        call = {"tool": tool, "args": args}
        self.calls["total"] += 1
        if self.calls["total"] > self.max_calls:
            return self._finish(call, {"status": "rejected", "message": f"call budget of {self.max_calls} for this scene is used up; load_scene starts a new one"})
        # Connection health + safety ordering: the stop-latch and argument validation must fire even if the backend is gone,
        # but a real connection loss must still surface as a clean {status:error} (never an exception) for calls that would proceed.
        conn_err = None
        try:
            self._ensure()
            fallen = self._pose()["fallen"]
        except RuntimeError as e:
            conn_err, fallen = e, False      # backend unreachable: assume upright for validation; report the loss after safety checks
        errs = validate_humanoid_call(tool, args, self.stopped, fallen)
        if errs: return self._finish(call, {"status": "rejected", "message": "; ".join(errs)})
        if conn_err is not None:
            return self._finish(call, {"status": "error", "message": f"the robot connection was lost ({conn_err}); "
                                       "restart the Webots house (start_webots.ps1) and start a new MCP session"})
        try:
            res = getattr(self, "_t_" + tool)(**args)
        except RuntimeError as e:
            res = {"status": "error", "message": str(e)}
        if HUMANOID_TOOLS[tool].category in ("motion", "macro", "effector"):
            self._dbg["task"] = {"tool": tool, "args": dict(args), "status": res["status"], "message": str(res.get("message", ""))[:200]}
            self.last_task = {"tool": tool, "args": dict(args), "status": res["status"], "goal": res.get("goal"), "data": res.get("data")}
            if tool not in ("walk_forward", "walk_backward", "turn", "side_step", "wave"):       # whole tasks, not single motions
                self.tasks.append({"tool": tool, "status": res["status"], "verified": self._verify(self.last_task)["passed"]})
        return self._finish(call, res)

    def _motion(self, motion) -> dict:
        """One motion: guard, obstacle sonar, play, read the pose back, check the result (turn angle, progress), log the accuracy."""
        before = self._pose()
        why = self.guard.check(motion, (before["x"], before["y"], before["yaw_deg"]), self.room, self.known_obstacles(), self.zones)
        if not why and motion == "forward": why = self._sonar_check(before)
        if why: self.last_veto = why; return {"status": "vetoed", "message": f"safety guard: {why}", "data": before}
        self.nao.play(motion)
        after = self._pose()
        if after["fallen"]: return {"status": "failed", "message": f"{motion}: the robot fell (call stand_up)", "data": after}
        rec = self._record(motion, before, after)
        self.motions += 1; self.distance += math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        self._mark_coverage(before, after)
        if motion in TURN_DEG and rec["error_deg"] > abs(TURN_DEG[motion]) / 2:
            return {"status": "failed", "message": f"{motion}: turned {rec['measured_deg']} degrees instead of {TURN_DEG[motion]}", "data": after}
        if motion in REACH and rec["moved_m"] < 0.02:
            self._note_bump(before, motion); return {"status": "failed", "message": f"{motion}: no progress (blocked)", "data": after}
        return {"status": "ok", "message": motion, "data": after}

    # ---------------------------------------------------------------------------------------------------------- sensing and checks
    def _ray(self, x, y, bearing_deg, max_d=2.5):
        """Distance along a ray to the nearest wall, furniture or bumped obstacle of the believed map: (metres, label) or (None, None)."""
        c, s = math.cos(math.radians(bearing_deg)), math.sin(math.radians(bearing_deg)); boxes = self.known_obstacles()
        for k in range(1, int(max_d / 0.05) + 1):
            d = k * 0.05; px, py = x + d * c, y + d * s
            for o in boxes:
                if abs(px - o["x"]) <= o["sx"] / 2 and abs(py - o["y"]) <= o["sy"] / 2: return d, o["label"]
        return None, None

    def _sonar_check(self, pose):
        """The sonar sees something close ahead that the map does not explain: remember it as an obstacle and refuse the step."""
        d = self.nao.sonar() if hasattr(self.nao, "sonar") else None
        if d is None or d >= SONAR_STOP: return None
        cone = [self._ray(pose["x"], pose["y"], pose["yaw_deg"] + a)[0] for a in range(-45, 46, 5)]      # the sonar beam is wide: a wall, door jamb or furniture off-axis answers too
        if any(e is not None and e <= d + 0.3 for e in cone): return None            # the map explains it: the guard already judged it
        yaw = math.radians(pose["yaw_deg"])
        ex, ey = pose["x"] + (d + 0.1) * math.cos(yaw), pose["y"] + (d + 0.1) * math.sin(yaw)
        for x1, y1, x2, y2 in house.WALLS:                                              # an echo beside a mapped wall or door jamb is the wall, not a new obstacle
            px, py = min(max(ex, min(x1, x2)), max(x1, x2)), min(max(ey, min(y1, y2)), max(y1, y2))
            if math.hypot(ex - px, ey - py) <= 0.65: return None
        ob = {"label": "unknown obstacle (sonar)", "x": round(ex, 2), "y": round(ey, 2), "sx": 0.3, "sy": 0.3}
        self.sonar_obs.append({**ob, "expires": self.motions + 30}); self.detections.append({**ob, "distance_m": round(d, 2)}); del self.detections[:-20]
        self._mem("obstacle", label=ob["label"], room=house.room_of(ob["x"], ob["y"]), x=ob["x"], y=ob["y"], note=f"sonar echo at {round(d, 2)} m")
        return f"obstacle detected {d:.2f} m ahead at ({ob['x']}, {ob['y']}) that is not on the map"

    def _record(self, motion, before, after):
        if motion in TURN_DEG:
            got = _wrap(after["yaw_deg"] - before["yaw_deg"])
            rec = {"motion": motion, "expected_deg": TURN_DEG[motion], "measured_deg": round(got, 1), "error_deg": round(abs(_wrap(got - TURN_DEG[motion])), 1)}
        elif motion in REACH:
            rec = {"motion": motion, "moved_m": round(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 3),
                   "heading_drift_deg": round(abs(_wrap(after["yaw_deg"] - before["yaw_deg"])), 1)}
        else: return {}
        self.accuracy.append(rec); del self.accuracy[:-200]
        return rec

    def accuracy_report(self):
        turns = [r for r in self.accuracy if "error_deg" in r]; walks = [r for r in self.accuracy if "moved_m" in r]
        out = {"samples": len(self.accuracy), "kinematic_fallbacks": getattr(self.nao, "fallbacks", 0)}
        if turns:
            errs = [r["error_deg"] for r in turns]
            out["turning"] = {"count": len(turns), "mean_error_deg": round(sum(errs) / len(errs), 1), "max_error_deg": max(errs), "tolerance_deg": TURN_TOL,
                              "pass": sum(errs) / len(errs) <= TURN_TOL}
        if walks:
            mv, dr = [r["moved_m"] for r in walks], [r["heading_drift_deg"] for r in walks]
            out["walking"] = {"count": len(walks), "mean_moved_m": round(sum(mv) / len(mv), 3), "min_moved_m": min(mv), "max_moved_m": max(mv),
                              "mean_heading_drift_deg": round(sum(dr) / len(dr), 1), "tolerance_deg": DRIFT_TOL, "pass": sum(dr) / len(dr) <= DRIFT_TOL}
        out["pass"] = None if not (turns or walks) else all(out[k]["pass"] for k in ("turning", "walking") if k in out)
        return out

    def _t_scan_obstacles(self):
        p = self._pose(); rays = []
        for rel in (-90, -45, 0, 45, 90):
            d, what = self._ray(p["x"], p["y"], p["yaw_deg"] + rel)
            rays.append({"bearing_deg": rel, "distance_m": d, "what": what})                # left positive; None = nothing within 2.5 m
        sonar = self.nao.sonar() if hasattr(self.nao, "sonar") else None
        near = [r for r in rays if r["distance_m"] is not None]
        data = {"rays": rays, "sonar_ahead_m": None if sonar is None else round(sonar, 2), "nearest": min(near, key=lambda r: r["distance_m"]) if near else None,
                "unmapped_detections": list(self.detections), "source": "house map + bumps (rays), live sonar ahead"}
        return {"status": "ok", "message": "obstacle scan (distances from the robot's centre)", "data": data}

    def _t_motion_accuracy(self):
        return {"status": "ok", "message": "walking and turning accuracy of the motions so far", "data": self.accuracy_report()}

    def _free_cells(self):
        """Floor cells (CELL grid) not under furniture that the brush can reach from a place the robot may stand on: cell -> room."""
        if not hasattr(self, "_cells"):
            boxes = house.obstacles(); w, h = self.room; ni, nj = int(w / CELL), int(h / CELL)
            def free(i, j): return not any(abs((i + 0.5) * CELL - o["x"]) <= o["sx"] / 2 and abs((j + 0.5) * CELL - o["y"]) <= o["sy"] / 2 for o in boxes)
            stand = {(i, j) for i in range(ni) for j in range(nj)
                     if all(self.guard._gap((i + 0.5) * CELL, (j + 0.5) * CELL, o) >= self.CLEAR_TIGHT for o in boxes)}
            reach = int((BRUSH_D + BRUSH_R) / CELL) + 1; self._cells = {}
            for i in range(ni):
                for j in range(nj):
                    if free(i, j) and any((i + a, j + b) in stand and math.hypot(a, b) * CELL <= BRUSH_D + BRUSH_R
                                          for a in range(-reach, reach + 1) for b in range(-reach, reach + 1)):
                        self._cells[(i, j)] = house.room_of((i + 0.5) * CELL, (j + 0.5) * CELL)
        return self._cells

    def _mark_coverage(self, before, after):
        """While a cleaning job runs, every floor cell under the brush (a disc BRUSH_D ahead of the robot) counts as cleaned."""
        if not self.cleaning["active"] or self.cleaning["paused"]: return
        cells = self._free_cells(); yaw = math.radians(after["yaw_deg"])
        for k in range(6):
            t = k / 5; bx = before["x"] + (after["x"] - before["x"]) * t + BRUSH_D * math.cos(yaw); by = before["y"] + (after["y"] - before["y"]) * t + BRUSH_D * math.sin(yaw)
            for i in range(int((bx - BRUSH_R) / CELL), int((bx + BRUSH_R) / CELL) + 1):
                for j in range(int((by - BRUSH_R) / CELL), int((by + BRUSH_R) / CELL) + 1):
                    if (i, j) in cells and math.hypot((i + 0.5) * CELL - bx, (j + 0.5) * CELL - by) <= BRUSH_R + CELL * 0.35: self.covered.add((i, j))

    def coverage_report(self, pattern="quick"):
        cells = self._free_cells(); out = {}; need = COVERAGE_OK[pattern]
        for room in house.ROOMS:
            total = [c for c, r in cells.items() if r == room]; done = [c for c in total if c in self.covered]
            out[room] = {"covered": round(len(done) / len(total), 3) if total else None, "ok": bool(total) and len(done) / len(total) >= need}
        allc = len(cells)
        return {"rooms": out, "house_covered": round(len(self.covered) / allc, 3) if allc else None, "required_per_room": need}

    def brush_check(self):
        p = self._pose(); yaw = math.radians(p["yaw_deg"]); exp = (p["x"] + BRUSH_D * math.cos(yaw), p["y"] + BRUSH_D * math.sin(yaw))
        got = self.nao.brush_pos() if hasattr(self.nao, "brush_pos") else None
        if got is None: return {"expected_xy": [round(v, 3) for v in exp], "measured_xy": None, "ok": False, "detail": "no brush in this simulator world"}
        err = math.hypot(got[0] - exp[0], got[1] - exp[1])
        return {"expected_xy": [round(v, 3) for v in exp], "measured_xy": [round(v, 3) for v in got], "error_m": round(err, 3), "tolerance_m": BRUSH_TOL, "ok": err <= BRUSH_TOL}

    def _t_verify_cleaning(self):
        b, cov = self.brush_check(), self.coverage_report()
        ok = b["ok"] and cov["house_covered"] is not None
        return {"status": "ok", "message": "brush position and cleaning coverage", "data": {"brush": b, "coverage": cov, "brush_ok": ok}}

    def _t_verify_task(self):
        """Check the outcome of the last motion/macro against what the robot reports, not against the tool's own status."""
        if self.last_task is None: return {"status": "rejected", "message": "no task has been run yet"}
        v = self._verify(self.last_task)
        return {"status": "ok", "message": f"task {v['task']} {'verified' if v['passed'] else 'NOT verified'}", "data": v}

    def _verify(self, t):
        p = self._pose(); checks = [{"check": "tool reported ok", "ok": t["status"] == "ok", "detail": t["status"]}, {"check": "robot upright", "ok": not p["fallen"], "detail": p["fallen"]}]
        if t.get("goal"):
            gx, gy, tol = t["goal"]; d = math.hypot(p["x"] - gx, p["y"] - gy)
            checks.append({"check": "at the goal", "ok": d <= tol, "detail": f"{d:.2f} m from ({gx:.2f}, {gy:.2f}), tolerance {tol} m"})
        if t["tool"] == "tour_house":
            visited = set((t.get("data") or {}).get("visited", [])); complete = visited == set(house.ROOMS)
            checks.append({"check": "every room visited", "ok": complete, "detail": sorted(visited)})
            if t["args"].get("finish", "stay") == "start" and self.start_pose is not None:
                d = math.hypot(p["x"] - self.start_pose[0], p["y"] - self.start_pose[1])
                checks.append({"check": "returned to start", "ok": d <= 0.6, "detail": f"{d:.2f} m from start"})
        if t["tool"] == "join_tv_watching":
            spot = (1.6, 1.65)
            distance = math.hypot(p["x"] - spot[0], p["y"] - spot[1])
            tv = next(item for item in house.ITEMS if item["label"] == "television")
            bearing = abs(_wrap(math.degrees(math.atan2(tv["y"] - p["y"], tv["x"] - p["x"])) - p["yaw_deg"]))
            checks.extend((
                {"check": "at the safe sofa-side viewing spot", "ok": distance <= 0.6, "detail": f"{distance:.2f} m from the target"},
                {"check": "facing the television", "ok": bearing <= 18, "detail": f"{bearing:.1f} degree bearing error"},
            ))
        if t["tool"] == "turn_to_object":
            item = next(item for item in house.ITEMS if item["label"] == t["args"]["name"])
            bearing = abs(_wrap(math.degrees(math.atan2(item["y"] - p["y"], item["x"] - p["x"])) - p["yaw_deg"]))
            checks.append({"check": f"facing {item['label']}", "ok": bearing <= 18, "detail": f"{bearing:.1f} degree bearing error"})
        if t["tool"] in ("clean_house", "clean_room"):
            pat = t["args"].get("pattern", "quick")
            cov = self.coverage_report(pat); bad = [r for r, v in cov["rooms"].items() if not v["ok"]]
            if t["tool"] == "clean_room":
                room = t["args"]["room"]
                bad = [room] if not cov["rooms"][room]["ok"] else []
                checks.append({"check": f"in the {room}", "ok": p["room"] == room, "detail": p["room"]})
            target = f"the {t['args']['room']}" if t["tool"] == "clean_room" else "every room"
            checks.append({"check": f"{target} at least {int(COVERAGE_OK[pat] * 100)}% covered ({pat} sweep)",
                           "ok": not bad, "detail": {r: v["covered"] for r, v in cov["rooms"].items()}})
        if t["tool"] == "run_plan" and self.last_plan:
            miss = [s["do"] for s in self.last_plan["steps"] if not s["met"]]
            checks.append({"check": "every planned step met its completion condition", "ok": not miss, "detail": miss or f"{len(self.last_plan['steps'])} step(s)"})
        if t["tool"] in ("clean", "clean_room", "clean_house", "clean_corners", "clean_direction"):
            b = self.brush_check(); checks.append({"check": "brush in place", "ok": b["ok"], "detail": b.get("error_m", b.get("detail"))})
        passed = all(c["ok"] for c in checks)
        return {"task": t["tool"], "passed": passed, "checks": checks}

    def metrics(self) -> dict:
        """Numbers for benchmarking a session: task success, distance, time, cleaned area and coverage, collisions, replans."""
        cells = self._free_cells(); done = len(self.covered); n = len(self.tasks); ok = sum(1 for t in self.tasks if t["status"] == "ok" and t["verified"])
        acc = self.accuracy_report()
        return {"scenario": self.scene, "simulator": self.nao.name, "tasks": n, "tasks_verified": ok, "success_rate": round(ok / n, 3) if n else None,
                "calls": self.calls["total"], "calls_ok": self.calls["ok"], "motions": self.motions, "distance_m": round(self.distance, 2),
                "wall_time_s": round(time.time() - self.t0, 2), "cleaned_area_m2": round(done * CELL * CELL, 2),
                "coverage_pct": round(100 * done / len(cells), 1) if cells else 0.0, "collisions": self.collisions, "sonar_detections": len(self.detections),
                "vetoes": self.calls["vetoed"], "replans": self.replans, "turn_error_deg": acc.get("turning", {}).get("mean_error_deg"),
                "heading_drift_deg": acc.get("walking", {}).get("mean_heading_drift_deg"), "fallbacks": acc["kinematic_fallbacks"]}

    def export_csv(self, path, **extra) -> str:
        """Append one metrics row to a CSV file (header written once); `extra` columns such as model or run id come first."""
        row = {**extra, **self.metrics()}
        exists = os.path.exists(path) and os.path.getsize(path) > 0
        import csv
        with open(path, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(row)); 
            if not exists: w.writeheader()
            w.writerow(row)
        return path

    def _note_bump(self, pose, motion):
        self.collisions += 1
        f, l = REACH[motion]; n = math.hypot(f, l); yaw = math.radians(pose["yaw_deg"])
        dx, dy = (f * math.cos(yaw) - l * math.sin(yaw)) / n, (f * math.sin(yaw) + l * math.cos(yaw)) / n
        x, y = pose["x"] + 0.3 * dx, pose["y"] + 0.3 * dy
        self.bumps.append({"label": "unknown obstacle (bump)", "x": round(x, 2), "y": round(y, 2), "sx": 0.2, "sy": 0.2})
        self._mem("obstacle", label="unknown obstacle (bump)", room=house.room_of(x, y), x=x, y=y, note=f"blocked {motion}")
    def _repeat(self, motion, steps):
        r = None
        for i in range(steps):
            r = self._motion(motion)
            if r["status"] != "ok": r["message"] = f"step {i + 1} of {steps}: {r['message']}"; return r
        return {"status": "ok", "message": f"{motion} x{steps}", "data": r["data"]}

    def _t_walk_forward(self, steps): return self._repeat("forward", steps)
    def _t_walk_backward(self, steps): return self._repeat("backward", steps)
    def _t_side_step(self, direction): return self._motion("side_" + direction)
    def _t_turn(self, direction, degrees):
        return self._motion("turn_left_180" if degrees == 180 else f"turn_{direction}_{degrees}")
    def _t_wave(self): return self._motion("wave")
    def _t_greet(self): return self._motion("wave")
    def _t_turn_to_object(self, name):
        item = next(it for it in house.ITEMS if it["label"] == name)
        result = self._face_point(item["x"], item["y"], name)
        if result["status"] != "ok": return result
        return {"status": "ok", "message": f"turned to face {name}",
                "data": {"object": name, "pose": self._pose(),
                         "bearing_error_deg": result["data"]["bearing_error_deg"]}}

    def _t_look_around(self):
        """Turn in place through a full circle, collecting every house item seen at any heading (the camera FOV is only 61 deg,
        so one describe_view misses most of the room). Returns the robot to its starting heading. Each turn is guard-checked."""
        start_yaw = self._pose()["yaw_deg"]
        seen = {}                                                   # label -> nearest sighting
        headings = []
        def collect():
            p = self._pose()
            headings.append(p["yaw_deg"])
            for it in self.visible_items():
                prev = seen.get(it["label"])
                if prev is None or it["distance_m"] < prev["distance_m"]:
                    seen[it["label"]] = it
        collect()
        turned = 0
        for _ in range(6):                                          # 6 x 60 deg = 360 deg
            r = self._motion("turn_left_60")
            if r["status"] != "ok":
                # A turn was blocked/failed: report what we saw so far rather than forcing it.
                return {"status": "failed", "message": f"look_around stopped after {turned} deg: {r['message']}",
                        "data": {"items": sorted(seen.values(), key=lambda o: o["distance_m"]), "headings_deg": headings, "pose": self._pose()}}
            turned += 60
            collect()
        items = sorted(seen.values(), key=lambda o: o["distance_m"])
        room = self._pose()["room"]
        return {"status": "ok", "message": f"{len(items)} item(s) seen while turning full circle in the {room}",
                "data": {"items": items, "count": len(items), "room": room, "start_yaw_deg": start_yaw, "pose": self._pose()}}

    def _t_join_tv_watching(self):
        """Take a clear place beside the sofa and face the television."""
        spot = (1.6, 1.65)
        result = self._walk_path(spot, "a safe place beside the sofa to watch television")
        if result["status"] != "ok": return result
        tv = next(item for item in house.ITEMS if item["label"] == "television")
        facing = self._face_point(tv["x"], tv["y"], "the television")
        if facing["status"] != "ok": return facing
        return {"status": "ok", "message": "moved beside the sofa and faced the television",
                "data": {"television": "television", "pose": self._pose(),
                         "screen_bearing_error_deg": facing["data"]["bearing_error_deg"]}}

    def _face_point(self, x, y, label, tolerance=18):
        for _ in range(2):
            p = self._pose()
            bearing = _wrap(math.degrees(math.atan2(y - p["y"], x - p["x"])) - p["yaw_deg"])
            if abs(bearing) <= tolerance:
                return {"status": "ok", "data": {"bearing_error_deg": round(bearing, 1)}}
            for motion in self.turn_motions(bearing):
                result = self._motion(motion)
                if result["status"] != "ok": return result
        p = self._pose()
        bearing = _wrap(math.degrees(math.atan2(y - p["y"], x - p["x"])) - p["yaw_deg"])
        if abs(bearing) > tolerance:
            return {"status": "failed", "message": f"could not face {label}: {bearing:.1f} degree bearing error",
                    "data": {"pose": p, "bearing_error_deg": round(bearing, 1)}}
        return {"status": "ok", "data": {"bearing_error_deg": round(bearing, 1)}}
    def _brush_pass(self):
        """Count the selected floor attachment at the current pose without playing a hand-wiping motion."""
        p = self._pose()
        self._mark_coverage(p, p)
        room = self._pose()["room"]
        if room and room not in self.cleaning["rooms"]: self.cleaning["rooms"].append(room)
        return None

    def _t_clean(self, mode):
        self._set_cleaning_mode(mode)
        self.cleaning["active"] = True
        try:
            self._brush_pass()
            self.cleaning["cells"] += 1
        finally:
            self.cleaning["active"] = False
        return {"status": "ok", "message": f"{mode} brush pass complete in the {self._pose()['room']}", "data": self.clean_status()}

    def _set_cleaning_mode(self, mode):
        if mode not in (None, "vacuum", "mop"): raise ValueError(f"unknown cleaning brush mode {mode!r}")
        if mode is not None and mode != self.cleaning["mode"]:
            self.nao.set_brush_mode(mode)
            self.cleaning["mode"] = mode
        elif mode is not None:
            self.cleaning["mode"] = mode

    def _t_clean_room(self, room, pattern="quick", mode=None):
        if self.cleaning["paused"]:
            return {"status": "rejected", "message": "cleaning is paused; call resume_cleaning first", "data": self.clean_status()}
        self._set_cleaning_mode(mode)
        already_swept = room in self.cleaning.setdefault("swept", [])
        coverage = self.coverage_report(pattern)["rooms"][room]
        if already_swept and coverage["ok"]:
            return {"status": "ok", "message": f"the {room.replace('_', ' ')} already meets {pattern} coverage",
                    "data": {"room": room, "pattern": pattern, "coverage": coverage, **self.clean_status()}}
        arrival = self._t_go_to_room(room)
        if arrival["status"] != "ok": return arrival
        self.cleaning["active"] = True
        try:
            result = self._sweep_room(room, pattern)
        finally:
            self.cleaning["active"] = False
        if result is not None: return result
        coverage = self.coverage_report(pattern)["rooms"][room]
        pct = "unknown" if coverage["covered"] is None else f"{round(100 * coverage['covered'])}%"
        required = round(100 * COVERAGE_OK[pattern])
        status = "ok" if coverage["ok"] else "failed"
        message = f"swept the {room.replace('_', ' ')} ({pct} floor coverage; requires {required}%)"
        return {"status": status, "message": message,
                "data": {"room": room, "pattern": pattern, "coverage": coverage, **self.clean_status()}}

    def _t_clean_status(self):
        return {"status": "ok", "message": "cleaning status", "data": self.clean_status()}

    def _t_pause_cleaning(self):
        self.cleaning["paused"] = True
        return {"status": "ok", "message": "cleaning paused", "data": self.clean_status()}

    def _t_resume_cleaning(self):
        self.cleaning["paused"] = False
        return {"status": "ok", "message": "cleaning resumed", "data": self.clean_status()}

    def _t_clean_house(self, pattern="quick", mode=None):
        """Sweep every room with the brush down: parallel rows (spacing from SWEEP_SPACING) inside the room, replanned around obstacles.
        Rows that cannot be reached are skipped and counted; the coverage grid says what was really cleaned."""
        if self.cleaning["paused"]:
            return {"status": "rejected", "message": "cleaning is paused; call resume_cleaning first", "data": self.clean_status()}
        self._set_cleaning_mode(mode)
        self.cleaning["active"] = True
        try:
            for room in house.ROOMS:
                if room in self.cleaning.setdefault("swept", []) and self.coverage_report(pattern)["rooms"][room]["ok"]: continue
                r = self._sweep_room(room, pattern)
                if r is not None: return r
        finally:
            self.cleaning["active"] = False
        cov = self.coverage_report()
        incomplete = [room for room, report in cov["rooms"].items() if not report["ok"]]
        status = "failed" if incomplete else "ok"
        detail = f"; below coverage target: {', '.join(incomplete)}" if incomplete else ""
        return {"status": status, "message": f"house cleaning {'incomplete' if incomplete else 'complete'} ({pattern} sweep, {int(100 * (cov['house_covered'] or 0))}% of the free floor covered){detail}",
                "data": {**self.clean_status(), "coverage": cov, "skipped_sweep_points": self.sweep_skipped}}

    def _sweep_room(self, room, pattern):
        """Sweep one room with the selected floor brush. -> None when done, else the result that ends the task."""
        step = SWEEP_SPACING[pattern]; fails = 0; (_, (x0, y0, x1, y1), _) = house.ROOMS[room]
        rows, y, flip = [], y0 + 0.5, False
        while y <= y1 - 0.5 + 1e-9:
            xs = [x0 + 0.5 + 0.25 * k for k in range(int((x1 - x0 - 1.0) / 0.25) + 1)]
            ok = [not any(self.guard._gap(x, y, o) < self.CLEAR + (0.0 if o.get("wall") else 0.1) for o in self._boxes()) for x in xs]
            runs, cur = [], []                                   # stretches of the row that are free to stand on
            for x, good in zip(xs, ok):
                if good: cur.append(x)
                elif cur: runs.append(cur); cur = []
            if cur: runs.append(cur)
            runs = [r_ for r_ in runs if len(r_) >= 3]           # a short stretch is a pocket beside furniture: not worth walking into
            self.sweep_skipped += ok.count(False)
            for run in (runs[::-1] if flip else runs):
                ends = [(run[0], y), (run[-1], y)]; rows += ends[::-1] if flip else ends
            y += step; flip = not flip
        for pt in rows:
            r = self._walk_path(pt, f"sweep point ({pt[0]:.1f}, {pt[1]:.1f}) in the {room.replace('_', ' ')}")
            if r["status"] == "ok": fails = 0; continue
            if self.stopped or self._pose()["fallen"] or r["status"] == "error": return r
            self.sweep_skipped += 1
            if r["status"] == "rejected": continue               # no safe path (a point beside furniture): skip it
            fails += 1
            if fails >= 3: self.cleaning["active"] = False; return {**r, "message": f"cleaning aborted in the {room}: 3 sweep points in a row failed ({r['message']})"}
        r = self._brush_pass()
        if r is not None: return r
        self.cleaning["swept"].append(room); self.cleaning["cells"] += 1
        return None

    def _t_clean_corners(self, room="all", mode=None):
        """Clean every corner of a room (or of the whole house): walk to a free point 0.5-0.8 m diagonally in from the corner, face the corner,
        make a pass with the selected floor attachment. Corners blocked by furniture are listed, not silently skipped."""
        if self.cleaning["paused"]:
            return {"status": "rejected", "message": "cleaning is paused; call resume_cleaning first", "data": self.clean_status()}
        self._set_cleaning_mode(mode)
        self.cleaning["active"] = True
        done, blocked = self.cleaning.setdefault("corners", []), []
        try:
            for name, (_, (x0, y0, x1, y1), _) in house.ROOMS.items():
                if room not in ("all", name): continue
                for cx, cy in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    tag = f"{name}@({cx:g},{cy:g})"
                    if tag in done: continue
                    sx, sy = 1 if cx == x0 else -1, 1 if cy == y0 else -1
                    spot = None
                    for d in (0.5, 0.6, 0.7, 0.8):
                        px, py = cx + sx * d, cy + sy * d
                        if not any(self.guard._gap(px, py, o) < self.CLEAR + (0.0 if o.get("wall") else 0.1) for o in self._boxes()): spot = (px, py); break
                    if spot is None: blocked.append(tag); continue
                    r = self._walk_path(spot, f"the {name.replace('_', ' ')} corner ({cx:g}, {cy:g})")
                    if r["status"] != "ok":
                        if self.stopped or self._pose()["fallen"] or r["status"] == "error": return r
                        blocked.append(tag); continue
                    p = self._pose(); b = _wrap(math.degrees(math.atan2(cy - p["y"], cx - p["x"])) - p["yaw_deg"])
                    for m in (self.turn_motions(b) if abs(b) > 10 else ()):
                        r = self._run(m, [])
                        if r["status"] != "ok": return r
                    r = self._brush_pass()
                    if r is not None: return r
                    done.append(tag)
        finally:
            self.cleaning["active"] = False
        msg = f"cleaned {len(done)} corner(s)" + (f"; blocked by furniture or no safe path: {', '.join(blocked)}" if blocked else "")
        return {"status": "ok" if done else "failed", "message": msg, "data": {**self.clean_status(), "corners_done": list(done), "corners_blocked": blocked}}

    def _t_clean_direction(self, direction, steps=1, mode=None):
        """Clean while moving in one direction with the brush down: forward and backward walk, left and right side-step."""
        if self.cleaning["paused"]:
            return {"status": "rejected", "message": "cleaning is paused; call resume_cleaning first", "data": self.clean_status()}
        self._set_cleaning_mode(mode)
        self.cleaning["active"] = True
        try:
            if direction in ("forward", "backward"): r = self._repeat(direction, min(steps, 3 if direction == "backward" else 6))
            else:
                r = {"status": "ok"}
                for _ in range(steps):
                    r = self._motion("side_" + direction)
                    if r["status"] != "ok": break
        finally: self.cleaning["active"] = False
        if r["status"] == "ok": r = {**r, "message": f"cleaned {steps} step(s) {direction} with the brush down; {r.get('message', '')}".strip()}
        return r

    # ---------------------------------------------------------------------------------------------------------- several targets in one job
    def _node(self, n, s, pattern, here):
        """One requested step as a plan node: where to stand, what to do there, and the condition that says it is done."""
        do = s.get("do", "visit")
        if "room" in s:
            kind, name, label = "room", s["room"], s["room"].replace("_", " ")
            spot, item = tuple(house.ROOMS[name][0][:2]), None
            words = {"visit": f"go to the {label}", "clean": f"sweep the {label} ({pattern})", "wave": f"go to the {label} and wave"}[do]
            if do == "clean": done = f"the brush has covered at least {int(COVERAGE_OK[pattern] * 100)}% of the {label} floor"
            else: done = f"robot inside the {label} and within 0.5 m of its entry point" + (", wave played" if do == "wave" else "")
        elif "object" in s:
            kind, name, label = "object", s["object"], s["object"].replace("_", " ")
            found = self._object_spot(name, here)
            spot, item = found if found else (None, None)
            words = {"visit": f"go to the {label}", "clean": f"clean beside the {label}", "wave": f"go to the {label} and wave"}[do]
            done = f"robot within 0.5 m of the spot beside the {label}" + (", brush pass done" if do == "clean" else ", wave played" if do == "wave" else "")
        else:
            kind, name = "point", (round(float(s["x"]), 2), round(float(s["y"]), 2)); label = f"({name[0]}, {name[1]})"
            spot, item = name, None
            words = {"visit": f"go to {label}", "clean": f"clean at {label}", "wave": f"go to {label} and wave"}[do]
            done = f"robot within 0.5 m of {label}" + (", brush pass done" if do == "clean" else ", wave played" if do == "wave" else "")
        return {"i": n, "kind": kind, "name": name, "do": do, "label": label, "spot": spot, "item": item, "action": words, "done_when": done}

    def _leg(self, a, b, cache):
        """Walking length from a to b on the house map (A*), or a straight-line estimate when the planner finds no path."""
        key = (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))
        if key not in cache:
            path = house.plan_path(a, b, self._boxes(), self.CLEAR) or house.plan_path(a, b, self._boxes(), self.CLEAR_TIGHT)
            cache[key] = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(path, path[1:])) if path else 2.0 * math.hypot(b[0] - a[0], b[1] - a[1]) + 2.0
        return cache[key]

    @staticmethod
    def _best_order(D, n, closed):
        """Shortest visiting order of nodes 1..n from node 0 (the robot), optionally ending back at node 0. Exact for n <= 8, else nearest
        neighbour improved by 2-opt."""
        cost = lambda o: sum(D[a][b] for a, b in zip((0,) + o, o)) + (D[o[-1]][0] if closed and o else 0.0)
        if n <= 8: return min(itertools.permutations(range(1, n + 1)), key=cost)
        left, o = set(range(1, n + 1)), []
        while left:
            nxt = min(left, key=lambda j: D[o[-1] if o else 0][j]); o.append(nxt); left.discard(nxt)
        o, better = tuple(o), True
        while better:
            better = False
            for i in range(n - 1):
                for j in range(i + 1, n):
                    cand = o[:i] + o[i:j + 1][::-1] + o[j + 1:]
                    if cost(cand) < cost(o) - 1e-9: o, better = cand, True
        return o

    def build_plan(self, steps, order="optimize", finish="stay", pattern="quick") -> dict:
        """Break a request into ordered, checkable steps. Repeated steps are merged, a visit to a room that is also swept is dropped,
        steps with no safe route are reported, and the rest are ordered to keep the walking short (order = optimize) or kept as given."""
        self._ensure(); p = self._pose(); here = (p["x"], p["y"]); skipped, nodes, seen = [], [], set()
        swept = {s["room"] for s in steps if "room" in s and s.get("do") == "clean"}
        for n, s in enumerate(steps, 1):
            key = json.dumps(s, sort_keys=True)
            if key in seen: skipped.append({"i": n, "step": s, "why": "same as an earlier step"}); continue
            seen.add(key)
            if "room" in s and s.get("do", "visit") == "visit" and s["room"] in swept: skipped.append({"i": n, "step": s, "why": f"the {s['room']} is swept anyway"}); continue
            nd = self._node(n, s, pattern, here)
            if nd["spot"] is None: skipped.append({"i": n, "step": s, "why": f"no reachable free place next to a {nd['name']}"}); continue
            if house.plan_path(here, nd["spot"], self._boxes(), self.CLEAR) is None and house.plan_path(here, nd["spot"], self._boxes(), self.CLEAR_TIGHT) is None:
                skipped.append({"i": n, "step": s, "why": f"no safe path to {nd['label']}"}); continue
            nodes.append(nd)
        cache = {}; closed = finish == "start" and self.start_pose is not None
        pts = [here] + [nd["spot"] for nd in nodes]
        D = [[0.0 if a == b else self._leg(pts[a], pts[b], cache) for b in range(len(pts))] for a in range(len(pts))]
        given = tuple(range(1, len(nodes) + 1))
        best = self._best_order(D, len(nodes), closed) if order == "optimize" and len(nodes) > 1 else given
        length = lambda o: sum(D[a][b] for a, b in zip((0,) + o, o)) + (self._leg(pts[o[-1]], self.start_pose[:2], cache) if closed and o else 0.0)
        ordered, prev = [], 0
        for k in best:
            ordered.append({**nodes[k - 1], "leg_m": round(D[prev][k], 2)}); prev = k
        return {"order": order, "finish": "start" if closed else "stay", "pattern": pattern, "steps": ordered, "skipped": skipped,
                "walk_m": round(length(best), 1) if nodes else 0.0, "as_given_m": round(length(given), 1) if nodes else 0.0}

    def _plan_view(self, plan):
        return {**{k: v for k, v in plan.items() if k != "steps"},
                "saved_m": round(plan["as_given_m"] - plan["walk_m"], 1),
                "steps": [{"n": k, "request": s["i"], "do": s["action"], "stand_at": [round(s["spot"][0], 2), round(s["spot"][1], 2)], "walk_m": s["leg_m"],
                           "done_when": s["done_when"]} for k, s in enumerate(plan["steps"], 1)]}

    def _t_plan_tasks(self, steps, order="optimize", finish="stay", pattern="quick"):
        """Plan only: the ordered steps, their completion conditions and the walking saved by ordering. The robot does not move."""
        plan = self.build_plan(steps, order, finish, pattern); v = self._plan_view(plan)
        return {"status": "ok" if plan["steps"] else "rejected", "message": f"{len(plan['steps'])} step(s), about {plan['walk_m']} m of walking"
                + (f" ({plan['as_given_m']} m in the order given)" if order == "optimize" and plan["as_given_m"] > plan["walk_m"] else "")
                + (f"; {len(plan['skipped'])} left out" if plan["skipped"] else ""), "data": v}

    def _step_met(self, nd):
        """Independent check of a step's completion condition from the pose and coverage, not from the tool's own status."""
        p = self._pose(); sx, sy = nd["spot"]; d = math.hypot(p["x"] - sx, p["y"] - sy)
        if nd["do"] == "clean" and nd["kind"] == "room":
            c = self.coverage_report(self._plan_pattern)["rooms"][nd["name"]]
            return bool(c["ok"]), f"{c['covered']} of the floor covered, need {COVERAGE_OK[self._plan_pattern]}"
        if nd["kind"] == "room": return p["room"] == nd["name"] and d <= 0.5, f"in the {p['room']}, {d:.2f} m from the entry point"
        return d <= 0.5, f"{d:.2f} m from the spot"

    def _do_step(self, nd):
        """Run one step with the same walking, guard and replanning as the single tools."""
        if nd["do"] == "clean" and nd["kind"] == "room":
            self.cleaning["active"] = True
            try:
                if nd["name"] in self.cleaning.setdefault("swept", []): return {"status": "ok", "message": f"the {nd['label']} was already swept"}
                r = self._sweep_room(nd["name"], self._plan_pattern)
                return r if r is not None else {"status": "ok", "message": f"swept the {nd['label']}"}
            finally:
                self.cleaning["active"] = False
        if nd["kind"] == "room": r = self._t_go_to_room(nd["name"])
        elif nd["kind"] == "object": r = self._reach_object(nd["spot"], nd["item"], nd["name"])
        else: r = self._walk_path(nd["spot"], nd["label"])
        if r["status"] != "ok" or nd["do"] == "visit": return r
        if nd["do"] == "clean":
            self.cleaning["active"] = True
            try:
                self._brush_pass()
                self.cleaning["cells"] += 1
            finally:
                self.cleaning["active"] = False
            return {**r, "message": f"{r['message']}; {self.cleaning['mode']} brush pass done"}
        w = self._motion("wave")
        return r if w["status"] == "ok" else w

    def _t_run_plan(self, steps, order="optimize", finish="stay", pattern="quick"):
        """Plan the steps (see build_plan), then run them in order. A step that fails is reported and the plan goes on with the next one,
        unless the robot is stopped or has fallen. Each step is checked against its own completion condition afterwards."""
        if self.cleaning["paused"] and any(s.get("do") == "clean" for s in steps):
            return {"status": "rejected", "message": "cleaning is paused; call resume_cleaning first", "data": self.clean_status()}
        plan = self.build_plan(steps, order, finish, pattern)
        if not plan["steps"]: return {"status": "rejected", "message": "no step can be run: " + "; ".join(s["why"] for s in plan["skipped"]), "data": self._plan_view(plan)}
        self._plan_pattern = pattern; results, halted = [], False; m0 = self.motions
        self._dbg["plan"] = [{"n": k, "do": s["action"], "state": "waiting"} for k, s in enumerate(plan["steps"], 1)]
        for k, nd in enumerate(plan["steps"], 1):
            self._dbg["plan"][k - 1]["state"] = "running"; self._show()
            if halted: results.append({"n": k, "do": nd["action"], "status": "not run", "met": False, "detail": "an earlier step stopped the plan"}); self._dbg["plan"][k - 1]["state"] = "not run"; continue
            n0 = self.motions; r = self._do_step(nd); met, detail = self._step_met(nd) if r["status"] == "ok" else (False, r["message"])
            results.append({"n": k, "do": nd["action"], "status": r["status"], "met": bool(met), "detail": detail, "motions": self.motions - n0})
            self._dbg["plan"][k - 1]["state"] = "done" if met else "failed"
            if self.stopped or self._pose()["fallen"] or r["status"] == "error": halted = True
        if finish == "start" and not halted:
            r = self._t_return_to_start(); results.append({"n": len(results) + 1, "do": "return to the start", "status": r["status"], "met": r["status"] == "ok", "detail": r["message"]})
        self.cleaning["active"] = False; self.last_plan = {"steps": results}
        ok = sum(1 for r in results if r["met"]); total = len(results) + len(plan["skipped"])
        failed = [f"{r['do']} ({r['detail']})" for r in results if not r["met"]] + [f"step {s['i']} left out ({s['why']})" for s in plan["skipped"] if "earlier" not in s["why"] and "anyway" not in s["why"]]
        return {"status": "ok" if not failed else "failed",
                "message": f"{ok} of {total} step(s) completed in {self.motions - m0} motions" + (f"; not completed: {'; '.join(failed)}" if failed else ""),
                "data": {"steps": results, "plan": self._plan_view(plan), "pose": self._pose()}}

    def _t_return_to_start(self):
        if self.start_pose is None: return {"status": "rejected", "message": "no start pose is available"}
        result = self._walk_path(self.start_pose[:2], "the starting point")
        if result["status"] == "ok":
            self.cleaning["active"] = False
        return result

    def _t_tour_house(self, finish="stay"):
        visited = []
        for room in house.ROOMS:
            result = self._t_go_to_room(room)
            pose = self._pose()
            if result["status"] != "ok" or pose["room"] != room or pose["fallen"]:
                status = result["status"] if result["status"] != "ok" else "failed"
                return {"status": status, "message": f"house tour stopped at {room}: {result['message']}",
                        "data": {"visited": visited, "pose": pose}}
            visited.append(room)
        if finish == "start":
            result = self._t_return_to_start()
            if result["status"] != "ok":
                return {"status": result["status"], "message": f"visited all rooms but could not return to start: {result['message']}",
                        "data": {"visited": visited, "pose": self._pose()}}
        return {"status": "ok", "message": f"visited {len(visited)} rooms" + (" and returned to the start" if finish == "start" else ""),
                "data": {"visited": visited, "finish": finish, "pose": self._pose()}}
    def _t_get_pose(self): return {"status": "ok", "message": "pose", "data": {**self._pose(), "house_m": list(self.room)}}

    def visible_items(self):
        x, y, yaw, _ = self.nao.pose(); out = []
        for it in house.ITEMS:
            dx, dy = it["x"] - x, it["y"] - y; d = math.hypot(dx, dy); b = _wrap(math.degrees(math.atan2(dy, dx)) - yaw)
            if d <= VIEW_RANGE and abs(b) <= FOV_DEG / 2 and not house.blocks_view(x, y, it["x"], it["y"]):
                out.append({"label": it["label"], "bearing_deg": round(b, 1), "distance_m": round(d, 2), "x": it["x"], "y": it["y"],
                            "room": house.room_of(it["x"], it["y"])})
        return sorted(out, key=lambda o: o["distance_m"])

    def _t_describe_view(self):
        out = self.visible_items()
        return {"status": "ok", "message": f"{len(out)} item(s) in view (bearing: left positive)", "data": out, "perception": "map"}

    def _t_look(self):
        png = self.nao.camera_png()
        if png is None: return {"status": "failed", "message": "no camera picture in this simulator (only in Webots)"}
        return {"status": "ok", "message": "camera picture from the NAO's top camera", "_image": base64.b64encode(png).decode(), "_mime": "image/png"}

    def _t_stand_up(self):
        p = self._pose()
        self.nao.set_pose(p["x"], p["y"], p["yaw_deg"])            # supervisor reset to standing at the same place (not a get-up motion)
        return {"status": "ok", "message": "standing again (simulator reset at the same place)", "data": self._pose()}

    def _t_stop_robot(self): self.stopped = True; return {"status": "ok", "message": "stopped: every motion is refused until load_scene"}

    def clean_status(self):
        self._ensure()
        return {**self.cleaning, "progress_rooms": len(self.cleaning.get("swept", self.cleaning["rooms"])), "total_rooms": len(house.ROOMS)}

    # ---------------------------------------------------------------------------------------------------------- walking to places
    CLEAR = ROBOT_R + MARGIN + 0.10          # planner clearance: wider than the guard's, so a step that drifts a little is not vetoed
    CLEAR_TIGHT = ROBOT_R + MARGIN + 0.03    # used only when no path exists with CLEAR
    # The NAO's turn motions are 40, 60 and 180 degrees; pairs give every multiple of 20 (60 - 40 = 20), so the heading error is <= 10 deg.
    TURNS = {20: ("turn_left_60", "turn_right_40"), 40: ("turn_left_40",), 60: ("turn_left_60",), 80: ("turn_left_40", "turn_left_40"),
             100: ("turn_left_60", "turn_left_40"), 120: ("turn_left_60", "turn_left_60"), 140: ("turn_left_60", "turn_left_40", "turn_left_40"),
             160: ("turn_left_180", "turn_right_60", "turn_left_40"), 180: ("turn_left_180",)}

    @classmethod
    def turn_motions(cls, bearing):
        """Motions that turn the robot by the multiple of 20 degrees closest to `bearing` (left positive)."""
        t = int(round(_wrap(bearing) / 20.0)) * 20
        if t == 0: return ()
        if t > 0 or t == -180: return cls.TURNS[abs(t)]
        flip = {"turn_left_40": "turn_right_40", "turn_left_60": "turn_right_60", "turn_right_40": "turn_left_40", "turn_right_60": "turn_left_60",
                "turn_left_180": "turn_left_180"}
        return tuple(flip[m] for m in cls.TURNS[-t])

    def _boxes(self):
        return self.known_obstacles() + [{"x": z["x"], "y": z["y"], "sx": 2 * z["r"], "sy": 2 * z["r"]} for z in self.zones]

    def _plan(self, goal, start=None):
        p = self._pose(); start = start or (p["x"], p["y"])
        return house.plan_path(start, goal, self._boxes(), self.CLEAR) or house.plan_path(start, goal, self._boxes(), self.CLEAR_TIGHT)

    def _walk_path(self, goal, label):
        trail, replans = [], 0
        self._falls = 0
        self._dbg.update(target={"label": label, "x": round(float(goal[0]), 2), "y": round(float(goal[1]), 2)}, route=[], next=None)
        while True:
            path = self._plan(goal)
            if path is None and self.sonar_obs:
                self.sonar_obs = []; path = self._plan(goal)           # sonar echoes may be false: forget them before giving up
            if path is None:
                self._log({"event": "route", "target": self._dbg["target"], "route": None, "replans": replans, "why": "no safe path"})
                return {"status": "rejected", "message": f"no safe path to {label} on the house map (walls, furniture, no-go zones)", "data": self._pose(), "trail": trail}
            self._dbg["route"] = [(round(x, 2), round(y, 2)) for x, y in path]
            self._log({"event": "route", "target": self._dbg["target"], "route": self._dbg["route"], "replans": replans})
            r = self._follow(path[1:], trail)
            if r is None: return {"status": "ok", "message": f"reached {label} after {len(trail)} motions", "data": self._pose(), "trail": trail, "waypoints": path,
                                  "goal": (goal[0], goal[1], 0.45), "replans": replans}
            if r["status"] in ("failed", "vetoed") and ("blocked" in r["message"] or r["status"] == "vetoed") and replans < 6:
                replans += 1; self.replans += 1; self.replan_log.append(r["message"]); del self.replan_log[:-20]
                self._log({"event": "replan", "target": self._dbg["target"], "n": replans, "reason": r["message"], "pose": self._pose(), **self._obstacle_view()})
                if r["status"] == "vetoed" and replans >= 2: self._run("backward", trail)      # back away from the obstacle, then plan again
                continue                                             # a bump or sonar detection added an obstacle, or a step drifted: plan again from here
            r["message"] = f"stopped on the way to {label} after {len(trail)} motions: {r['message']}"; r["trail"] = trail; r["goal"] = (goal[0], goal[1], 0.45); r["replans"] = replans; return r

    def _run(self, motion, trail):
        """One motion for the path follower. A fall is recovered (stand_up) up to 3 times per walk, so a stumble does not end the job."""
        r = self._motion(motion); trail.append(motion)
        if r["status"] == "failed" and "fell" in r["message"] and self._falls < 3:
            self._falls += 1; self._t_stand_up(); return {"status": "ok", "message": "recovered from a fall", "data": self._pose()}
        return r

    def _follow(self, waypoints, trail, max_motions=None):
        """Walk to each waypoint: turn toward it (to within 10 degrees), walk one cycle, re-read the pose. None = arrived.
        Every motion still goes through _motion, so the guard checks each one. The motion limit grows with the route (a drifting real
        NAO needs about 10 motions per metre) and stops a walk that goes in circles."""
        if max_motions is None:
            p0 = self._pose(); pts = [(p0["x"], p0["y"])] + list(waypoints)
            max_motions = int(max(120, 60 + 25 * sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))))
        i = 0
        while i < len(waypoints):
            wx, wy = waypoints[i]; last = i == len(waypoints) - 1
            self._dbg["next"] = (round(wx, 2), round(wy, 2))
            if len(trail) >= max_motions: return {"status": "failed", "message": f"more than {max_motions} motions"}
            p = self._pose(); dx, dy = wx - p["x"], wy - p["y"]; d = math.hypot(dx, dy)
            if d <= (0.3 if last else 0.25): i += 1; continue
            b = _wrap(math.degrees(math.atan2(dy, dx)) - p["yaw_deg"])
            turns = self.turn_motions(b) if abs(b) > (25 if d > 1.2 else 10) else ()   # a far waypoint tolerates the walk's drift: no 2-motion fix every step
            if turns:
                for m in turns:
                    r = self._run(m, trail)
                    if r["status"] != "ok": return r
                continue
            why = self.guard.check("forward", (p["x"], p["y"], p["yaw_deg"]), self.room, self.known_obstacles(), self.zones)
            if why and d <= REACH["forward"][0] + 0.2:
                i += 1; continue                                     # a full step would overshoot: this waypoint is as close as is safe
            r = self._run("forward", trail)
            if r["status"] != "ok": return r
        return None

    def _t_go_to(self, x, y):
        w, h = self.room
        if not (ROBOT_R <= x <= w - ROBOT_R and ROBOT_R <= y <= h - ROBOT_R):
            return {"status": "rejected", "message": f"({x}, {y}) is outside the house (house {w} x {h} m)"}
        return self._walk_path((float(x), float(y)), f"({x}, {y})")

    def _t_go_to_room(self, room): return self._walk_path(house.ROOMS[room][0][:2], f"the {room.replace('_', ' ')}")

    @staticmethod
    def footprint(item):
        """Box around an item and the furniture it stands on or touches (a television on its stand, a desk and its chair)."""
        x1, y1, x2, y2 = item["x"] - item["sx"] / 2, item["y"] - item["sy"] / 2, item["x"] + item["sx"] / 2, item["y"] + item["sy"] / 2
        for o in house.ITEMS:
            if not o["obstacle"] or o is item: continue
            if o["x"] - o["sx"] / 2 - 0.1 < x2 and o["x"] + o["sx"] / 2 + 0.1 > x1 and o["y"] - o["sy"] / 2 - 0.1 < y2 and o["y"] + o["sy"] / 2 + 0.1 > y1:
                x1, y1 = min(x1, o["x"] - o["sx"] / 2), min(y1, o["y"] - o["sy"] / 2)
                x2, y2 = max(x2, o["x"] + o["sx"] / 2), max(y2, o["y"] + o["sy"] / 2)
        return (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1

    def _object_spot(self, name, start=None):
        """The reachable free point beside the nearest item with that name (same room, in sight of it) with the shortest path from `start`
        (default: where the robot stands). -> (spot, item) or None."""
        p = self._pose(); start = start or (p["x"], p["y"]); best = None
        for it in sorted((i for i in house.ITEMS if i["label"] == name), key=lambda i: math.hypot(i["x"] - start[0], i["y"] - start[1])):
            # the item's own box first (closest approach); the merged footprint with the furniture it touches only as a fallback
            for cx, cy, sx, sy, pad in ((it["x"], it["y"], it["sx"], it["sy"], self.CLEAR + 0.1), (*self.footprint(it), self.CLEAR + 0.1)):
                ex, ey = sx / 2 + pad, sy / 2 + pad
                for k in range(28):                                  # points on a rectangle around the box
                    t = k / 7.0; side, f = int(t), t - int(t)
                    gx, gy = [(cx - ex + 2 * ex * f, cy - ey), (cx + ex, cy - ey + 2 * ey * f), (cx + ex - 2 * ex * f, cy + ey), (cx - ex, cy + ey - 2 * ey * f)][side]
                    if house.room_of(gx, gy) != house.room_of(it["x"], it["y"]) or house.blocks_view(gx, gy, it["x"], it["y"]): continue
                    path = self._plan((gx, gy), start)
                    if path is None: continue
                    length = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(path, path[1:]))
                    gap = math.hypot(max(abs(gx - it["x"]) - it["sx"] / 2, 0.0), max(abs(gy - it["y"]) - it["sy"] / 2, 0.0))
                    score = length + 4 * gap                         # prefer a spot close to the item itself, then a short walk
                    if best is None or score < best[0]: best = (score, (gx, gy), it)
                if best: break
            if best: break
        return (best[1], best[2]) if best else None

    def _t_go_to_object(self, name):
        """Walk next to the nearest item with that name (see _object_spot) and face it."""
        found = self._object_spot(name)
        if found is None: return {"status": "rejected", "message": f"no reachable free place next to a {name}"}
        return self._reach_object(*found, name)

    def _reach_object(self, spot, item, name):
        r = self._walk_path(spot, f"the {name}")
        if r["status"] == "ok":                                      # face the item at the end
            q = self._pose(); b = _wrap(math.degrees(math.atan2(item["y"] - q["y"], item["x"] - q["x"])) - q["yaw_deg"])
            for m in self.turn_motions(b):
                t = self._motion(m); r["trail"].append(m)
                if t["status"] != "ok": break
            r["data"] = self._pose()
        return r

    def _finish(self, call, res):
        res.setdefault("data", None)
        self.calls[res["status"]] = self.calls.get(res["status"], 0) + 1
        self._log({"event": "call", "call": call, **{k: v for k, v in res.items() if not k.startswith("_")}})
        self._remember(call, res)
        self._trace(call, res)
        return res

    # ---------------------------------------------------------------------------------------------------------- memory (advice only)
    def _mem(self, kind, **kw):
        """Write to the memory. A memory error never touches the robot: it is swallowed."""
        if self.memory is None: return
        try: self.memory.add(kind, sim=self.nao.name if self._backend is not None else self.sim, **kw)
        except Exception: pass

    def _remember(self, call, res):
        if self.memory is None: return
        tool = call.get("tool"); spec = HUMANOID_TOOLS.get(tool)
        if spec is None or tool == "recall_memory": return
        p = self._pose()
        if tool == "describe_view" and res.get("status") == "ok":
            for it in res.get("data") or []:
                self._mem("observation", label=it["label"], room=it.get("room"), x=it["x"], y=it["y"], note=f"seen from ({p['x']}, {p['y']})")
        elif spec.category in ("motion", "macro", "effector", "safety") and tool not in ("pause_cleaning", "resume_cleaning"):
            self._mem("action", tool=tool, args=call.get("args") or None, status=res["status"], room=p["room"], x=p["x"], y=p["y"], yaw=p["yaw_deg"],
                      note=None if res["status"] == "ok" else str(res.get("message", ""))[:160])
            if spec.category == "macro" and res["status"] == "ok": self._mem("position", room=p["room"], x=p["x"], y=p["y"], yaw=p["yaw_deg"], note=f"after {tool}")

    def recall_memory(self, limit=5) -> dict:
        """What the robot remembers that is relevant to where it is now (see orbit_ros/nao_memory.py). Advice only."""
        self._ensure()
        if self.memory is None: return {"enabled": False, "note": "memory is off (--no-memory)"}
        p = self._pose()
        rec = self.memory.recall(sim=self.nao.name, room=p["room"], x=p["x"], y=p["y"], limit=limit)
        return {"enabled": True, **rec, "brief": self.memory.brief(rec), "pose_now": p}

    def task_memory(self, instruction=None, goal=None, limit=3) -> dict:
        """Episodic + semantic recall for a NEW task (past task outcomes and verified strategies), as EVIDENCE for planning.
        Current sensors and the safety guard always override it. Empty when memory is off or retrieval is disabled."""
        self._ensure()
        if self.memory is None: return {"enabled": False, "note": "memory is off (--no-memory)"}
        p = self._pose()
        goal = goal or self._goal_key(instruction)
        tb = self.memory.task_brief(instruction=instruction, goal=goal, sim=self.nao.name, start_room=p["room"], limit=limit)
        return {"enabled": True, "pose_now": p, **tb}

    def _goal_key(self, instruction):
        """Normalize an instruction to a goal key using the house vocabulary (keeps nao_memory a leaf module)."""
        from .nao_memory import NaoMemory
        return NaoMemory.normalize_goal(instruction, rooms=list(house.ROOMS),
                                        objects={it["label"] for it in house.ITEMS})

    def record_episode(self, instruction, result, goal=None, experiment=None, scenario=None, start=None,
                       planned=None, feedback=None, provenance=None) -> dict:
        """Write one TASK episode after a verified outcome (episodic memory) and distil a semantic strategy. Honors write_after_task.
        `result` is the dict returned by robot_call for the task's top-level tool; success/outcome are taken from it + the true pose."""
        if self.memory is None: return {"enabled": False}
        if not getattr(self.memory, "write_after_task", True): return {"enabled": True, "written": False, "reason": "write_after_task disabled"}
        p = self._pose()
        status = (result or {}).get("status")
        success = status == "ok" and not p["fallen"]
        g = goal or self._goal_key(instruction)
        tool = (result or {}).get("_tool") or (self.last_task or {}).get("tool")
        args = (self.last_task or {}).get("args")
        executed = [{"tool": tool, "status": status}] if tool else None
        eid = self.memory.add_episode(instruction, goal=g, sim=self.nao.name, experiment=experiment, scenario=scenario,
                                      start=start or {"room": p["room"], "x": p["x"], "y": p["y"], "yaw": p["yaw_deg"]},
                                      planned=planned, executed=executed, outcome=str((result or {}).get("message", ""))[:200],
                                      success=success, final={"room": p["room"], "x": p["x"], "y": p["y"], "yaw": p["yaw_deg"]},
                                      replans=self.replans, failed_actions=self.calls.get("failed", 0), feedback=feedback, provenance=provenance)
        if tool:                                                     # distil a strategy only from a real top-level tool outcome
            self.memory.learn_strategy(g, tool=tool, args=args, success=bool(success), sim=self.nao.name,
                                       note=feedback, provenance=provenance)
        return {"enabled": True, "written": True, "episode_id": eid, "goal": g, "success": bool(success)}

    # ---------------------------------------------------------------------------------------------------------- session tools
    def set_no_go_zone(self, x, y, radius=0.5) -> dict:
        self._ensure()
        for v in (x, y, radius):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v): raise ValueError("x, y and radius must be numbers (metres)")
        if not 0.1 <= radius <= 3: raise ValueError("radius must be 0.1..3 m")
        self.zones.append({"x": float(x), "y": float(y), "r": float(radius)})
        self._log({"event": "set_no_go_zone", "zone": self.zones[-1]}); return {"no_go_zones": list(self.zones)}

    def status(self) -> dict:
        self._ensure()
        return {"robot": self.robot_name, "scene": self.scene, "simulator": self.nao.name, "house_m": list(self.room), "pose": self._pose(),
                "stopped": self.stopped, "bumped_obstacles": list(self.bumps), "no_go_zones": list(self.zones), "calls": dict(self.calls),
                "last_veto": self.last_veto, "cleaning": self.clean_status(), "audit_log": self.log_path}

    def house_info(self) -> dict:
        return {"house_m": list(self.room), "rooms": {k: {"area": list(v[1]), "entry_point": list(v[0][:2])} for k, v in house.ROOMS.items()},
                "items": [{"label": it["label"], "x": it["x"], "y": it["y"], "size": [it["sx"], it["sy"], it["sz"]], "room": house.room_of(it["x"], it["y"])}
                          for it in house.ITEMS]}

    def connection_status(self) -> dict:
        """Transport / simulator health, read-only (never moves the robot, never raises).

        Reports how the MCP server is connected to the robot: which backend is live, whether the live Webots extern
        controller is actually connected (a real sup.getSelf(), not just a listening socket), the single controller
        endpoint used (tcp://127.0.0.1:1235 for Webots), the robot's upright/stopped readiness and the remaining call
        budget. This is the diagnostic to call first when a command 'succeeds' but nothing seems to move.
        """
        from . import humanoid as _h
        info = {"robot": self.robot_name, "simulator": self.sim, "scene": self.scene,
                "transport": "in-process (direct)", "uses_port_1240": False,
                "call_budget": {"used": self.calls["total"], "max": self.max_calls,
                                "remaining": max(0, self.max_calls - self.calls["total"])}}
        if self.sim == "webots":
            info["controller_url"] = _h.WEBOTS_CONTROLLER_URL     # tcp://127.0.0.1:1235/orbit_nao
            info["controller_port"] = 1235
        backend = self._backend                                   # do NOT touch self.nao: that would force a connection attempt
        if backend is None:
            info.update(connected=False, backend=None,
                        detail="no backend yet; the first robot call (or load_scene) connects to the simulator")
            return info
        info["backend"] = backend.name
        try:
            if self.sim == "webots":
                # A real connection answers getSelf(); a merely-listening port would not.
                ok = getattr(backend, "sup", None) is not None and backend.sup.getSelf() is not None
                info["connected"] = bool(ok)
                if ok:
                    x, y, yaw, fallen = backend.pose()
                    info.update(pose={"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw, 1)},
                                fallen=bool(fallen), kinematic_fallbacks=getattr(backend, "fallbacks", 0))
            else:
                info["connected"] = True                          # the fake backend is always "connected"
        except Exception as e:                                    # a dead/paused Webots must not raise out of a status read
            info.update(connected=False, detail=f"{type(e).__name__}: {e}")
        info.update(stopped=self.stopped, ready=not self.stopped,
                    last_veto=self.last_veto)
        return info

    CAMERA_MODES = ("overview", "follow", "closeup", "target", "human", "demo", "reset")

    def camera(self, mode="overview", zoom=None, target=None) -> dict:
        """Adaptive display camera (Phase 16). DISPLAY ONLY: this never calls the LLM, never moves the robot, never changes the
        robot pose or physics, and never touches memory. It only writes the Webots Viewpoint node through the EXISTING Supervisor.

        mode: overview | follow | closeup | target | human | demo | reset.
        zoom: 'in' | 'out' | a positive number (dolly the camera closer/further along its view axis). Applied after the mode.
             zoom may be given on its own (mode omitted/'zoom') to only dolly the current framing.
        target: [x, y] required for mode 'target'; ignored otherwise.

        Returns {"status": ok|error, "mode", "zoom", "available", "simulator", "builtin_follow"?, "detail"?}. Never raises.
        Only the live Webots backend has a viewpoint; in --sim fake it returns status ok with available=False and does nothing.
        """
        from .viewpoint import Viewpoint
        info = {"status": "ok", "mode": None, "zoom": None, "simulator": self.sim, "available": False}
        # Build/cache the Viewpoint against the live backend. Do NOT force a connection in fake unless one already exists.
        backend = self._backend
        if backend is None and self.sim == "webots":
            try: backend = self.nao          # connecting is the normal path for a live camera call
            except Exception as e:
                info.update(status="error", detail=f"no backend: {type(e).__name__}: {e}"); return info
        if backend is None:
            info.update(detail="no backend yet (fake session, not connected); camera is a no-op"); return info
        vp = getattr(self, "_vp", None)
        if vp is None or getattr(vp, "nao", None) is not backend:
            try: vp = Viewpoint(backend); self._vp = vp
            except Exception as e:
                info.update(status="error", detail=f"viewpoint init failed: {type(e).__name__}: {e}"); return info
        info["available"] = bool(vp.available())
        if not vp.available():
            info.update(detail="no Webots Viewpoint node (not a live Webots world); camera is a no-op"); return info

        mode = (str(mode).strip().lower() if mode is not None else "overview")
        zoom_only = mode in ("", "zoom")
        try:
            if zoom_only:
                pass
            elif mode == "overview": r = vp.overview()
            elif mode == "reset": r = vp.reset()
            elif mode == "follow": r = vp.follow()
            elif mode == "demo": r = vp.demo()
            elif mode == "closeup": r = vp.closeup()
            elif mode == "human": r = vp.human()
            elif mode == "target":
                if not (isinstance(target, (list, tuple)) and len(target) >= 2):
                    info.update(status="error", detail="mode 'target' needs target=[x, y]"); return info
                r = vp.target([float(target[0]), float(target[1])])
            else:
                info.update(status="error", detail=f"unknown camera mode {mode!r}; choose from {list(self.CAMERA_MODES)} or 'zoom'")
                return info
            if not zoom_only:
                info["mode"] = r.get("mode", mode)
                if r.get("ok") is False:
                    info.update(status="error", detail=r.get("reason", "camera mode failed"), mode_result=r)
                else:
                    info["builtin_follow"] = r.get("builtin")
                    info["mode_result"] = r
            # optional dolly after the mode (or on its own)
            if zoom is not None:
                zr = vp.zoom(zoom)
                info["zoom"] = zoom
                info["zoom_result"] = zr
                if zr.get("ok") is False:
                    info.update(status="error", detail=zr.get("reason", "zoom failed"))
            if zoom_only and zoom is None:
                info.update(status="error", detail="give a mode or a zoom amount"); return info
            # render the change on the existing controller so a screenshot would capture it (no motion command = no robot movement)
            vp.render(frames=6)
        except Exception as e:                                   # a display problem must never disturb the robot
            info.update(status="error", detail=f"{type(e).__name__}: {e}"); return info
        self._log({"event": "camera", "mode": info.get("mode"), "zoom": info.get("zoom"), "available": info["available"],
                   "status": info["status"]})
        return info

    def search_github(self, query, limit=5, sort="best") -> dict:
        """Search public GitHub repositories over the internet (read-only, standard library urllib only).

        This does not move the robot and never goes through the safety guard: it only fetches text from GitHub's
        public REST search API (https://api.github.com/search/repositories) so the AI client can answer questions
        about code on the web. A network or API error is returned as a {"status": "error", ...} dict and never
        raised, so a failed search can never disturb the robot. Set a GITHUB_TOKEN in the environment to raise the
        unauthenticated rate limit; none is required for public search.
        """
        import urllib.parse, urllib.request, urllib.error
        q = str(query or "").strip()
        if not q: return {"status": "error", "query": query, "message": "query must be a non-empty string of GitHub search terms"}
        if isinstance(limit, bool) or not isinstance(limit, int): return {"status": "error", "query": q, "message": "limit must be an integer 1..20"}
        if not 1 <= limit <= 20: return {"status": "error", "query": q, "message": "limit must be an integer 1..20"}
        sort = str(sort or "best").lower()
        SORTS = {"best": ("", ""), "stars": ("stars", "desc"), "updated": ("updated", "desc"), "forks": ("forks", "desc")}
        if sort not in SORTS: return {"status": "error", "query": q, "message": f"sort must be one of {sorted(SORTS)}"}
        sort_key, order = SORTS[sort]
        params = {"q": q, "per_page": str(limit)}
        if sort_key: params["sort"] = sort_key; params["order"] = order
        url = "https://api.github.com/search/repositories?" + urllib.parse.urlencode(params)
        headers = {"Accept": "application/vnd.github+json", "User-Agent": "orbit-ros-nao",
                   "X-GitHub-Api-Version": "2022-11-28"}
        token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
        if token: headers["Authorization"] = f"Bearer {token}"
        self._log({"event": "search_github", "query": q, "limit": limit, "sort": sort})
        try:
            req = urllib.request.Request(url, headers=headers, method="GET")
            with urllib.request.urlopen(req, timeout=15) as resp:
                raw = resp.read()
            data = json.loads(raw.decode("utf-8"))
        except urllib.error.HTTPError as e:
            detail = ""
            try: detail = json.loads(e.read().decode("utf-8")).get("message", "")
            except Exception: pass
            return {"status": "error", "query": q, "message": f"GitHub returned HTTP {e.code}" + (f": {detail}" if detail else "")}
        except urllib.error.URLError as e:
            return {"status": "error", "query": q, "message": f"could not reach GitHub (no internet?): {e.reason}"}
        except Exception as e:                                        # malformed JSON, timeout, ...
            return {"status": "error", "query": q, "message": f"{type(e).__name__}: {e}"}
        if not isinstance(data, dict): return {"status": "error", "query": q, "message": "unexpected response from GitHub (not a JSON object)"}
        items = data.get("items") or []
        if not isinstance(items, list): items = []
        repos = [{"full_name": it.get("full_name"), "description": it.get("description"), "url": it.get("html_url"),
                  "stars": it.get("stargazers_count"), "forks": it.get("forks_count"), "language": it.get("language"),
                  "updated": it.get("updated_at")} for it in items[:limit]]
        return {"status": "ok", "query": q, "sort": sort, "total_count": data.get("total_count", len(repos)),
                "count": len(repos), "repositories": repos}

    def believed_map(self, cell=0.25) -> dict:
        self._ensure()
        w, h = self.room; nx, ny = int(round(w / cell)), int(round(h / cell)); p = self._pose()
        arrow = ">^<v"[int(((p["yaw_deg"] + 45) % 360) // 90)]
        obs = self.known_obstacles(); rows = []
        walls = set()                                                # cells a wall line runs through (walls are thinner than a cell)
        for x1, y1, x2, y2 in house.WALLS:
            n = int(max(abs(x2 - x1), abs(y2 - y1)) / (cell / 2)) + 1
            for k in range(n + 1):
                px, py = x1 + (x2 - x1) * k / n, y1 + (y2 - y1) * k / n
                walls.add((min(nx - 1, int(px / cell)), min(ny - 1, int(py / cell))))
        for j in range(ny - 1, -1, -1):
            line = ""
            for i in range(nx):
                cx, cy = (i + 0.5) * cell, (j + 0.5) * cell
                if (i, j) == (min(nx - 1, int(p["x"] / cell)), min(ny - 1, int(p["y"] / cell))): ch = arrow
                elif (i, j) in walls: ch = "|"
                elif any(not o.get("wall") and abs(cx - o["x"]) <= o["sx"] / 2 and abs(cy - o["y"]) <= o["sy"] / 2 for o in obs): ch = "#"
                elif any(math.hypot(cx - z["x"], cy - z["y"]) <= z["r"] for z in self.zones): ch = "X"
                else: ch = "."
                line += ch
            rows.append(line)
        return {"map": "\n".join(rows), "legend": f"one character = {cell} m; {arrow} robot heading, | wall, # furniture or bump, X no-go zone; top row = north (+y)"}

    # ---------------------------------------------------------------------------------------------------------- visual debugging
    def _obstacle_view(self):
        """Obstacles the robot works with now, split by where they come from: bumps, sonar echoes (they expire), no-go zones."""
        return {"obstacles": {"bumped": [{"x": b["x"], "y": b["y"]} for b in self.bumps],
                              "sonar": [{"x": o["x"], "y": o["y"], "expires_in_motions": max(0, o["expires"] - self.motions)} for o in self.sonar_obs],
                              "no_go_zones": [dict(z) for z in self.zones]}}

    def debug_state(self) -> dict:
        """Current target, planned route, robot position, obstacle locations and task status in one dictionary."""
        self._ensure()
        d = self._dbg; t = d["task"]
        return {"task": t, "target": d["target"], "route": list(d["route"]), "next_waypoint": d["next"], "plan": [dict(s) for s in d["plan"]], "pose": self._pose(),
                **self._obstacle_view(), "replans": self.replans, "motions": self.motions, "collisions": self.collisions, "last_veto": self.last_veto}

    def debug_view(self, cell=0.25) -> dict:
        """The house map with the route (*), the target (T), unmapped obstacles found by bump or sonar (?) and the robot (arrow) drawn in,
        next to the state of debug_state. Reads only the session state: it does not move the robot."""
        st = self.debug_state(); grid = [list(row) for row in self.believed_map(cell)["map"].splitlines()]
        ny, nx = len(grid), len(grid[0])
        def put(x, y, ch, over=(".", "*", "#", "X")):
            i, j = min(nx - 1, max(0, int(x / cell))), min(ny - 1, max(0, int(y / cell)))
            if grid[ny - 1 - j][i] in over: grid[ny - 1 - j][i] = ch
        for (x0, y0), (x1, y1) in zip(st["route"], st["route"][1:]):
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / (cell / 2)))
            for k in range(n + 1): put(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, "*", over=(".",))
        for o in st["obstacles"]["bumped"] + st["obstacles"]["sonar"]: put(o["x"], o["y"], "?", over=(".", "#", "*"))
        if st["target"]: put(st["target"]["x"], st["target"]["y"], "T", over=(".", "*", "#", "X", "?"))
        p, t = st["pose"], st["task"]
        lines = [f"task: {t['tool']} {json.dumps(t['args'])} -> {t['status']}: {t['message']}" if t else "task: none yet",
                 f"target: {st['target']['label']} at ({st['target']['x']}, {st['target']['y']})" if st["target"] else "target: none",
                 f"robot: ({p['x']}, {p['y']}) facing {p['yaw_deg']} deg in the {p['room']}" + (" FALLEN" if p["fallen"] else ""),
                 f"route: {len(st['route'])} waypoints, next {st['next_waypoint']}", f"replans {st['replans']}, bumps/sonar {len(st['obstacles']['bumped'])}/{len(st['obstacles']['sonar'])}"]
        if st["plan"]:
            cur = next((s for s in st["plan"] if s["state"] == "running"), None)
            lines.insert(1, f"plan: {sum(s['state'] == 'done' for s in st['plan'])}/{len(st['plan'])} done" + (f", now: {cur['do']}" if cur else ""))
        return {**st, "map": "\n".join("".join(r) for r in grid), "summary": lines,
                "legend": f"one character = {cell} m; arrow robot, T target, * planned route, ? unmapped obstacle (bump or sonar), # furniture, | wall, X no-go zone; top row = north (+y)"}

    def _trace(self, call, res):
        """After every motion, macro, effector or safety call: one compact record in the audit log and the status label in the Webots view.
        Diagnostic only: a lost connection (pose read fails) must never turn an already-formed result into an exception."""
        spec = HUMANOID_TOOLS.get(call.get("tool"))
        if spec is None or spec.category not in ("motion", "macro", "effector", "safety"): return
        try:
            st = self.debug_state()
            self._log({"event": "debug", "tool": call["tool"], "status": res["status"], "target": st["target"], "route": st["route"], "next_waypoint": st["next_waypoint"],
                       "pose": st["pose"], **self._obstacle_view(), "replans": st["replans"]})
        except RuntimeError as e:                                     # the simulator/connection is gone: record it, do not raise
            self._log({"event": "debug", "tool": call["tool"], "status": res["status"], "pose": None, "note": f"debug state unavailable: {e}"})
        self._show()

    def _show(self):
        show = getattr(self.nao, "show", None) if self._backend is not None else None
        if show:
            try: show(self.debug_view()["summary"])
            except Exception: pass                                    # a display problem never touches the robot

    def capabilities(self) -> dict:
        return {"robot": self.robot_name, "simulator": self.sim, "tools": {n: {"args": {k: {"kind": a.kind, "min": a.min, "max": a.max, "values": list(a.values)}
                                                                                    for k, a in s.args.items()}, "preconditions": list(s.pre)}
                                                                       for n, s in HUMANOID_TOOLS.items()},
                "rooms": sorted(house.ROOMS), "safety": "every motion passes the argument check and the humanoid safety guard"}

    def _log(self, rec):
        if not self.log_path: return
        try:
            with open(self.log_path, "a", encoding="utf-8") as f: f.write(json.dumps({"t": round(time.time(), 3), **rec}, default=str) + "\n")
        except OSError: pass
