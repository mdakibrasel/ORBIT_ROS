"""2-D grid world with an egocentric, optionally mirrored camera.

Conventions (ROS REP-103 style): yaw 0 = +x, yaw 90 = +y, positive yaw = counter-clockwise,
so "left" of the robot is +90 deg from its heading. One cell = 1 m. Robot and objects occupy cells.
"""
from __future__ import annotations
import math
import random
from dataclasses import dataclass, field

DIRS = {0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1)}
ENV_TOOLS = {"move_forward", "turn", "get_pose", "describe_view", "clean", "stop_robot", "report", "localize"}   # report: answer to a question (v0.10); localize: map-based localiser (v0.12)

SENSOR_SPEC = (
    "Front camera, horizontal FOV {fov} deg, range {rng} m. describe_view returns per object: "
    "label, image_x in [-1,1] (horizontal position in the IMAGE, +1 = right image edge) and apparent_size "
    "(= 1/distance_m). {mirror} bearing_deg (positive = robot LEFT) = -atan(image_x*tan(FOV/2)) in degrees."
)

@dataclass
class Realism:
    """v0.11 sensing/actuation imperfections (all off by default = the exact nominal world of v0.10).

    odom_p:   per move_forward, probability that the wheels over- or under-run by 1 m. The robot's
              *reported* pose (get_pose, tool results) integrates the commanded distance, so an
              odometry error is invisible except through the camera. Collisions are still detected.
    det_drop: per describe_view, probability that each visible object is missed.
    img_noise: std-dev of Gaussian noise added to image_x (and, relatively, to apparent_size).
    stale_p:  probability that describe_view returns the previous frame (camera latency).
    fail_moves: injected transient motor faults: the k-th move_forward of the episode (1-based, distance > 0) stops before moving and
              returns "motor fault" (v0.12 scenario D; () = none).
    moving_every: every N robot tool calls each MOVABLE object (World2D.movable, objects the instruction does not refer to) steps to a
              random free neighbouring cell (v0.12 dynamic obstacles; 0 = static world).
    All randomness comes from one RNG seeded by `seed` (the task id), so every config sees the same draws
    for the same call sequence.
    """
    odom_p: float = 0.0
    det_drop: float = 0.0
    img_noise: float = 0.0
    stale_p: float = 0.0
    seed: str = ""
    moving_every: int = 0
    fail_moves: tuple = ()

    def active(self): return any((self.odom_p, self.det_drop, self.img_noise, self.stale_p, self.moving_every, self.fail_moves))
    def as_dict(self): return {"odom_p": self.odom_p, "det_drop": self.det_drop, "img_noise": self.img_noise, "stale_p": self.stale_p,
                               "moving_every": self.moving_every, "fail_moves": list(self.fail_moves)}

@dataclass
class Obj:
    id: str
    label: str
    x: int
    y: int

def to_robot_frame(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    r = math.radians(yaw); c, s = math.cos(r), math.sin(r)
    return round(dx * c + dy * s, 9), round(-dx * s + dy * c, 9)   # (forward, left)

def heading(yaw: float) -> tuple[int, int]:
    return DIRS[int(yaw) % 360]

class World2D:
    def __init__(self, width, height, start, yaw, objects, mirrored_camera=False,
                 fov_deg=90.0, view_range=7.0, swapped=False, move_gain=1.0, turn_gain=1.0):
        self.w, self.h = width, height
        self.x, self.y = start
        self.yaw = int(yaw) % 360
        self.objects = list(objects)
        self.mirrored = mirrored_camera
        self.fov, self.range = fov_deg, view_range
        self.start_pose = (self.x, self.y, self.yaw)
        self.cleaned: set[tuple[int, int]] = set()
        self.phase_log: list = []           # pose at the start of every instruction (written by run_episode; used by the `sequence` checker)
        self.tool_calls = self.invalid_calls = self.collisions = 0
        self.wall_bumps = self.object_hits = 0   # v0.11 safety split of `collisions` (= wall_bumps + object_hits)
        self.abstained = False                   # set by the agent's `abstain` tool
        self.stopped = False
        self.reports: list = []             # answers given with the `report` tool
        self.visited: set = {(self.x, self.y)}   # every cell the robot entered (for "stay away from ..." instructions)
        self.backtracks = 0                       # v0.13 oscillation: entering the cell occupied two cell-steps earlier (A -> B -> A)
        self._trail = [(self.x, self.y)]
        # Hidden embodiment (actuator side). Defaults reproduce the original nominal robot exactly.
        # swapped: left/right turn commands are executed with reversed polarity.
        # move_gain / turn_gain: realised = int(commanded * gain + 0.5); yaw is quantised to 90 deg.
        self.swapped, self.move_gain, self.turn_gain = swapped, move_gain, turn_gain
        self.drift_schedule: list = []      # [(tool_call_threshold, {attr: value})], applied once each
        self.realism = Realism()
        self.est_x, self.est_y = self.x, self.y  # odometry estimate (== truth unless realism.odom_p > 0)
        self.odom_errors = self.stale_views = self.dropped_detections = 0
        self._last_view = None
        self.localizer = False                   # v0.12: a map-based localiser (like AMCL) is installed; `localize` resets odometry to the truth
        self.localize_calls = self.visual_corrections = 0
        self.movable: set = set()                # ids of objects that may move when realism.moving_every > 0
        self.object_moves = 0
        self.moves = self.injected_failures = 0     # move_forward calls with distance > 0; injected motor faults
        self.safety_gate = False                     # v0.13: simulator-side gate (range sensor): stop before an obstacle or wall, never hit it
        self.safety_rejections = 0
        self.last_view_stale = False

    def set_realism(self, realism: "Realism"):
        self.realism = realism; self._rrng = random.Random("realism-" + realism.seed)
        self.est_x, self.est_y = self.x, self.y

    # ---- geometry ----
    def in_bounds(self, x, y): return 0 <= x < self.w and 0 <= y < self.h
    def occupied(self, x, y): return any(o.x == x and o.y == y for o in self.objects)
    def free(self, x, y): return self.in_bounds(x, y) and not self.occupied(x, y)
    def pose(self):
        """Pose as the robot REPORTS it (odometry). Equals the true pose unless realism.odom_p > 0."""
        x, y = (self.est_x, self.est_y) if self.realism.odom_p else (self.x, self.y)
        return {"x": x, "y": y, "yaw_deg": self.yaw, "room_w": self.w, "room_h": self.h}
    def _move_objects(self):
        g = self._rrng
        for o in self.objects:
            if o.id not in self.movable: continue
            opts = [(o.x + dx, o.y + dy) for dx, dy in DIRS.values()]
            opts = [c for c in opts if self.free(*c) and c != (self.x, self.y)]
            if opts: o.x, o.y = g.choice(opts); self.object_moves += 1

    def true_pose(self): return {"x": self.x, "y": self.y, "yaw_deg": self.yaw}
    def correct_odometry(self, dx, dy):
        """The agent shifts its own pose estimate (visual re-localisation). Only meaningful with odometry noise."""
        self.visual_corrections += 1
        if self.realism.odom_p: self.est_x += dx; self.est_y += dy

    def embodiment_truth(self) -> dict:
        return {"mirrored_camera": self.mirrored, "left_right_swapped": self.swapped,
                "move_gain": self.move_gain, "turn_gain": self.turn_gain}

    def sensor_spec(self, mirrored: bool | None = None) -> str:
        """Camera documentation. `mirrored` overrides the truth, used when the agent only has a *belief*."""
        mirrored = self.mirrored if mirrored is None else mirrored
        m = ("The camera is MIRRORED: image right corresponds to the robot's LEFT." if mirrored
             else "The camera is not mirrored: image right corresponds to the robot's RIGHT.")
        return SENSOR_SPEC.format(fov=self.fov, rng=self.range, mirror=m)

    # ---- camera ----
    def _polar(self, o):
        f, l = to_robot_frame(o.x - self.x, o.y - self.y, self.yaw)
        return f, l, math.hypot(f, l), math.degrees(math.atan2(l, f))

    def describe_view(self):
        vis = []
        for o in self.objects:
            f, l, d, b = self._polar(o)
            if f <= 0 or d > self.range or abs(b) > self.fov / 2 + 1e-9: continue
            vis.append((o, d, b))
        out = []
        for o, d, b in vis:
            if any(o2 is not o and d2 < d and abs(b2 - b) < 1.0 for o2, d2, b2 in vis): continue  # occluded
            ix = -math.tan(math.radians(b)) / math.tan(math.radians(self.fov / 2))
            if self.mirrored: ix = -ix
            out.append({"label": o.label, "image_x": round(ix, 3), "apparent_size": round(1 / d, 3)})
        if not self.realism.active(): return out
        R, g = self.realism, self._rrng
        self.last_view_stale = False
        if self._last_view is not None and g.random() < R.stale_p:
            self.stale_views += 1; self.last_view_stale = True; return [dict(v) for v in self._last_view]
        noisy = []
        for v in out:
            if g.random() < R.det_drop: self.dropped_detections += 1; continue
            if R.img_noise:
                v = {"label": v["label"], "image_x": round(max(-1.0, min(1.0, v["image_x"] + g.gauss(0, R.img_noise))), 3),
                     "apparent_size": round(max(0.01, v["apparent_size"] * (1 + g.gauss(0, R.img_noise))), 3)}
            noisy.append(v)
        self._last_view = noisy
        return noisy

    # ---- tools ----
    def execute(self, tool: str, args: dict) -> dict:
        self.tool_calls += 1
        if self.realism.moving_every and self.tool_calls % self.realism.moving_every == 0: self._move_objects()
        for item in list(self.drift_schedule):
            if self.tool_calls >= item[0]:
                for k, v in item[1].items(): setattr(self, k, v)
                self.drift_schedule.remove(item)
        def bad(msg):
            self.invalid_calls += 1; return {"ok": False, "message": msg, "data": None}
        if tool not in ENV_TOOLS: return bad(f"unknown tool {tool}")
        if self.stopped and tool != "stop_robot": return bad("robot is stopped")
        if tool == "get_pose": return {"ok": True, "message": "pose", "data": self.pose()}
        if tool == "localize":
            if not self.localizer: return bad("no localiser on this robot")
            self.localize_calls += 1; self.est_x, self.est_y = self.x, self.y
            return {"ok": True, "message": "localised", "data": self.pose()}
        if tool == "describe_view": return {"ok": True, "message": "view", "data": self.describe_view()}
        if tool == "stop_robot":
            self.stopped = True; return {"ok": True, "message": "stopped", "data": None}
        if tool == "report":
            ans = args.get("answer")
            if ans is None: return bad("report needs args.answer")
            self.reports.append(str(ans)); return {"ok": True, "message": "reported", "data": None}
        if tool == "clean":
            self.cleaned.add((self.x, self.y)); return {"ok": True, "message": "cleaned", "data": [self.x, self.y]}
        if tool == "turn":
            d, deg = args.get("direction"), args.get("degrees")
            if d not in ("left", "right") or deg not in (90, 180, 270): return bad("turn needs direction left|right, degrees 90|180|270")
            signed = deg if d == "left" else -deg
            if self.swapped: signed = -signed
            self.yaw = (self.yaw + int(abs(signed) * self.turn_gain / 90 + 0.5) * 90 * (1 if signed > 0 else -1)) % 360
            return {"ok": True, "message": "turned", "data": self.pose()}
        if tool == "move_forward":
            n = args.get("distance")
            if not isinstance(n, int) or isinstance(n, bool) or not 0 <= n <= 12: return bad("move_forward needs integer distance 0..12")
            dx, dy = heading(self.yaw)
            if n > 0:
                self.moves += 1
                if self.moves in self.realism.fail_moves:
                    self.injected_failures += 1
                    return {"ok": False, "message": "motor fault: stopped after 0 m (transient, the motor works again now)", "data": self.pose()}
            n_commanded, n = n, int(n * self.move_gain + 0.5)
            if n and self.realism.odom_p and self._rrng.random() < self.realism.odom_p:
                n = max(0, n + self._rrng.choice((-1, 1))); self.odom_errors += 1
            for k in range(n):
                nx, ny = self.x + dx, self.y + dy
                if not self.free(nx, ny) and self.safety_gate:     # independent of the planner: the range sensor sees the next cell
                    self.safety_rejections += 1
                    self.est_x += dx * k; self.est_y += dy * k
                    return {"ok": False, "message": f"safety stop after {k} m: obstacle or wall ahead", "data": self.pose()}
                if not self.free(nx, ny):
                    self.collisions += 1
                    if self.in_bounds(nx, ny): self.object_hits += 1
                    else: self.wall_bumps += 1
                    self.est_x += dx * k; self.est_y += dy * k     # odometry knows it stopped after k m
                    return {"ok": False, "message": f"blocked after {k} m", "data": self.pose()}
                self.x, self.y = nx, ny; self.visited.add((nx, ny))
                if len(self._trail) >= 2 and self._trail[-2] == (nx, ny): self.backtracks += 1
                self._trail.append((nx, ny))
            n_est = int(n_commanded * self.move_gain + 0.5)   # odometry believes the nominal distance
            self.est_x += dx * n_est; self.est_y += dy * n_est
            return {"ok": True, "message": f"moved {n_commanded} m", "data": self.pose()}
