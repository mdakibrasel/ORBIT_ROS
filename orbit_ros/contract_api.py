"""Simulator-independent robot contract (v0.13): one typed interface that every simulator adapter implements.

    observe()       -> Observation   pose (as the robot reports it), camera view, a step counter, a timestamp, validity and staleness flags
    act(tool, args) -> ActionResult  typed status: ok | blocked | safety_stop | fault | invalid | stopped | timeout
    stop()                           stops all motion; later motion is refused with status "stopped"
    reset(spec)                      builds the scenario described by a ScenarioSpec (same spec -> same initial observation)
    state()         -> dict          GROUND TRUTH for the evaluator only (true pose, objects with ids and cells); never shown to an agent
    capabilities()  -> dict          supported actions and sensors, motion model, ground-truth availability

The agent loop (`agent2d.run_episode`) still talks to the World2D-compatible object directly, so no existing result changes. This layer is
what the conformance suite (`orbit_ros/conformance.py`, `python conformance.py --sim ...`) checks for every adapter: World2D, Webots and
FakeSim through `WebotsWorld2D`, ROS 2 through `Ros2Sim`. A simulator counts as supported only after it passes that suite.
"""
from __future__ import annotations
import time
from dataclasses import dataclass, field, asdict
from .world2d import World2D, Obj

STATUSES = ("ok", "blocked", "safety_stop", "fault", "invalid", "stopped", "timeout")
REQUIRED_ACTIONS = ("move_forward", "turn", "stop_robot")

@dataclass
class ScenarioSpec:
    width: int
    height: int
    start: tuple
    yaw: int
    objects: list = field(default_factory=list)     # [(id, label, x, y)]
    mirrored_camera: bool = False

    def world(self) -> World2D:
        return World2D(self.width, self.height, tuple(self.start), self.yaw, [Obj(*o) for o in self.objects], mirrored_camera=self.mirrored_camera)

@dataclass
class Observation:
    step: int                 # robot calls so far (monotonic)
    stamp_s: float            # adapter clock when the observation was taken
    pose: dict                # x, y, yaw_deg as the robot reports it (odometry), room size
    view: list                # camera detections: label, image_x, apparent_size
    valid: bool = True
    stale: bool = False       # the camera returned an old frame

@dataclass
class ActionResult:
    status: str
    message: str
    pose: dict | None
    duration_s: float

def status_of(result: dict) -> str:
    if result.get("ok"): return "ok"
    m = str(result.get("message", ""))
    if m.startswith("blocked"): return "blocked"
    if m.startswith("safety stop"): return "safety_stop"
    if m.startswith("motor fault"): return "fault"
    if m.startswith("robot is stopped"): return "stopped"
    return "invalid"

class RobotAdapter:
    """Contract over any World2D-compatible world (World2D, WebotsWorld2D on FakeSim / Webots / ROS 2)."""
    def __init__(self, make_world, motion="grid", name="world2d", clock=time.monotonic):
        self.make_world, self.motion, self.name, self.clock, self.world = make_world, motion, name, clock, None
    def capabilities(self) -> dict:
        return {"actions": ["move_forward", "turn", "clean", "stop_robot", "report", "localize"], "sensors": ["pose", "camera"],
                "motion": self.motion, "ground_truth": True, "action_timeout": True, "name": self.name}
    def reset(self, spec: ScenarioSpec):
        self.world = self.make_world(spec); return self.observe()
    def observe(self) -> Observation:
        w = self.world
        pose = w.execute("get_pose", {})["data"]; view = w.execute("describe_view", {})["data"]
        return Observation(w.tool_calls, self.clock(), pose, view, valid=pose is not None and view is not None, stale=bool(getattr(w, "last_view_stale", False)))
    def act(self, tool, args=None, timeout_s=None) -> ActionResult:
        if tool not in ("move_forward", "turn", "clean", "stop_robot", "report", "localize"):
            return ActionResult("invalid", f"unknown action {tool}", None, 0.0)
        t0 = self.clock(); r = self.world.execute(tool, dict(args or {})); dt = self.clock() - t0
        st = status_of(r)
        if timeout_s is not None and dt > timeout_s and st == "ok": st = "timeout"
        return ActionResult(st, str(r.get("message", "")), r.get("data") if isinstance(r.get("data"), dict) else None, dt)
    def stop(self) -> ActionResult: return self.act("stop_robot")
    def state(self) -> dict:
        w = self.world
        return {"x": w.x, "y": w.y, "yaw_deg": w.yaw, "objects": [{"id": o.id, "label": o.label, "x": o.x, "y": o.y} for o in w.objects],
                "collisions": w.collisions}

def world2d_adapter():
    return RobotAdapter(lambda spec: spec.world(), "grid", "world2d")

def fakesim_adapter(motion="grid"):
    from .webots_adapter import WebotsWorld2D, FakeSim, calibrate_lidar
    sim = FakeSim()
    if motion == "continuous": calibrate_lidar(sim)
    return RobotAdapter(lambda spec: WebotsWorld2D.adopt(spec.world(), sim, "camera", motion), motion, f"fakesim-{motion}")

def sim_adapter(sim, motion="grid", name="sim"):
    """Webots (`WebotsSim()`) or ROS 2 (`Ros2Sim(...)`) behind the same contract."""
    from .webots_adapter import WebotsWorld2D, calibrate_lidar
    if motion == "continuous": calibrate_lidar(sim)
    return RobotAdapter(lambda spec: WebotsWorld2D.adopt(spec.world(), sim, "camera", motion), motion, name)
