"""ORBIT-PEL safety guard (v0.14 milestone M4): hard constraints checked before a call reaches the robot, independent of the model.

The guard predicts what a motion call would do from the robot's REPORTED pose and the agent's BELIEVED map (cells it saw occupied or
bumped into, cells the plan declared no-go) and vetoes the call if the prediction leaves the room, enters a believed obstacle or a
no-go cell. It never reads the true pose or the evaluator's task data, so it can be wrong exactly when the robot's belief is wrong;
that is what a real robot's software guard can do as well.

Motion model: a move of n cells is predicted as exactly n cells along the reported heading. The simulator can apply a motor gain
(`move_gain`) and odometry error; over-travel caused by a wrong gain or by odometry drift is NOT predicted by the guard (review L3).
The simulator-side stop below is the layer that sees true occupancy.

What is simulated and what is not: every check here is software in a simulation. Nothing is enforced by hardware. The simulator-side
stop (`--safety-gate`, World2D) is a second, independent layer that uses the simulator's true occupancy (it plays the role of a
range sensor), and it is a separate switch.
"""
from __future__ import annotations
from ..world2d import heading
from ..navtools import MACROS, expand

def _walk(pose, n):
    dx, dy = heading(pose["yaw_deg"]); return [(pose["x"] + dx * k, pose["y"] + dy * k) for k in range(1, n + 1)]

class SafetyGuard:
    def __init__(self, max_steps_per_call: int = 12):
        self.max_steps_per_call = max_steps_per_call

    def check(self, call: dict, pose: dict | None, obstacles_known, no_go=(), start_pose=None, cleaned=frozenset()) -> str | None:
        """None = allowed; otherwise the reason for the veto."""
        if pose is None or any(k not in pose for k in ("x", "y", "yaw_deg", "room_w", "room_h")): return "no valid pose: motion is not allowed"
        tool, args = call.get("tool"), call.get("args") or {}
        blocked, nogo = {tuple(c) for c in obstacles_known}, {tuple(c) for c in no_go}
        w, h = pose["room_w"], pose["room_h"]
        if tool == "move_forward":
            n = args.get("distance")
            # v0.14.1 (review L2): fail closed. Anything but an integer 0..limit is vetoed here, so a third-party adapter that would
            # accept a negative or fractional distance never receives one, even with the registry switched off.
            if not isinstance(n, int) or isinstance(n, bool): return f"malformed distance {n!r}: must be an integer 0..{self.max_steps_per_call}"
            if n < 0: return f"negative distance {n}: must be an integer 0..{self.max_steps_per_call}"
            if n > self.max_steps_per_call: return f"distance {n} exceeds the per-call limit {self.max_steps_per_call}"
            for c in _walk(pose, n):
                if not (0 <= c[0] < w and 0 <= c[1] < h): return f"would leave the room at cell {list(c)}"
                if c in blocked: return f"would enter believed obstacle cell {list(c)}"
                if c in nogo: return f"would enter declared no-go cell {list(c)}"
            return None
        if tool in MACROS:
            prims, err = expand(tool, args, pose, start_pose or (pose["x"], pose["y"], pose["yaw_deg"]), sorted(blocked | nogo), cleaned)
            if err: return f"{tool}: {err}"
            p = dict(pose)
            for a in prims:
                if a["tool"] == "turn":
                    d = a["args"]["degrees"] * (1 if a["args"]["direction"] == "left" else -1); p["yaw_deg"] = (p["yaw_deg"] + d) % 360
                elif a["tool"] == "move_forward":
                    r = self.check(a, p, blocked, nogo)
                    if r: return f"{tool} expansion: {r}"
                    p["x"], p["y"] = _walk(p, a["args"]["distance"])[-1] if a["args"]["distance"] else (p["x"], p["y"])
            return None
        return None
