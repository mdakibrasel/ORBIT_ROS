"""v0.8 extension: runtime execution monitor.

Compares what the robot *believes* each hardware command will do with what the world reports it did.  It never changes the
world and never calls the LLM; it only classifies each executed hardware sub-command and, when something is off, "trips" so the
runner can stop executing a plan that was written under a belief that has just been contradicted.

  ok          observed pose == pose predicted under the translation belief
  deviation   command succeeded but the pose differs (wrong gain / polarity / drift)   -> trip
  blocked     the world refused the move (obstacle or wall)                             -> trip
  invalid     the world rejected the call                                               -> trip

`suspended=True` (set by the runner during plan-as-probe) records nothing: a probe is *meant* to be surprised.
Everything here is a simulator-level mechanism; no claim is made about real sensors, whose pose estimates are noisy.
"""
from __future__ import annotations
from dataclasses import dataclass
from .embodiment_runtime import predict_pose

@dataclass
class MonitorEvent:
    kind: str
    tool: str
    expected: tuple | None
    observed: tuple
    detail: str = ""

class ExecutionMonitor:
    def __init__(self, enabled: bool = True):
        self.enabled, self.suspended = enabled, False
        self.events: list[MonitorEvent] = []
        self.tripped: str | None = None
        self.n_deviation = self.n_blocked = self.n_invalid = 0

    def reset(self) -> None:
        """Clear the trip flag (called before each plan execution); the event history is kept."""
        self.tripped = None

    def after_hw(self, pre: dict, hw_tool: str, hw_args: dict, post: dict, result: dict, belief) -> MonitorEvent | None:
        if not self.enabled or self.suspended or hw_tool not in ("turn", "move_forward"): return None
        obs = (post["x"], post["y"], post["yaw_deg"])
        if not result.get("ok"):
            msg = str(result.get("message", ""))
            kind = "blocked" if msg.startswith("blocked") else "invalid"
            ev = MonitorEvent(kind, hw_tool, None, obs, msg)
        else:
            exp = predict_pose(pre, hw_tool, hw_args, belief)
            ev = MonitorEvent("ok" if exp == obs else "deviation", hw_tool, exp, obs, "" if exp == obs else f"expected {exp}, observed {obs}")
        self.events.append(ev)
        if ev.kind != "ok":
            self.tripped = ev.kind; setattr(self, f"n_{ev.kind}", getattr(self, f"n_{ev.kind}") + 1)
        return ev
