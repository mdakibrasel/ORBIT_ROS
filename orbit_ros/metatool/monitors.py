"""In-step runtime monitor evaluator for the meta-tool (Phase 17, feature 6). Pure code: no robot, no LLM.

Defines the deterministic checks a motion loop should run at >= 5 Hz of simulation time: progress toward the active
waypoint, heading drift, contract invariants, person proximity (only where reliable observations exist), and fall
state. Each `evaluate` call takes a lightweight snapshot and returns a MonitorVerdict (continue / stop + reason +
observed values + thresholds) that the caller records in the trace. On a stop verdict the caller must bring the robot
to a safe state before any repair.

This module is the EVALUATOR only. Wiring it into the live Webots motion loop touches the protected motion/safety
path and can only be validated live; that integration is deferred and documented rather than changed blindly here.
Nothing in this module moves the robot or overrides the HumanoidGuard."""
from __future__ import annotations
import math
from dataclasses import dataclass, field

MIN_HZ = 5.0
NO_PROGRESS_M = 0.02          # minimum progress toward the waypoint between checks to count as "moving"
NO_PROGRESS_WINDOW_S = 3.0    # if no progress for this long -> stop
MAX_HEADING_DRIFT_DEG = 45.0  # drift between desired heading and actual heading that is unsafe
PERSON_CRITICAL_M = 0.6       # only enforced when person distance is actually observed


@dataclass
class MonitorVerdict:
    action: str                 # continue | stop
    reason: str = ""
    observed: dict = field(default_factory=dict)
    thresholds: dict = field(default_factory=dict)

    def as_dict(self):
        return {"action": self.action, "reason": self.reason, "observed": self.observed, "thresholds": self.thresholds}


@dataclass
class ProgressState:
    """Caller-held state across checks for one motion segment."""
    last_dist: float | None = None
    last_improve_t: float = 0.0
    started: bool = False


def evaluate(snapshot, progress: ProgressState, now):
    """Return a MonitorVerdict for one tick.

    snapshot keys (all optional; missing -> that check is skipped):
      fallen: bool
      dist_to_waypoint: float (m)
      desired_heading_deg, actual_heading_deg: float
      person_distance_m: float   (ONLY when a person was actually observed; None/absent otherwise)
      contract_violation: str     (set by the caller's contract check; non-empty -> stop)
    """
    obs, thr = {}, {}

    # 1) fall state
    if snapshot.get("fallen"):
        return MonitorVerdict("stop", "fall detected", {"fallen": True}, {})

    # 2) contract invariant violation (caller supplies the verified result)
    cv = snapshot.get("contract_violation")
    if cv:
        return MonitorVerdict("stop", f"contract invariant violated: {cv}", {"contract_violation": cv}, {})

    # 3) person proximity (ONLY if a real observation is present; never assume clear)
    pd = snapshot.get("person_distance_m")
    if pd is not None:
        obs["person_distance_m"] = pd; thr["person_critical_m"] = PERSON_CRITICAL_M
        if pd < PERSON_CRITICAL_M:
            return MonitorVerdict("stop", f"person within {PERSON_CRITICAL_M} m", obs, thr)

    # 4) heading drift
    dh = snapshot.get("desired_heading_deg"); ah = snapshot.get("actual_heading_deg")
    if dh is not None and ah is not None:
        drift = abs(_wrap180(dh - ah))
        obs["heading_drift_deg"] = round(drift, 1); thr["max_heading_drift_deg"] = MAX_HEADING_DRIFT_DEG
        if drift > MAX_HEADING_DRIFT_DEG:
            return MonitorVerdict("stop", f"heading drift {round(drift,1)} deg exceeds {MAX_HEADING_DRIFT_DEG}", obs, thr)

    # 5) progress toward waypoint
    d = snapshot.get("dist_to_waypoint")
    if d is not None:
        obs["dist_to_waypoint"] = round(d, 3); thr["no_progress_window_s"] = NO_PROGRESS_WINDOW_S
        if not progress.started:
            progress.started = True; progress.last_dist = d; progress.last_improve_t = now
        else:
            if progress.last_dist is None or (progress.last_dist - d) >= NO_PROGRESS_M:
                progress.last_dist = d; progress.last_improve_t = now
            elif (now - progress.last_improve_t) >= NO_PROGRESS_WINDOW_S:
                return MonitorVerdict("stop", f"no progress for {NO_PROGRESS_WINDOW_S}s "
                                      f"(dist stuck at {round(d,3)} m)", obs, thr)

    return MonitorVerdict("continue", "", obs, thr)


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def min_interval_s(hz=MIN_HZ):
    """The maximum seconds between checks to sustain at least `hz` of simulation time."""
    return 1.0 / float(hz)
