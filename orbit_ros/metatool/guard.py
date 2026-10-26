"""Loop + budget guard for the meta-tool executor (section 4.7). Pure logic; it decides whether to BLOCK a call, it never moves the robot.

Fingerprint = (tool, normalised args, state digest). State digest = (pose rounded to 0.1 m and 15 deg, fallen, stopped, memory size).
Block when the same fingerprint ran 2x with no state change, or when an A-B-A-B-A pattern appears. Budgets are configurable."""
from __future__ import annotations
import json
from dataclasses import dataclass, field


@dataclass
class Budget:
    tool_calls: int = 60
    llm_calls: int = 12
    repairs: int = 3
    seconds: float = 900.0


def state_digest(session):
    try:
        p = session._pose()
        pose = (round(p["x"] / 0.1) * 0.1, round(p["y"] / 0.1) * 0.1, round(p["yaw_deg"] / 15) * 15, int(p["fallen"]))
        stopped = int(session.stopped)
    except Exception:
        pose, stopped = (None, None, None, None), 0
    mem = getattr(session, "memory", None)
    msize = 0
    if mem:
        try: msize = mem.count()
        except Exception: msize = 0
    return (pose, stopped, msize)


def _norm_args(args):
    return json.dumps(args or {}, sort_keys=True, default=str)


@dataclass
class LoopGuard:
    budget: Budget = field(default_factory=Budget)
    _history: list = field(default_factory=list)       # list of (tool, norm_args, digest)
    _fp_count: dict = field(default_factory=dict)
    tool_calls: int = 0
    blocked: int = 0

    def check(self, tool, args, session):
        """-> (allowed: bool, reason: str). Call BEFORE executing. Records the attempt for pattern detection."""
        digest = state_digest(session)
        fp = (tool, _norm_args(args), digest)
        # same fingerprint executed 2x already with no state change -> block the 3rd
        if self._fp_count.get(fp, 0) >= 2:
            self.blocked += 1
            return False, f"loop: {tool} with the same args and unchanged state ran {self._fp_count[fp]}x"
        # A-B-A-B-A oscillation on the last 5 tool identities (ignoring state)
        tools_seq = [h[0] for h in self._history[-4:]] + [tool]
        if len(tools_seq) == 5 and tools_seq[0] == tools_seq[2] == tools_seq[4] and tools_seq[1] == tools_seq[3] and tools_seq[0] != tools_seq[1]:
            self.blocked += 1
            return False, f"loop: oscillating {tools_seq[0]}-{tools_seq[1]} pattern"
        return True, ""

    def record(self, tool, args, session):
        """Call AFTER executing a (non-blocked) call so the next check sees it."""
        fp = (tool, _norm_args(args), state_digest(session))
        self._fp_count[fp] = self._fp_count.get(fp, 0) + 1
        self._history.append(fp)
        self.tool_calls += 1

    def over_budget(self, llm_calls, repairs, elapsed_s):
        if self.tool_calls > self.budget.tool_calls: return "tool-call budget exceeded"
        if llm_calls > self.budget.llm_calls: return "LLM-call budget exceeded"
        if repairs > self.budget.repairs: return "repair budget exceeded"
        if elapsed_s > self.budget.seconds: return "time budget exceeded"
        return None
