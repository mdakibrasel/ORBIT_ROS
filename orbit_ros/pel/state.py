"""ORBIT-PEL persistent execution state `orbit.exec_state/v1` (v0.14 milestone M3).

What the execution manager (M4) knows about one episode, as versioned JSON that can be written atomically after every step:
plan revisions (with the reason for each), the cursor, completed steps, every attempt (call, status, message, duration), budgets used,
the current observation-derived fingerprint, a no-progress counter, and the terminal outcome. v0.14.1 adds three optional fields that a
resume needs: the no-go cells accumulated over ALL plan revisions of the instruction (`no_go_cells`), the cleaned-cell count when the
instruction started (`cleaned_at_start`, for `completion.cleaned_min`) and the cells the robot reports having visited (`visited_cells`,
for `completion.visited`). Records written by v0.14.0 lack them and still load (the fields get their defaults; the no-go set is then
rebuilt from every stored revision, see `accumulated_no_go`).

    st = ExecutionState(episode="goto|y90|...", config="PEL")
    st.start_plan(plan_v1, "initial", fingerprint(obs))
    st.record_attempt(step_id, call, "ok", "moved 2 m", 0.001, fingerprint(obs2)); st.advance()
    st.save(path); st2 = ExecutionState.load(path)
    reason = resume_blocker(st2, fingerprint(fresh_obs))     # None = safe to continue at st2.cursor

Persistent task memory = this record (plus the agent's landmark memory). Temporary context = the prompt built for one model call; it
is rebuilt each time and never stored as truth.

The fingerprint is built ONLY from what the robot reports (pose estimate, camera view, cells the agent believes blocked or cleaned).
It never reads World2D.x / .y or Task.expected: a resume decision must be one the robot itself could make.
"""
from __future__ import annotations
import hashlib, json, os, tempfile
from dataclasses import dataclass, field, asdict

SCHEMA = "orbit.exec_state/v1"
TERMINAL_CODES = ("done", "abstained", "needs_clarification", "unsupported", "budget_exhausted", "no_progress", "safe_stop",
                  "observation_failure", "timeout", "error")
MAX_OBS_LOG = 200

def _digest(obj) -> str: return hashlib.sha256(json.dumps(obj, sort_keys=True, default=list).encode()).hexdigest()[:12]

def fingerprint(pose: dict, view=None, obstacles_known=(), cleaned_known=(), stopped: bool = False) -> dict:
    """Observation-derived state key. pose: the robot's REPORTED pose (x, y, yaw_deg). view: camera detections (label, image_x,
    apparent_size). obstacles_known / cleaned_known: cells the agent believes. Nothing here may come from evaluator ground truth."""
    v = sorted((d.get("label"), round(float(d.get("image_x", 0)), 2), round(float(d.get("apparent_size", 0)), 2)) for d in (view or []))
    return {"pose": [pose["x"], pose["y"], pose["yaw_deg"]], "stopped": bool(stopped),
            "obstacles_hash": _digest(sorted(map(list, obstacles_known))), "cleaned": len(set(map(tuple, cleaned_known))),
            "view_hash": _digest(v)}

def progress_key(fp: dict) -> tuple:
    """The part of a fingerprint that counts as progress (pose, stop flag, cleaned cells); a new view alone is not progress."""
    return (tuple(fp["pose"]), fp["stopped"], fp["cleaned"])

def _pk_json(fp: dict) -> list: return [list(fp["pose"]), bool(fp["stopped"]), int(fp["cleaned"])]

@dataclass
class Attempt:
    step: int
    call: dict
    status: str              # contract status: ok | blocked | safety_stop | fault | invalid | stopped | timeout | rejected | vetoed
                             # | precondition_failed (v0.14.1: the step's own `pre` did not hold; nothing was sent)
    message: str
    duration_s: float
    revision: int
    fingerprint: dict | None = None

@dataclass
class ExecutionState:
    episode: str = ""
    config: str = ""
    phase: int = 0
    plan_revisions: list = field(default_factory=list)     # [{"revision", "reason", "plan", "made_for"}]
    cursor: int = 0
    completed_steps: list = field(default_factory=list)    # step ids of the CURRENT revision
    attempts: list = field(default_factory=list)
    budgets: dict = field(default_factory=lambda: {"tool_calls": 0, "llm_calls": 0, "tokens": 0, "replans": 0, "retries": 0,
                                                   "rejections": 0, "vetoes": 0})
    fingerprint: dict | None = None
    no_progress_steps: int = 0
    seen_calls: list = field(default_factory=list)          # [progress_key, call] pairs of executed calls (duplicate detection)
    observations: list = field(default_factory=list)       # compact log of fingerprints after each attempt (capped)
    terminal: dict | None = None
    seq: int = 0                                            # increases with every update; a saved file with a lower seq is older
    no_go_cells: list = field(default_factory=list)        # v0.14.1: [[x, y], ...] declared by any revision of this instruction
    cleaned_at_start: int | None = None                     # v0.14.1: believed cleaned cells when the instruction started
    visited_cells: list = field(default_factory=list)      # v0.14.1: [[x, y], ...] cells on the robot's reported path in this instruction

    # ------------------------------------------------------------------------------------------------------------ update rules
    @property
    def revision(self) -> int: return self.plan_revisions[-1]["revision"] if self.plan_revisions else -1
    @property
    def plan(self) -> dict | None: return self.plan_revisions[-1]["plan"] if self.plan_revisions else None
    def steps(self) -> list: return (self.plan or {}).get("steps", [])
    def current_step(self) -> dict | None:
        s = self.steps(); return s[self.cursor] if self.cursor < len(s) else None

    def _touch(self): self.seq += 1

    def start_plan(self, plan_dict: dict, reason: str, made_for: dict | None):
        """A new plan revision (plan_dict = PlanV1.to_dict()). Resets the cursor and the completed list of the previous revision."""
        if self.terminal: raise RuntimeError("episode already terminated")
        rev = self.revision + 1
        self.plan_revisions.append({"revision": rev, "reason": str(reason)[:300], "plan": json.loads(json.dumps(plan_dict)), "made_for": made_for})
        if rev > 0 and reason != "initial": self.budgets["replans"] += 1
        self.cursor, self.completed_steps = 0, []; self._touch()

    def record_attempt(self, step: int, call: dict, status: str, message: str, duration_s: float, fp: dict | None, robot_call: bool = True):
        a = Attempt(step, json.loads(json.dumps(call, default=str)), status, str(message)[:300], round(float(duration_s), 6), self.revision, fp)
        before = self.fingerprint                                # the state the call was issued from
        self.attempts.append(asdict(a))
        if robot_call and status not in ("rejected", "vetoed"): self.budgets["tool_calls"] += 1
        if status == "rejected": self.budgets["rejections"] += 1
        if status == "vetoed": self.budgets["vetoes"] += 1
        if fp is not None:
            if self.fingerprint is not None and progress_key(fp) == progress_key(self.fingerprint): self.no_progress_steps += 1
            else: self.no_progress_steps = 0
            self.fingerprint = fp
            self.observations.append(fp); del self.observations[:-MAX_OBS_LOG]
        if robot_call and status not in ("rejected", "vetoed"):
            if before is not None: self.seen_calls.append([_pk_json(before), a.call])    # keyed on the state the call was issued from
        self._touch()

    def is_duplicate(self, call: dict) -> bool:
        """The same call was already executed from the same progress state (it cannot yield anything new)."""
        if self.fingerprint is None: return False
        key = [_pk_json(self.fingerprint), json.loads(json.dumps(call, default=str))]
        return key in self.seen_calls

    def advance(self):
        s = self.current_step()
        if s is not None: self.completed_steps.append(s.get("id"))
        self.cursor += 1; self._touch()

    def count(self, budget: str, n: int = 1): self.budgets[budget] = self.budgets.get(budget, 0) + n; self._touch()

    def terminate(self, code: str, reason: str = ""):
        if code not in TERMINAL_CODES: raise ValueError(f"unknown terminal code {code}")
        if self.terminal is None: self.terminal = {"code": code, "reason": str(reason)[:300]}; self._touch()

    # ------------------------------------------------------------------------------------------------------------ persistence
    def to_dict(self) -> dict: return {"schema": SCHEMA, **asdict(self)}

    def save(self, path: str):
        """Atomic write: a temporary file in the same folder, then os.replace (never a half-written state file)."""
        d = os.path.dirname(os.path.abspath(path)); os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".state_", suffix=".json", dir=d)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f: json.dump(self.to_dict(), f, indent=1, sort_keys=True)
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp): os.remove(tmp)
            raise

    @classmethod
    def from_dict(cls, d: dict) -> "ExecutionState":
        if not isinstance(d, dict) or d.get("schema") != SCHEMA: raise ValueError(f"not an {SCHEMA} record")
        known = set(cls.__dataclass_fields__)
        unknown = set(d) - known - {"schema"}
        if unknown: raise ValueError(f"unknown fields in state record: {sorted(unknown)}")
        return cls(**{k: v for k, v in d.items() if k in known})

    @classmethod
    def load(cls, path: str) -> "ExecutionState":
        with open(path, encoding="utf-8") as f: return cls.from_dict(json.load(f))

def no_go_of(constraints) -> set:
    """Well-formed [x, y] cells of a plan's constraints.no_go_cells (malformed entries are a plan-level schema error, see schema.check_plan)."""
    v = constraints.get("no_go_cells") if isinstance(constraints, dict) else None
    if not isinstance(v, list): return set()
    return {tuple(c) for c in v if isinstance(c, (list, tuple)) and len(c) == 2 and all(isinstance(x, int) and not isinstance(x, bool) for x in c)}

def accumulated_no_go(st: "ExecutionState") -> set:
    """Every no-go cell of this instruction: the stored union (v0.14.1) plus the cells of EVERY plan revision (so a v0.14.0 record,
    which has no stored union, still restores the cells an earlier revision declared)."""
    out = {tuple(c) for c in st.no_go_cells or [] if isinstance(c, (list, tuple)) and len(c) == 2}
    for r in st.plan_revisions: out |= no_go_of((r.get("plan") or {}).get("constraints"))
    return out

def pending_failure(st: "ExecutionState") -> str | None:
    """The unhandled failure a crash left behind, or None. If the last attempt belongs to the current revision and to the step at the
    cursor and did not succeed, the process died after that attempt was saved but before the failure was handled (re-plan or stop).
    A resume must then handle the failure, never re-issue the step."""
    if not st.attempts or not st.plan_revisions: return None
    a, cur = st.attempts[-1], st.current_step()
    if a.get("revision") != st.revision or a.get("status") == "ok": return None
    if cur is None or a.get("step") != cur.get("id"): return None
    return f"{(a.get('call') or {}).get('tool')} {(a.get('call') or {}).get('args')} ended {a.get('status')}: {a.get('message', '')}"

# ---------------------------------------------------------------------------------------------------------------- staleness
def stale_reason(made_for: dict | None, fp: dict) -> str | None:
    """A stored plan is stale when the robot is not where the plan assumed, or the believed map changed since the plan was made."""
    if not made_for: return None
    if list(made_for.get("pose", [])) != list(fp["pose"]): return f"robot pose {fp['pose']} differs from the plan's start {made_for.get('pose')}"
    if made_for.get("obstacles_hash") != fp["obstacles_hash"]: return "the known obstacle map changed since the plan was made"
    if made_for.get("stopped") != fp["stopped"]: return "the robot's stop state changed"
    return None

def resume_blocker(st: ExecutionState, fresh_fp: dict) -> str | None:
    """None if the episode may continue at st.cursor; otherwise why not (the caller must re-observe and re-plan, or stop)."""
    if st.terminal: return f"episode already ended ({st.terminal['code']})"
    if st.fingerprint is None: return "no recorded observation to compare with"
    if fresh_fp["stopped"]: return "the robot is stopped"
    if list(fresh_fp["pose"]) != list(st.fingerprint["pose"]):
        return f"robot pose {fresh_fp['pose']} differs from the last recorded pose {st.fingerprint['pose']}"
    if fresh_fp["obstacles_hash"] != st.fingerprint["obstacles_hash"]: return "the known obstacle map differs from the recorded one"
    if fresh_fp["cleaned"] != st.fingerprint["cleaned"]: return "the cleaned-cell count differs from the recorded one"
    if fresh_fp["view_hash"] != st.fingerprint["view_hash"]: return "the camera view differs from the recorded one (something moved)"
    return None
