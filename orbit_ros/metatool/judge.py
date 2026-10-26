"""Optional advisory plan judge for the meta-tool (Phase 17, feature 9). Pure code: no robot, no LLM.

DISABLED by default. When enabled, it reviews a (already schema-valid) plan for missing, redundant or
contradictory actions and returns advisory notes. The judge is STRICTLY ADVISORY: it can never override the
validator, the safety gate, the contract or execution checks. The caller may use its notes to drive at most three
refinement rounds; a plan that the judge dislikes but that passes all hard checks is still allowed.

This is a deterministic structural judge (not an LLM critic): it flags patterns that are cheap to detect and
unambiguous, so it adds measurable value without introducing an unsupported self-critique loop."""
from __future__ import annotations
from dataclasses import dataclass, field

MAX_ROUNDS = 3


@dataclass
class JudgeNote:
    kind: str           # missing | redundant | contradictory
    detail: str
    step: str = ""

    def as_dict(self):
        return {"kind": self.kind, "detail": self.detail, "step": self.step}


@dataclass
class JudgeResult:
    ok: bool                      # True when no advisory notes
    notes: list = field(default_factory=list)

    def as_dict(self):
        return {"ok": self.ok, "notes": [n.as_dict() for n in self.notes]}


def review(plan):
    """Return a JudgeResult with advisory notes. Never raises; never mutates the plan."""
    notes = []
    steps = list(getattr(plan, "steps", []) or [])

    # redundant: the exact same (tool, args) appears more than once
    seen = {}
    for s in steps:
        key = (s.tool, _freeze(s.args))
        if key in seen:
            notes.append(JudgeNote("redundant", f"{s.tool} with identical args repeats step {seen[key]!r}", s.id))
        else:
            seen[key] = s.id

    # contradictory: a stop_robot before later motion steps, or go_to the same target twice with other work between
    stopped_at = None
    for i, s in enumerate(steps):
        if s.tool == "stop_robot":
            stopped_at = i
        elif stopped_at is not None and _is_motion(s.tool):
            notes.append(JudgeNote("contradictory",
                                   f"{s.tool} is planned after stop_robot (step index {stopped_at}); the robot would be stopped",
                                   s.id))

    # missing: a goal target named in goal_struct that no step addresses (advisory mirror of INCOMPLETE,
    # but non-blocking and without needing the fact set)
    gs = getattr(plan, "goal_struct", None) or {}
    present = set()
    for s in steps:
        for v in _arg_values(s.args):
            present.add(str(v).strip().lower().replace(" ", "_"))
    for t in (gs.get("targets") or []):
        nt = str(t).strip().lower().replace(" ", "_")
        if nt and nt not in present:
            notes.append(JudgeNote("missing", f"goal target {t!r} is not addressed by any step"))

    return JudgeResult(ok=not notes, notes=notes)


def _is_motion(tool):
    return tool not in (
        "get_pose", "describe_view", "look", "look_around", "scan_obstacles", "motion_accuracy",
        "verify_cleaning", "verify_task", "clean_status", "plan_tasks", "stop_robot",
    )


def _freeze(args):
    import json
    try:
        return json.dumps(args or {}, sort_keys=True, default=str)
    except Exception:
        return str(args)


def _arg_values(args):
    if isinstance(args, dict):
        for v in args.values(): yield from _arg_values(v)
    elif isinstance(args, (list, tuple)):
        for v in args: yield from _arg_values(v)
    else:
        yield args
