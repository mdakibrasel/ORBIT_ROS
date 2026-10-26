"""Static plan validation BEFORE any motion (section 4.4). Pure code: no robot, no LLM.

validate(plan, facts, done_ids=()) -> list[Issue]. A plan with feasible:false and a reason is valid (an explicit refusal)."""
from __future__ import annotations
from dataclasses import dataclass
import difflib

from . import spec as mspec
from .plan import Plan, REF_RE

MAX_STEPS = 20

# Read-only tools that never move the robot or an effector. Everything else is treated as "actionable"
# (it can cause motion/effector/cleaning). Kept as an explicit allow-list so a new tool is actionable by
# default (fail-safe). Used only by the deterministic REFUSAL_MISMATCH check (Phase 17, feature 2).
READ_ONLY_TOOLS = frozenset({
    "get_pose", "describe_view", "look", "look_around", "scan_obstacles", "motion_accuracy",
    "verify_cleaning", "verify_task", "clean_status", "plan_tasks", "house_info", "get_map",
})

# Words that signal an explicit refusal/inability in a free-text field. Matched on word boundaries,
# lower-cased, so object/place names that merely contain a substring do not trigger.
_REFUSAL_PHRASES = (
    "i cannot", "i can't", "cannot do", "can't do", "i will not", "i won't", "unable to",
    "not able to", "i refuse", "refusing", "not safe", "unsafe", "not possible", "impossible",
    "i must not", "will not perform", "cannot comply", "not permitted", "not allowed",
)


def _is_actionable(tool):
    """A step is actionable (may move the robot/effector) unless it is a known read-only tool."""
    return tool not in READ_ONLY_TOOLS


def _refusal_language(text):
    """Return the first explicit-refusal phrase found in `text` (word-ish match), else ''. Pure string check."""
    if not text:
        return ""
    low = " " + str(text).lower().replace("’", "'") + " "
    for p in _REFUSAL_PHRASES:
        # pad with spaces/punctuation tolerance: require the phrase to appear as a run, not inside a longer word
        idx = low.find(p)
        if idx >= 0:
            before = low[idx - 1] if idx > 0 else " "
            after_i = idx + len(p)
            after = low[after_i] if after_i < len(low) else " "
            if not before.isalnum() and not after.isalnum():
                return p
    return ""


@dataclass
class Issue:
    step: str
    code: str
    msg: str
    hint: str = ""

    def as_dict(self): return {"step": self.step, "code": self.code, "msg": self.msg, "hint": self.hint}


def _closest(name, names, n=3):
    return difflib.get_close_matches(name, list(names), n=n, cutoff=0.0)[:n]


def validate(plan, registry, facts, done_ids=(), check_refusal=True, check_complete=False):
    """registry: {name: ToolSpec} (HUMANOID_TOOLS or the live registry). facts: set of symbolic facts (mspec.facts).

    check_refusal (Phase 17, feature 2): when True, add the deterministic REFUSAL_MISMATCH issue if a plan both
    refuses and requests motion. A rejected plan must produce zero motion, so a self-contradictory plan is refused
    rather than executed. Set False only for code-level ablation comparisons; the executor always leaves it on.

    check_complete (Phase 17, feature 1): when True, add the INCOMPLETE issue for any goal_struct target not
    represented by a step or constraint, driving a bounded refinement round. Off by default (opt-in, conservative)."""
    if not isinstance(plan, Plan): plan = Plan.from_dict(plan)
    issues = []
    tools = dict(registry.items()) if hasattr(registry, "items") else {s.name: s for s in registry}

    actionable = [s for s in (plan.steps or []) if _is_actionable(s.tool)]

    if not plan.feasible:
        if not plan.refusal_reason:
            issues.append(Issue("", "EMPTY", "feasible:false needs a refusal_reason", "state why the request is refused"))
        # A refusal that still carries motion steps is contradictory: refuse it outright (zero motion).
        if check_refusal and actionable:
            issues.append(Issue(actionable[0].id, "REFUSAL_MISMATCH",
                                "feasible:false but the plan still contains actionable (motion) steps",
                                "a refusal must carry no motion steps; remove them or set feasible:true"))
        return issues                                  # an explicit refusal is otherwise valid

    # feasible:true but an explicit refusal_reason is set -> contradictory; treat as a refusal mismatch.
    if check_refusal and plan.refusal_reason and actionable:
        issues.append(Issue("", "REFUSAL_MISMATCH",
                            f"refusal_reason is set ({plan.refusal_reason!r}) while feasible:true with motion steps",
                            "clear refusal_reason to act, or set feasible:false and remove motion steps"))

    # explicit refusal LANGUAGE in a free-text field while requesting motion -> refuse (do not infer permission).
    if check_refusal and actionable:
        for field_name, text in (("goal", plan.goal), ("done_when", plan.done_when)):
            ph = _refusal_language(text)
            if ph:
                issues.append(Issue("", "REFUSAL_MISMATCH",
                                    f"refusal language ({ph!r}) in {field_name} while the plan requests motion",
                                    "if refusing, set feasible:false with no motion steps; otherwise remove the refusal wording"))
                break
        else:
            for s in actionable:
                ph = _refusal_language(s.why)
                if ph:
                    issues.append(Issue(s.id, "REFUSAL_MISMATCH",
                                        f"refusal language ({ph!r}) in step {s.id} 'why' while it requests motion",
                                        "a step that says it refuses must not also be executed; fix the plan"))
                    break

    steps = plan.steps
    if not steps:
        return [Issue("", "EMPTY", "a feasible plan needs at least one step", "add steps or set feasible:false with a reason")]
    if len(steps) > MAX_STEPS:
        issues.append(Issue("", "TOO_LONG", f"{len(steps)} steps > {MAX_STEPS}", "split the task or use a macro tool"))

    ids = [s.id for s in steps]
    seen = set()
    for s in steps:
        if s.id in seen: issues.append(Issue(s.id, "DUP_ID", f"duplicate step id {s.id!r}", "give every step a unique id"))
        seen.add(s.id)
    idset = set(ids)

    for s in steps:
        if s.on_fail not in ("repair", "skip", "abort"):
            issues.append(Issue(s.id, "BAD_ONFAIL", f"on_fail={s.on_fail!r}", "use repair | skip | abort"))
        for a in s.after:
            if a not in idset: issues.append(Issue(s.id, "BAD_DEP", f"depends on unknown step {a!r}", "reference an existing step id"))

    # cycle check
    try:
        plan.topo_order()
    except ValueError as e:
        if "cycle" in str(e): issues.append(Issue("", "CYCLE", str(e), "remove the circular dependency"))

    # per-step tool + args
    for s in steps:
        spec = tools.get(s.tool)
        if spec is None:
            issues.append(Issue(s.id, "UNKNOWN_TOOL", f"{s.tool!r} is not a tool", "closest: " + ", ".join(_closest(s.tool, tools))))
            continue
        if not isinstance(s.args, dict):
            issues.append(Issue(s.id, "ARG_TYPE", "args must be an object", "use a JSON object for args")); continue
        for k in s.args:
            if k not in spec.args:
                issues.append(Issue(s.id, "ARG_UNKNOWN", f"unknown arg {k!r} for {s.tool}",
                                    _spell_hint(k, spec.args)))
        for name, a in spec.args.items():
            if name not in s.args:
                if a.required: issues.append(Issue(s.id, "ARG_MISSING", f"{s.tool} needs {name}", f"add {name}"))
                continue
            v = s.args[name]
            if _is_ref(v): continue                     # a ${ref} is resolved at exec time; type-checked then
            err = a.check(v)
            if err:
                code = ("ARG_ENUM" if a.kind == "enum" else "ARG_RANGE" if ("in " in err and a.kind == "int") else "ARG_TYPE")
                hint = ""
                if a.kind == "enum" and isinstance(v, str):
                    cand = [x for x in a.values if isinstance(x, str) and x.lower().replace("_", "") == v.lower().replace("_", "")]
                    if cand: hint = f"did you mean {cand[0]!r}?"
                issues.append(Issue(s.id, code, f"{s.tool}.{name} {err}", hint))

    # ${ref} legality: only to an ancestor in `after`
    pos = {s.id: i for i, s in enumerate(steps)}
    for s in steps:
        anc = _ancestors(plan, s.id)
        for sid, _fld in s.refs():
            if sid not in anc:
                issues.append(Issue(s.id, "BAD_REF", f"${{{sid}.{_fld}}} is not an ancestor of {s.id}",
                                    f"add {sid} to {s.id}.after, or reference an earlier step"))

    # REPEAT: identical consecutive call on a non-repeatable tool
    for a, b in zip(steps, steps[1:]):
        if a.tool == b.tool and a.args == b.args and not mspec.spec_for(a.tool).repeatable:
            issues.append(Issue(b.id, "REPEAT", f"{b.tool} repeated identically right after {a.id}", "a non-repeatable tool should run once"))

    # PRECONDITION: forward-simulate requires/effects in topological order
    issues += _forward_sim(plan, set(facts), done_ids)

    # INCOMPLETE (Phase 17, feature 1): every required target named in goal_struct must be represented in the plan
    # (as a step arg value) or in the compiled constraints. Conservative: only flags targets that are known rooms/
    # objects and appear NOWHERE, to avoid false positives that would block otherwise-valid plans.
    if check_complete:
        issues += _completeness(plan, tools, facts)

    return issues


def _norm_name(x):
    return str(x).strip().lower().replace(" ", "_")


def _completeness(plan, tools, facts):
    """Return INCOMPLETE issues for goal_struct targets not represented anywhere in the plan/constraints."""
    gs = plan.goal_struct or {}
    targets = [t for t in (gs.get("targets") or []) if str(t).strip()]
    if not targets:
        return []
    # the set of known names we are willing to assert about (rooms + house items + taught places from facts)
    known = set()
    for f in facts:
        if f.startswith("known("):
            known.add(_norm_name(f[len("known("):-1]))
    # gather every string value present in step args + the constraints text
    present = set()
    for s in plan.steps:
        for v in _arg_strings(s.args):
            present.add(_norm_name(v))
    constraint_blob = " ".join(_norm_name(c) for c in (gs.get("constraints") or []))
    out = []
    for t in targets:
        nt = _norm_name(t)
        if nt not in known:
            continue                       # only assert about targets we can resolve to a known name
        if nt in present:
            continue
        if nt and nt in constraint_blob:
            continue
        out.append(Issue("", "INCOMPLETE", f"goal target {t!r} is not addressed by any step or constraint",
                         f"add a step that targets {t!r}, or drop it from goal_struct.targets"))
    return out


def _arg_strings(args):
    if isinstance(args, str):
        yield args
    elif isinstance(args, dict):
        for v in args.values(): yield from _arg_strings(v)
    elif isinstance(args, (list, tuple)):
        for v in args: yield from _arg_strings(v)


def _spell_hint(bad, argmap):
    cand = [x for x in argmap if x.lower().replace("_", "") == bad.lower().replace("_", "")]
    if cand: return f"did you mean {cand[0]!r}?"
    close = difflib.get_close_matches(bad, list(argmap), n=1, cutoff=0.0)
    return (f"known args: {', '.join(argmap)}" + (f"; closest {close[0]!r}" if close else ""))


def _is_ref(v):
    return isinstance(v, str) and REF_RE.search(v) is not None


def _ancestors(plan, sid):
    """All transitive predecessors of sid via `after`."""
    anc, stack = set(), list(plan.by_id(sid).after if plan.by_id(sid) else [])
    while stack:
        a = stack.pop()
        if a in anc: continue
        anc.add(a); st = plan.by_id(a)
        if st: stack += st.after
    return anc


def _forward_sim(plan, facts, done_ids):
    """Walk steps in topological order, check each tool's requires against the simulated fact set, then apply its effects."""
    issues = []
    try: order = plan.topo_order()
    except ValueError: return issues                   # cycle/bad-dep already reported
    # done steps are assumed to have applied their effects already
    done = set(done_ids)
    for s in order:
        if s.id in done: _apply_effects(facts, mspec.spec_for(s.tool), s.args); continue
        spec = mspec.spec_for(s.tool)
        for req in spec.requires:
            filled = mspec.fill(req, s.args)
            if filled is None: continue                 # arg missing: ARG_MISSING already reported
            neg = filled.startswith("!")
            pred = filled[1:] if neg else filled
            holds = pred in facts
            if neg and holds:
                issues.append(Issue(s.id, "PRECONDITION", f"{s.tool} requires NOT {pred} but it holds", _pre_hint(pred)))
            if not neg and not holds:
                issues.append(Issue(s.id, "PRECONDITION", f"{s.tool} requires {pred} which is not established yet", _pre_hint(pred)))
        _apply_effects(facts, spec, s.args)
    return issues


def _apply_effects(facts, spec, args):
    for eff in spec.effects:
        filled = mspec.fill(eff, args)
        if filled is None: continue
        if filled.startswith("-"):
            pat = filled[1:]
            if pat.endswith("(*)"):
                pref = pat[:-3] + "("
                for f in [x for x in facts if x.startswith(pref)]: facts.discard(f)
            else: facts.discard(pat)
        elif filled.startswith("!"):
            facts.discard(filled[1:])
        else:
            facts.add(filled)


def _pre_hint(pred):
    if pred.startswith("known("):
        name = pred[len("known("):-1]
        return f"'{name}' is not a known room or house item; name a known target or refuse (feasible:false)"
    if pred == "upright": return "the robot is fallen; stand_up first"
    if pred == "not_stopped": return "the robot is stopped; load_scene restarts it"
    return f"establish {pred} before this step"
