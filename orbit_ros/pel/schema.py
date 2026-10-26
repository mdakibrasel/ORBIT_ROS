"""ORBIT-PEL plan schema `orbit.plan/v1` (v0.14 milestone M1).

A plan is plain data: a goal, an ordered list of steps (tool call + optional precondition and expected observation), a completion
condition, constraints the planner declares for itself, a status (ok / needs_clarification / unsupported) and revision metadata.

    plan, issues = parse_plan(obj)          # never raises; accepts orbit.plan/v1 objects AND the legacy {"actions", "expect"} replies
    errors = [i for i in issues if i.severity == "error"]
    legacy = to_legacy(plan)                # the dict `agent2d.normalize_plan` would have produced, so the existing runner can execute it

This module only describes and checks plans. It does not execute anything, does not know argument ranges (that is the tool registry,
milestone M2) and never sees evaluator ground truth: `constraints.no_go_cells` are the planner's own reading of the instruction.

Legacy round trip: for every input x, `to_legacy(parse_plan(x)[0]) == agent2d.normalize_plan(x)`. Unknown keys of legacy actions and
plans are kept in `extra` / `legacy_extra` for that reason.
"""
from __future__ import annotations
import json
from dataclasses import dataclass, field

SCHEMA = "orbit.plan/v1"
STATUSES = ("ok", "needs_clarification", "unsupported")
POSE_KEYS = ("x", "y", "yaw_deg")
PRE_KEYS = {"robot_stopped": bool, "pose": dict, "sees": str, "localised": bool}
EXPECT_KEYS = {"pose": dict, "sees": str, "not_sees": str, "cleaned_cell": bool, "reported": bool}
COMPLETION_KEYS = {"pose": dict, "cleaned_min": int, "reported": bool, "visited": list}
CONSTRAINT_KEYS = {"no_go_cells": list, "max_steps": int, "stay_in_room": bool}
ON_FAILURE = ("replan", "retry", "ask", "stop")
PLAN_KEYS = {"schema", "plan_id", "revision", "parent_revision", "revision_reason", "goal", "status", "clarification",
             "unsupported_reason", "constraints", "steps", "completion", "made_for"}
STEP_KEYS = {"id", "tool", "args", "pre", "expect", "on_failure", "region", "note"}

def default_known_tools() -> frozenset:
    """Tool names the existing runtime understands (agent tools, macro tools, and the `done` control word).
    Name-level only: argument types and ranges belong to the registry (M2)."""
    from ..cognition import AGENT_TOOLS
    from ..navtools import MACROS
    return frozenset(AGENT_TOOLS | MACROS | {"done"})

@dataclass(frozen=True)
class Issue:
    code: str          # schema_error | missing_field | bad_type | bad_value | duplicate_id | unknown_tool | malformed_action | empty_plan | unknown_field
    where: str         # "plan", "steps[2]", "steps[2].args", "constraints.no_go_cells[0]", ...
    message: str
    severity: str = "error"     # "error" or "warning"
    def __str__(self): return f"{self.severity} {self.code} at {self.where}: {self.message}"

@dataclass
class Step:
    id: int
    tool: str
    args: dict = field(default_factory=dict)
    pre: dict = field(default_factory=dict)
    expect: dict = field(default_factory=dict)
    on_failure: str = "replan"
    region: str | None = None
    note: str = ""
    extra: dict = field(default_factory=dict)      # unknown keys of a legacy action (kept for the round trip)

    def call(self) -> dict: return {"tool": self.tool, "args": dict(self.args)}

@dataclass
class PlanV1:
    steps: list = field(default_factory=list)
    goal: str = ""
    status: str = "ok"
    clarification: str | None = None
    unsupported_reason: str | None = None
    constraints: dict = field(default_factory=dict)
    completion: dict = field(default_factory=dict)
    plan_id: str = ""
    revision: int = 0
    parent_revision: int | None = None
    revision_reason: str = ""
    made_for: dict = field(default_factory=dict)   # observation-derived context the plan was made for (pose, map digest, phase)
    source: str = SCHEMA                           # SCHEMA, "legacy" or "legacy_hierarchical"
    legacy_extra: dict | None = None               # the legacy plan dict minus its actions (round trip)

    def calls(self) -> list: return [s.call() for s in self.steps]

    def to_dict(self) -> dict:
        """orbit.plan/v1 JSON (no legacy bookkeeping)."""
        d = {"schema": SCHEMA, "plan_id": self.plan_id, "revision": self.revision, "goal": self.goal, "status": self.status,
             "steps": [_step_dict(s) for s in self.steps]}
        for k in ("parent_revision", "clarification", "unsupported_reason"):
            if getattr(self, k) is not None: d[k] = getattr(self, k)
        if self.revision_reason: d["revision_reason"] = self.revision_reason
        for k in ("constraints", "completion", "made_for"):
            if getattr(self, k): d[k] = json.loads(json.dumps(getattr(self, k)))
        return d

def _step_dict(s: Step) -> dict:
    d = {"id": s.id, "tool": s.tool, "args": dict(s.args)}
    if s.pre: d["pre"] = dict(s.pre)
    if s.expect: d["expect"] = dict(s.expect)
    if s.on_failure != "replan": d["on_failure"] = s.on_failure
    if s.region is not None: d["region"] = s.region
    if s.note: d["note"] = s.note
    return d

# ------------------------------------------------------------------------------------------------------------------------ parsing
def parse_plan(obj, known_tools=None) -> tuple:
    """Any model reply (already JSON-decoded) -> (PlanV1, [Issue]). Never raises.
    obj with "schema": "orbit.plan/v1" is read as a v1 plan; anything else goes through the legacy path, exactly as
    `agent2d.normalize_plan` would read it. known_tools=None uses `default_known_tools()`; pass an empty set to skip the name check."""
    known = default_known_tools() if known_tools is None else frozenset(known_tools)
    if isinstance(obj, dict) and "schema" in obj:
        if obj["schema"] != SCHEMA:
            return PlanV1(source=str(obj.get("schema"))[:40]), [Issue("schema_error", "plan.schema", f"unsupported schema {obj['schema']!r}; expected {SCHEMA!r}")]
        plan, issues = _parse_v1(obj)
    else:
        plan, issues = _parse_legacy(obj)
    issues += check_plan(plan, known)
    return plan, issues

def parse_plan_json(text: str, known_tools=None) -> tuple:
    """Raw text -> (PlanV1, [Issue]). Text that is not JSON gives an empty plan with one schema_error."""
    try: obj = json.loads(text)
    except (TypeError, ValueError) as e:
        return PlanV1(source="unparsed"), [Issue("schema_error", "plan", f"not JSON: {str(e)[:80]}")]
    return parse_plan(obj, known_tools)

def _parse_legacy(p) -> tuple:
    from ..agent2d import normalize_plan
    issues = []
    if not isinstance(p, (dict, list)): issues.append(Issue("schema_error", "plan", f"plan must be an object or a list, got {type(p).__name__}"))
    n = normalize_plan(p)
    hier = isinstance(p, dict) and isinstance(p.get("regions"), list) and not p.get("actions")
    steps = []
    for i, a in enumerate(n["actions"]):
        extra = {k: v for k, v in a.items() if k not in ("tool", "args")}
        if a["tool"] == "__malformed__": issues.append(Issue("malformed_action", f"steps[{i}]", f"action is not {{tool, args}}: {a['args'].get('raw', '')[:60]}"))
        steps.append(Step(id=i + 1, tool=a["tool"], args=a["args"], region=a.get("_region"), extra=extra))
    legacy_extra = {k: v for k, v in n.items() if k != "actions"}
    completion = {}
    e = n.get("expect")
    if isinstance(e, dict):
        pose = {k: e[k] for k in POSE_KEYS if k in e}
        if pose: completion["pose"] = pose
        if "cleaned_min" in e: completion["cleaned_min"] = e["cleaned_min"]
    elif e is not None: issues.append(Issue("bad_type", "plan.expect", "expect must be an object", "warning"))
    return PlanV1(steps=steps, completion=completion, source="legacy_hierarchical" if hier else "legacy", legacy_extra=legacy_extra), issues

def _parse_v1(d) -> tuple:
    issues = []
    for k in d:
        if k not in PLAN_KEYS: issues.append(Issue("unknown_field", f"plan.{k}", "not part of orbit.plan/v1; ignored", "warning"))
    raw_steps = d.get("steps")
    if raw_steps is None: issues.append(Issue("missing_field", "plan.steps", "a v1 plan needs a steps list")); raw_steps = []
    elif not isinstance(raw_steps, list): issues.append(Issue("bad_type", "plan.steps", "steps must be a list")); raw_steps = []
    steps = []
    for i, s in enumerate(raw_steps):
        w = f"steps[{i}]"
        if not isinstance(s, dict): issues.append(Issue("bad_type", w, "a step must be an object")); continue
        for k in s:
            if k not in STEP_KEYS: issues.append(Issue("unknown_field", f"{w}.{k}", "not part of a v1 step; ignored", "warning"))
        if "tool" not in s: issues.append(Issue("missing_field", f"{w}.tool", "every step needs a tool")); continue
        if not isinstance(s["tool"], str): issues.append(Issue("bad_type", f"{w}.tool", "tool must be a string")); continue
        if "id" not in s: issues.append(Issue("missing_field", f"{w}.id", "every step needs an integer id")); sid = None
        else: sid = s["id"]
        args = s.get("args", {})
        if args is None: args = {}
        steps.append(Step(id=sid, tool=s["tool"], args=args, pre=s.get("pre") or {}, expect=s.get("expect") or {},
                          on_failure=s.get("on_failure", "replan"), region=s.get("region"), note=s.get("note", "")))
    plan = PlanV1(steps=steps, goal=d.get("goal", ""), status=d.get("status", "ok"), clarification=d.get("clarification"),
                  unsupported_reason=d.get("unsupported_reason"), constraints=d.get("constraints") or {}, completion=d.get("completion") or {},
                  plan_id=d.get("plan_id", ""), revision=d.get("revision", 0), parent_revision=d.get("parent_revision"),
                  revision_reason=d.get("revision_reason", ""), made_for=d.get("made_for") or {})
    return plan, issues

# --------------------------------------------------------------------------------------------------------------------- checking
def _is_int(v): return isinstance(v, int) and not isinstance(v, bool)

def _check_pose(p, where, issues):
    if not isinstance(p, dict): issues.append(Issue("bad_type", where, "pose must be an object")); return
    for k, v in p.items():
        if k not in POSE_KEYS: issues.append(Issue("unknown_field", f"{where}.{k}", "pose keys are x, y, yaw_deg", "warning"))
        elif not _is_int(v): issues.append(Issue("bad_type", f"{where}.{k}", "pose values must be integers"))
        elif k == "yaw_deg" and v % 90: issues.append(Issue("bad_value", f"{where}.yaw_deg", "yaw_deg must be a multiple of 90"))

def _check_keys(obj, spec, where, issues):
    if not isinstance(obj, dict): issues.append(Issue("bad_type", where, "must be an object")); return
    for k, v in obj.items():
        if k not in spec: issues.append(Issue("unknown_field", f"{where}.{k}", f"allowed: {', '.join(spec)}", "warning")); continue
        t = spec[k]
        if t is int and not _is_int(v): issues.append(Issue("bad_type", f"{where}.{k}", "must be an integer"))
        elif t is not int and not isinstance(v, t): issues.append(Issue("bad_type", f"{where}.{k}", f"must be {t.__name__}"))
        elif k == "pose": _check_pose(v, f"{where}.pose", issues)

def _cells(v, where, issues):
    if not isinstance(v, list): return
    for j, c in enumerate(v):
        if not (isinstance(c, (list, tuple)) and len(c) == 2 and all(_is_int(x) for x in c)):
            issues.append(Issue("bad_type", f"{where}[{j}]", "a cell is [x, y] with integers"))

def check_plan(plan: PlanV1, known_tools=None) -> list:
    """Structural checks of a PlanV1 (also usable on plans built in code). Returns a list of Issue. Step ids, per-step pre / expect
    and on_failure are checked for v1 plans only; legacy plans get ids 1..n from their order."""
    known = default_known_tools() if known_tools is None else frozenset(known_tools)
    issues = []
    if plan.status not in STATUSES: issues.append(Issue("bad_value", "plan.status", f"status must be one of {STATUSES}"))
    if plan.status == "needs_clarification" and not (isinstance(plan.clarification, str) and plan.clarification.strip()):
        issues.append(Issue("missing_field", "plan.clarification", "needs_clarification requires the question to ask"))
    if plan.status == "unsupported" and not (isinstance(plan.unsupported_reason, str) and plan.unsupported_reason.strip()):
        issues.append(Issue("missing_field", "plan.unsupported_reason", "unsupported requires a reason"))
    if plan.status in ("needs_clarification", "unsupported") and plan.steps:
        issues.append(Issue("bad_value", "plan.steps", f"a plan with status {plan.status} must not contain steps"))
    if plan.status == "ok" and not plan.steps:
        issues.append(Issue("empty_plan", "plan.steps", "no steps (acceptable only if the instruction is already complete)", "warning"))
    if not _is_int(plan.revision) or plan.revision < 0: issues.append(Issue("bad_type", "plan.revision", "revision must be an integer >= 0"))
    if plan.parent_revision is not None and (not _is_int(plan.parent_revision) or not _is_int(plan.revision) or plan.parent_revision >= plan.revision):
        issues.append(Issue("bad_value", "plan.parent_revision", "parent_revision must be an integer smaller than revision"))
    if not isinstance(plan.goal, str): issues.append(Issue("bad_type", "plan.goal", "goal must be a string"))
    _check_keys(plan.constraints, CONSTRAINT_KEYS, "constraints", issues)
    if isinstance(plan.constraints, dict):
        _cells(plan.constraints.get("no_go_cells"), "constraints.no_go_cells", issues)
        ms = plan.constraints.get("max_steps")                  # v0.14.1: the planner's own step limit is checked (review M3)
        n_steps = sum(1 for s in plan.steps if s.tool != "done")
        if _is_int(ms) and n_steps > ms:
            issues.append(Issue("bad_value", "constraints.max_steps", f"the plan has {n_steps} steps, more than its own max_steps {ms}"))
        if plan.constraints.get("stay_in_room") is False:
            issues.append(Issue("bad_value", "constraints.stay_in_room", "ignored: the safety guard always keeps the robot in the room", "warning"))
    _check_keys(plan.completion, COMPLETION_KEYS, "completion", issues)
    if isinstance(plan.completion, dict): _cells(plan.completion.get("visited"), "completion.visited", issues)
    seen = set()
    for i, s in enumerate(plan.steps):
        w = f"steps[{i}]"
        if plan.source == SCHEMA:
            if s.id is None: pass                       # reported as missing_field by the parser
            elif not _is_int(s.id) or s.id < 1: issues.append(Issue("bad_type", f"{w}.id", "id must be an integer >= 1"))
            elif s.id in seen: issues.append(Issue("duplicate_id", f"{w}.id", f"step id {s.id} is used twice"))
            seen.add(s.id)
            if s.on_failure not in ON_FAILURE: issues.append(Issue("bad_value", f"{w}.on_failure", f"on_failure must be one of {ON_FAILURE}"))
            _check_keys(s.pre, PRE_KEYS, f"{w}.pre", issues); _check_keys(s.expect, EXPECT_KEYS, f"{w}.expect", issues)
            if not isinstance(s.args, dict): issues.append(Issue("bad_type", f"{w}.args", "args must be an object"))
        if known and s.tool != "__malformed__" and s.tool not in known:
            issues.append(Issue("unknown_tool", f"{w}.tool", f"{s.tool!r} is not a supported tool"))
    return issues

def errors(issues) -> list: return [i for i in issues if i.severity == "error"]

# ------------------------------------------------------------------------------------------------------------------- conversion
def to_legacy(plan: PlanV1) -> dict:
    """PlanV1 -> the legacy dict the existing runner executes. For a plan parsed from a legacy reply this is exactly
    `agent2d.normalize_plan(reply)`. For a native v1 plan: actions = the steps' calls (with `_region` when set), expect = the
    completion pose and cleaned_min. Per-step pre/expect have no legacy form and are dropped (the M4 manager reads them)."""
    if plan.legacy_extra is not None:
        return {**plan.legacy_extra, "actions": [{**s.extra, "tool": s.tool, "args": s.args} for s in plan.steps]}
    acts = []
    for s in plan.steps:
        a = {"tool": s.tool, "args": dict(s.args) if isinstance(s.args, dict) else s.args}
        if s.region is not None: a["_region"] = s.region
        acts.append(a)
    out = {"actions": acts}
    exp = dict(plan.completion.get("pose") or {}) if isinstance(plan.completion, dict) else {}
    if isinstance(plan.completion, dict) and "cleaned_min" in plan.completion: exp["cleaned_min"] = plan.completion["cleaned_min"]
    if exp: out["expect"] = exp
    return out

def from_legacy(obj, **meta) -> PlanV1:
    """Legacy reply -> a native v1 plan (drops the round-trip bookkeeping). meta sets plan_id, revision, goal, ..."""
    plan, _ = _parse_legacy(obj)
    plan.legacy_extra = None; plan.source = SCHEMA
    for s in plan.steps: s.extra = {}
    for k, v in meta.items(): setattr(plan, k, v)
    return plan
