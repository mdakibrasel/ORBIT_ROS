"""ORBIT-PEL tool registry (v0.14 milestone M2): one explicit list of every tool the runtime supports, with typed argument schemas,
preconditions, a category, retry safety and a timeout.

    from orbit_ros.pel.registry import REGISTRY, validate_call, RegistryContext
    issues = validate_call({"tool": "move_forward", "args": {"distance": 3}}, RegistryContext(stopped=False))

Agreement with the simulator: for the robot tools (`world2d.ENV_TOOLS`) the registry accepts exactly the argument values that
`World2D.execute` accepts (tested in `tests/test_pel_registry.py`). Two simulator behaviours are mirrored on purpose and documented,
not "fixed", because changing them would change existing results:
  * `turn.degrees` is compared with `in (90, 180, 270)`, so 90.0 is accepted like 90;
  * unknown extra argument keys are ignored by every tool (the registry reports them as warnings, not errors).
Macro tools (`goto_cell`, `cover_region`) mirror the argument checks of `navtools.expand` (x and y go through int(), so "3" and 3.7
are accepted as 3); whether the cell is inside the room, free or reachable is a world-state question for the guard (M4), not the registry.
Agent tools (`ask`, `abstain`, `remember`) mirror `agent2d.run_episode.do`.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from .schema import Issue

@dataclass(frozen=True)
class Arg:
    kind: str                    # int | enum | any | text | intlike | cells
    required: bool = True
    min: int | None = None
    max: int | None = None
    values: tuple = ()
    doc: str = ""

    def check(self, v):
        """-> error message or None."""
        if self.kind == "int":
            if not isinstance(v, int) or isinstance(v, bool): return "must be an integer"
            if (self.min is not None and v < self.min) or (self.max is not None and v > self.max): return f"must be in {self.min}..{self.max}"
        elif self.kind == "enum":
            if v not in self.values: return f"must be one of {'|'.join(map(str, self.values))}"
        elif self.kind == "any":
            if v is None: return "must not be null"
        elif self.kind == "intlike":
            try: int(v)
            except (TypeError, ValueError): return "must be an integer"
        elif self.kind == "cells":
            try: {(int(c[0]), int(c[1])) for c in (v or [])}
            except (TypeError, ValueError, IndexError, KeyError): return "must be a list of [x, y] cells"
        return None

@dataclass(frozen=True)
class ToolSpec:
    name: str
    category: str                # motion | sensing | effector | answer | safety | agent | macro | control
    args: dict = field(default_factory=dict)
    pre: tuple = ()              # names of RegistryContext predicates that must hold
    side_effect: bool = False    # changes the world or the robot state
    retry_safe: bool = False     # may be repeated after a transient failure without changing the outcome
    timeout_s: float = 10.0      # wall-clock limit per call for adapters that can block (Webots, ROS 2)
    robot: bool = True           # sent to the robot adapter (False: handled by the agent runtime)
    prompt: str = ""             # the tool's line in a planner prompt (M6 prompts are generated from these)

@dataclass
class RegistryContext:
    """The runtime facts a precondition may depend on (robot-reported state and configuration, never evaluator ground truth)."""
    stopped: bool = False
    has_localiser: bool = False
    memory: bool = False
    nav_tools: bool = False
    asks_left: int = 2

PRECONDITIONS = {
    "not_stopped": (lambda c: not c.stopped, "robot is stopped"),
    "has_localiser": (lambda c: c.has_localiser, "no localiser on this robot"),
    "memory": (lambda c: c.memory, "no memory tool in this configuration"),
    "nav_tools": (lambda c: c.nav_tools, "navigation tools are not enabled"),
    "asks_left": (lambda c: c.asks_left > 0, "no more questions allowed"),
}

_T = []
def _t(*a, **k): _T.append(ToolSpec(*a, **k))
_t("move_forward", "motion", {"distance": Arg("int", min=0, max=12, doc="cells (1 cell = 1 m) along the current heading")}, ("not_stopped",),
   side_effect=True, prompt='{"tool":"move_forward","args":{"distance":int 0..12}}')
_t("turn", "motion", {"direction": Arg("enum", values=("left", "right")), "degrees": Arg("enum", values=(90, 180, 270))}, ("not_stopped",),
   side_effect=True, prompt='{"tool":"turn","args":{"direction":"left|right","degrees":90|180|270}}')
_t("clean", "effector", {}, ("not_stopped",), side_effect=True, retry_safe=True, prompt='{"tool":"clean","args":{}} (cleans current cell)')
_t("describe_view", "sensing", {}, ("not_stopped",), retry_safe=True, prompt='{"tool":"describe_view","args":{}}')
_t("get_pose", "sensing", {}, ("not_stopped",), retry_safe=True, prompt='{"tool":"get_pose","args":{}}')
_t("localize", "sensing", {}, ("not_stopped", "has_localiser"), retry_safe=True,
   prompt='{"tool":"localize","args":{}} (map-based localiser: corrects your pose estimate)')
_t("report", "answer", {"answer": Arg("any", doc="the answer to a question in the instruction")}, ("not_stopped",), side_effect=True,
   prompt='{"tool":"report","args":{"answer":str}} (states an answer to a question)')
_t("stop_robot", "safety", {}, (), side_effect=True, retry_safe=True, prompt='{"tool":"stop_robot","args":{}}')
_t("remember", "agent", {"label": Arg("any", required=False)}, ("memory",), retry_safe=True, robot=False,
   prompt='{"tool":"remember","args":{"label":str}} (stores nearest visible object of that label in memory)')
_t("ask", "agent", {"question": Arg("any", required=False)}, ("asks_left",), robot=False,
   prompt='{"tool":"ask","args":{"question":str}} (ask the person who gave the instruction; only if it is genuinely ambiguous)')
_t("abstain", "agent", {"reason": Arg("any", required=False)}, (), side_effect=True, robot=False,
   prompt='{"tool":"abstain","args":{"reason":str}} (refuse because the instruction is impossible or unsafe; ends the task)')
_t("goto_cell", "macro", {"x": Arg("intlike"), "y": Arg("intlike")}, ("not_stopped", "nav_tools"), side_effect=True, robot=False,
   prompt='{"tool":"goto_cell","args":{"x":int,"y":int}} (drives to that map cell on a shortest known-free path)')
_t("cover_region", "macro", {"side": Arg("enum", values=("left", "right", "front", "back", "all")), "avoid": Arg("cells", required=False)},
   ("not_stopped", "nav_tools"), side_effect=True, robot=False,
   prompt='{"tool":"cover_region","args":{"side":"left|right|front|back|all","avoid":[[x,y],...]}} (cleans every reachable cell of that half)')
_t("done", "control", {}, (), robot=False, prompt='{"tool":"done","args":{}} (the instruction is complete)')
_t("replan", "control", {"reason": Arg("any", required=False)}, (), robot=False, prompt='{"tool":"replan","args":{"reason":str}}')

REGISTRY = {s.name: s for s in _T}
ROBOT_TOOLS = frozenset(n for n, s in REGISTRY.items() if s.robot)

def validate_call(call, ctx: RegistryContext | None = None, check_pre: bool = True) -> list:
    """A tool call {"tool", "args"} -> list of Issue (empty = admissible). ctx=None or check_pre=False checks only name and arguments."""
    if not isinstance(call, dict) or not isinstance(call.get("tool"), str):
        return [Issue("malformed_action", "call", "a call is {\"tool\": str, \"args\": {...}}")]
    spec = REGISTRY.get(call["tool"])
    if spec is None: return [Issue("unknown_tool", "call.tool", f"{call['tool']!r} is not a registered tool")]
    args = call.get("args")
    if args is None: args = {}
    if not isinstance(args, dict): return [Issue("bad_type", "call.args", "args must be an object")]
    out = []
    for name, a in spec.args.items():
        if name not in args:
            if a.required: out.append(Issue("missing_field", f"call.args.{name}", f"{spec.name} needs {name}"))
            continue
        msg = a.check(args[name])
        if msg: out.append(Issue("bad_value", f"call.args.{name}", f"{spec.name}.{name} {msg}"))
    for k in args:
        if k not in spec.args: out.append(Issue("unknown_field", f"call.args.{k}", f"{spec.name} ignores {k}", "warning"))
    if check_pre and ctx is not None:
        for p in spec.pre:
            ok, why = PRECONDITIONS[p]
            if not ok(ctx): out.append(Issue("precondition", f"call.pre.{p}", why))
    return out

def admissible(call, ctx=None) -> bool: return not [i for i in validate_call(call, ctx) if i.severity == "error"]

def prompt_text(tools) -> str:
    """Planner-prompt tool list generated from the registry (used by PEL prompts; the existing prompts are unchanged)."""
    return "Tools (JSON): " + " | ".join(REGISTRY[t].prompt for t in tools)

def validate_plan_calls(plan, ctx=None) -> list:
    """Every step of a schema.PlanV1 through validate_call (no preconditions: they depend on the state at execution time)."""
    out = []
    for i, s in enumerate(plan.steps):
        for iss in validate_call({"tool": s.tool, "args": s.args}, ctx, check_pre=False):
            out.append(Issue(iss.code, f"steps[{i}]." + iss.where.split(".", 1)[1], iss.message, iss.severity))
    return out
