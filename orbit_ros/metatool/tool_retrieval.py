"""Tool retrieval for the meta-tool (Phase 17, feature 8). Pure code: no robot, no LLM, no network.

Selects a relevant SUBSET of the registered tools for the planner prompt, using the normalized goal, tool
categories and tool names/arg-names/prompt text. The planner sees only the subset; the VALIDATOR always keeps the
full registered catalogue, so a hallucinated or omitted tool is still caught deterministically. Essential tools are
always included when they actually exist in the registry (verified against the live registry, not assumed).

Design choices (deterministic, testable):
  * score(tool) = keyword overlap (goal tokens vs tool name + arg names + category + prompt) + small category bonus;
  * ties broken by a stable order (registry insertion order);
  * the result always contains every essential tool THAT EXISTS, then the top-scored others, capped at k;
  * retrieval_miss(plan, subset) reports whether a validated plan needs a tool outside the subset, so the caller
    can do ONE bounded retry with the full catalogue and record the miss.

No tool is invented here and nothing bypasses the safety pipeline: this only chooses which existing tool schemas are
shown to the planner."""
from __future__ import annotations
import re

# Essential tools the planner should always be allowed to use when they exist. We verify existence against the
# live registry before including any of them (the prompt's list names some tools this project does not have, e.g.
# ask_user/say; those are simply skipped rather than invented).
ESSENTIAL_CANDIDATES = ("stop_robot", "stand_up", "ask_user", "get_status", "say")

# A light category hint per intent keyword, to bias retrieval toward the right family.
_CATEGORY_KEYWORDS = {
    "motion": ("go", "walk", "move", "turn", "step", "navigate", "approach", "forward", "backward", "side"),
    "effector": ("clean", "vacuum", "mop", "sweep", "wipe", "wave", "greet", "dust"),
    "sensing": ("look", "see", "describe", "view", "scan", "pose", "where", "camera", "check", "verify"),
    "macro": ("tour", "house", "room", "corner", "plan", "return", "join", "each", "all"),
    "safety": ("stop", "halt", "stand", "up", "recover", "fallen"),
}

_WORD = re.compile(r"[a-z0-9]+")


def _tools_dict(registry):
    if isinstance(registry, dict): return dict(registry)
    if hasattr(registry, "items"): return dict(registry.items())
    return {s.name: s for s in registry}


def _tokens(text):
    return set(_WORD.findall(str(text).lower()))


def essential_tools(registry):
    """The essential tools that ACTUALLY EXIST in this registry (verified; never invented)."""
    tools = _tools_dict(registry)
    return [t for t in ESSENTIAL_CANDIDATES if t in tools]


def _tool_text(name, spec):
    parts = [name, getattr(spec, "category", ""), getattr(spec, "prompt", "")]
    parts += list(getattr(spec, "args", {}) or {})
    for a in (getattr(spec, "args", {}) or {}).values():
        parts.append(getattr(a, "doc", ""))
    return " ".join(str(p) for p in parts)


def score_tool(goal_tokens, name, spec):
    """Deterministic relevance score of a tool for the goal tokens."""
    ttoks = _tokens(_tool_text(name, spec))
    overlap = len(goal_tokens & ttoks)
    # category bonus: if a goal token maps to this tool's category family
    cat = getattr(spec, "category", "")
    bonus = 0
    for g in goal_tokens:
        for c, kws in _CATEGORY_KEYWORDS.items():
            if g in kws and c == cat:
                bonus += 1
    return overlap * 2 + bonus


def retrieve(goal, registry, k=12, extra_names=()):
    """Return a list of tool names: all existing essential tools + top-scored others, capped at k (k>=#essential).

    goal: the instruction or normalized goal string. extra_names: tool names to force-include (e.g. tools the LLM's
    goal_struct already references). Stable and deterministic."""
    tools = _tools_dict(registry)
    order = list(tools.keys())                     # stable registry order
    rank = {n: i for i, n in enumerate(order)}
    goal_tokens = _tokens(goal)

    forced = []
    for n in list(essential_tools(registry)) + [x for x in extra_names if x in tools]:
        if n not in forced:
            forced.append(n)

    k = max(int(k), len(forced))                   # never drop an essential tool to honor k
    scored = sorted(
        (n for n in order if n not in forced),
        key=lambda n: (-score_tool(goal_tokens, n, tools[n]), rank[n]),
    )
    chosen = list(forced)
    for n in scored:
        if len(chosen) >= k: break
        chosen.append(n)
    # return in stable registry order for a reproducible prompt
    return [n for n in order if n in set(chosen)]


def subset_registry(registry, names):
    """A {name: ToolSpec} dict restricted to `names` (preserving specs). For building the planner prompt/schema."""
    tools = _tools_dict(registry)
    return {n: tools[n] for n in names if n in tools}


def retrieval_miss(plan, subset_names):
    """True if the plan uses a tool outside the retrieved subset (a miss -> caller retries with full catalogue)."""
    sub = set(subset_names)
    for s in getattr(plan, "steps", []) or []:
        if s.tool and s.tool not in sub:
            return True
    return False
