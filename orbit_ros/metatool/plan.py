"""Meta-tool plan data model (section 4.2). Pure data + helpers; no robot, no LLM, standard library only."""
from __future__ import annotations
import json, re
from dataclasses import dataclass, field, asdict

REF_RE = re.compile(r"\$\{(\w+)\.(\w+)\}")          # ${stepId.field}


@dataclass
class Step:
    id: str
    tool: str
    args: dict = field(default_factory=dict)
    after: list = field(default_factory=list)       # ids this step depends on
    on_fail: str = "repair"                          # "repair" | "skip" | "abort"
    why: str = ""
    # runtime fields
    status: str = "pending"                          # pending|running|done|failed|skipped
    result: dict = field(default_factory=dict)
    attempts: int = 0

    def refs(self):
        """(step_id, field) pairs this step's args reference via ${id.field}."""
        out = []
        for v in _walk_strings(self.args):
            out += REF_RE.findall(v)
        return out


@dataclass
class Plan:
    goal: str = ""
    goal_struct: dict = field(default_factory=dict)   # {verb, targets[], constraints[], negations[]}
    feasible: bool = True
    refusal_reason: str = ""
    steps: list = field(default_factory=list)         # list[Step]
    done_when: str = ""
    revision: int = 0

    @staticmethod
    def from_dict(d: dict) -> "Plan":
        steps = []
        for s in d.get("steps", []) or []:
            steps.append(Step(id=str(s.get("id", "")), tool=str(s.get("tool", "")), args=dict(s.get("args", {}) or {}),
                              after=list(s.get("after", []) or []), on_fail=str(s.get("on_fail", "repair")),
                              why=str(s.get("why", ""))))
        return Plan(goal=str(d.get("goal", "")), goal_struct=dict(d.get("goal_struct", {}) or {}),
                    feasible=bool(d.get("feasible", True)), refusal_reason=str(d.get("refusal_reason", "")),
                    steps=steps, done_when=str(d.get("done_when", "")), revision=int(d.get("revision", 0)))

    def to_dict(self) -> dict:
        return {"goal": self.goal, "goal_struct": self.goal_struct, "feasible": self.feasible,
                "refusal_reason": self.refusal_reason, "done_when": self.done_when, "revision": self.revision,
                "steps": [{"id": s.id, "tool": s.tool, "args": s.args, "after": s.after, "on_fail": s.on_fail,
                           "why": s.why, "status": s.status} for s in self.steps]}

    def by_id(self, sid):
        for s in self.steps:
            if s.id == sid: return s
        return None

    def topo_order(self):
        """Steps in dependency order, keeping the written order where dependencies allow. Raises ValueError on a cycle or bad dep."""
        ids = [s.id for s in self.steps]
        if len(set(ids)) != len(ids): raise ValueError("duplicate step id")
        idset = set(ids)
        for s in self.steps:
            for a in s.after:
                if a not in idset: raise ValueError(f"step {s.id} depends on unknown step {a}")
        ordered, placed = [], set()
        remaining = list(self.steps)
        progressed = True
        while remaining and progressed:
            progressed = False
            for s in list(remaining):                 # written order preserved among the ready ones
                if all(a in placed for a in s.after):
                    ordered.append(s); placed.add(s.id); remaining.remove(s); progressed = True
        if remaining:
            raise ValueError(f"cycle among steps: {[s.id for s in remaining]}")
        return ordered

    def pseudo_code(self):
        """Human-readable plan for the scratchpad/HUD, with ticks for completed steps."""
        lines = [f"GOAL: {self.goal}" + (f"  (revision {self.revision})" if self.revision else "")]
        if not self.feasible:
            lines.append(f"REFUSED: {self.refusal_reason}"); return "\n".join(lines)
        mark = {"done": "[x]", "failed": "[!]", "skipped": "[-]", "running": "[>]"}
        for s in self.steps:
            m = mark.get(s.status, "[ ]")
            a = f" after {s.after}" if s.after else ""
            lines.append(f"  {m} {s.id}: {s.tool}({json.dumps(s.args)}){a}  on_fail={s.on_fail}" + (f"  # {s.why}" if s.why else ""))
        return "\n".join(lines)


def _walk_strings(obj):
    if isinstance(obj, str): yield obj
    elif isinstance(obj, dict):
        for v in obj.values(): yield from _walk_strings(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj: yield from _walk_strings(v)


def resolve_refs(args, results, allowed_ancestors):
    """Replace ${id.field} in args using results[id][field]. Only ids in allowed_ancestors may be referenced.
    Returns (resolved_args, errors). A whole-string ref keeps the referenced value's type; an embedded ref is stringified."""
    errors = []
    def sub(v):
        if isinstance(v, str):
            m = REF_RE.fullmatch(v.strip())
            if m:
                sid, fld = m.group(1), m.group(2)
                if sid not in allowed_ancestors: errors.append(f"ref ${{{sid}.{fld}}} is not an ancestor"); return v
                src = (results.get(sid) or {})
                if fld not in src: errors.append(f"ref ${{{sid}.{fld}}} has no field {fld}"); return v
                return src[fld]
            def repl(mm):
                sid, fld = mm.group(1), mm.group(2)
                if sid not in allowed_ancestors: errors.append(f"ref ${{{sid}.{fld}}} is not an ancestor"); return mm.group(0)
                src = (results.get(sid) or {})
                if fld not in src: errors.append(f"ref ${{{sid}.{fld}}} has no field {fld}"); return mm.group(0)
                return str(src[fld])
            return REF_RE.sub(repl, v)
        if isinstance(v, dict): return {k: sub(x) for k, x in v.items()}
        if isinstance(v, list): return [sub(x) for x in v]
        return v
    return sub(args), errors


def extract_json(text):
    """Strip <think>...</think> blocks and code fences, then return the first balanced JSON object (string-aware). None if not found."""
    if not text: return None
    t = re.sub(r"<think>.*?</think>", " ", text, flags=re.DOTALL | re.IGNORECASE)
    t = re.sub(r"```(?:json)?", " ", t)
    start = t.find("{")
    while start != -1:
        depth, i, in_str, esc = 0, start, False, False
        while i < len(t):
            c = t[i]
            if in_str:
                if esc: esc = False
                elif c == "\\": esc = True
                elif c == '"': in_str = False
            else:
                if c == '"': in_str = True
                elif c == "{": depth += 1
                elif c == "}":
                    depth -= 1
                    if depth == 0:
                        chunk = t[start:i + 1]
                        try: return json.loads(chunk)
                        except ValueError: break
            i += 1
        start = t.find("{", start + 1)
    return None
