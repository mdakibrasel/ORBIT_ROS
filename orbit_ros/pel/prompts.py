"""ORBIT-PEL planner prompt for orbit.plan/v1 replies (v0.14 milestone M6). Optional: used only by PEL configurations run with
`--pel-prompt v1`. The existing prompts (backends2d.py) are not changed; without the switch PEL configurations get the same plan and
repair prompts as S1-S5, so a comparison isolates the execution layer.

The v1 prompt reuses the full observation context of LLMBackend._context (conventions, tool list, camera output, pose, known
obstacles, history, clarifications, instruction) and replaces only the reply-format paragraph: the model may declare per-step expected
observations, completion conditions, no-go cells, and a needs_clarification / unsupported status.
"""
from __future__ import annotations

PEL_V1_FORMAT = (
    '\nReturn ONLY JSON in this format (orbit.plan/v1), no markdown:\n'
    '{"schema":"orbit.plan/v1","status":"ok|needs_clarification|unsupported",'
    '"steps":[{"id":1,"tool":str,"args":{...},"expect":{"pose":{"x":int,"y":int,"yaw_deg":int}}}, ...],'
    '"completion":{"pose":{"x":int,"y":int},"cleaned_min":int},"constraints":{"no_go_cells":[[x,y],...]},'
    '"clarification":str,"unsupported_reason":str}\n'
    'Rules: steps are executed one at a time and checked against your "expect" (optional per step); "completion" says how the robot '
    'can tell the instruction is done; list in "no_go_cells" every cell the instruction forbids (the runtime will refuse to enter them); '
    'use status "needs_clarification" with a question when the instruction is genuinely ambiguous, "unsupported" with a reason when it '
    'is impossible or unsafe; then "steps" may be empty. Plan the whole instruction from the current pose.')

class PelV1Backend:
    """Wraps an LLMBackend: plan and repair ask for orbit.plan/v1; every other call and attribute is the wrapped backend's."""
    def __init__(self, inner): self.inner = inner
    def __getattr__(self, k): return getattr(self.inner, k)
    def plan(self, ph, ctx):
        return self.inner._plan_or_empty(self.inner._context(ph, ctx) + PEL_V1_FORMAT, "plan")
    def repair(self, ph, ctx, failure):
        p = (self.inner._context(ph, ctx) + f"\nThe previous plan failed: {failure}\nWrite a new plan for the REMAINING work starting from "
             "the current pose." + PEL_V1_FORMAT)
        return self.inner._plan_or_empty(p, "repair")
