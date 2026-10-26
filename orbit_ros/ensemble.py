"""Backend wrappers (v0.12).

SelfConsistent(backend, k): self-consistency voting. Every plan-type request is sampled k times; the plans are dry-run in the believed
    world and grouped by their predicted outcome (final cell, heading, number of cleaned cells, error-free or not). The largest group wins
    (ties: the error-free group, then the earliest sample). Single-action requests vote on the exact JSON call. Costs k model calls per
    request; with a real model it only makes sense at temperature > 0 (`--temperature 0.7 --self-consistency 5`).

Mixed(planner, stepper): model mixing. plan / repair / assess / write_lesson go to `planner` (e.g. a large model), the per-step calls of
    S0 and C1-C4 (next_action, plan_step, scratchpad_step) to `stepper` (e.g. a small, fast model). Calls and usage are summed over both."""
import json
from collections import Counter
from .cognition import dry_run

def _calls(plan):
    """The tool calls of a plan: legacy {"actions": [...]} or orbit.plan/v1 {"steps": [...]} (v0.14.1, review L5: v1 samples were all
    given the same key, so the vote always picked sample 1)."""
    if "actions" in plan or not isinstance(plan.get("steps"), list): return plan.get("actions", [])
    return [{"tool": s.get("tool"), "args": s.get("args") or {}} for s in plan["steps"] if isinstance(s, dict) and s.get("tool") != "done"]

def _key(plan, ctx):
    try:
        fin, cleaned, errs = dry_run(_calls(plan), ctx["pose"], [tuple(o) for o in ctx["obstacles_known"]], ctx.get("start_pose"))
        return (fin["x"], fin["y"], fin["yaw_deg"], len(cleaned), not errs)
    except Exception:
        return ("invalid",)

class SelfConsistent:
    def __init__(self, backend, k=3):
        self.inner, self.k = backend, max(1, int(k)); self.votes = []      # votes: [(winning share, k)] per request, for the report
    def __getattr__(self, name): return getattr(self.inner, name)
    def _vote_plans(self, ctx, sample):
        plans = [sample() for _ in range(self.k)]
        keys = [_key(p, ctx) for p in plans]; cnt = Counter(keys)
        best = max(cnt, key=lambda q: (cnt[q], q[-1] is True, -keys.index(q)))
        self.votes.append((cnt[best], self.k))
        return plans[keys.index(best)]
    def _vote_action(self, sample):
        acts = [sample() for _ in range(self.k)]
        keys = [json.dumps(a, sort_keys=True, default=str) for a in acts]; cnt = Counter(keys)
        best = max(cnt, key=lambda q: (cnt[q], -keys.index(q)))
        self.votes.append((cnt[best], self.k))
        return acts[keys.index(best)]
    def plan(self, ph, ctx): return self._vote_plans(ctx, lambda: self.inner.plan(ph, ctx))
    def repair(self, ph, ctx, failure): return self._vote_plans(ctx, lambda: self.inner.repair(ph, ctx, failure))
    def plan_step(self, ph, ctx): return self._vote_plans(ctx, lambda: self.inner.plan_step(ph, ctx))
    def next_action(self, ph, ctx): return self._vote_action(lambda: self.inner.next_action(ph, ctx))
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        return self._vote_action(lambda: self.inner.scratchpad_step(ph, ctx, actions, cursor, constrained))

class Mixed:
    def __init__(self, planner, stepper):
        self.planner, self.stepper = planner, stepper
        self.usage_events = []                         # merged in call order, each tagged with its role
        self._seen = {"planner": len(planner.usage_events), "stepper": len(stepper.usage_events)}
    def __getattr__(self, name): return getattr(self.planner, name)
    @property
    def calls(self): return self.planner.calls + self.stepper.calls
    @property
    def parse_errors(self): return getattr(self.planner, "parse_errors", 0) + getattr(self.stepper, "parse_errors", 0)
    def _run(self, role, fn, *a):
        b = getattr(self, role); out = fn(*a); ev = b.usage_events
        self.usage_events += [dict(u, role=role) for u in ev[self._seen[role]:]]; self._seen[role] = len(ev)
        return out
    def begin_episode(self, key): self.planner.begin_episode(key); self.stepper.begin_episode(key + "|stepper")
    def plan(self, ph, ctx): return self._run("planner", self.planner.plan, ph, ctx)
    def repair(self, ph, ctx, failure): return self._run("planner", self.planner.repair, ph, ctx, failure)
    def assess(self, ph, ctx): return self._run("planner", self.planner.assess, ph, ctx)
    def write_lesson(self, task, row, trace): return self._run("planner", self.planner.write_lesson, task, row, trace)
    def next_action(self, ph, ctx): return self._run("stepper", self.stepper.next_action, ph, ctx)
    def plan_step(self, ph, ctx): return self._run("stepper", self.stepper.plan_step, ph, ctx)
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        return self._run("stepper", self.stepper.scratchpad_step, ph, ctx, actions, cursor, constrained)

class TokenBudget:
    """Per-episode token budget (input + output tokens as reported by the provider). Once the budget is spent, no further model call is
    made: plans come back empty, step requests return `done`, assess says complete, so the episode ends. `exhausted` is logged."""
    def __init__(self, backend, max_tokens):
        self.inner, self.max_tokens, self.exhausted, self._u0 = backend, int(max_tokens), False, len(backend.usage_events)
    def __getattr__(self, name): return getattr(self.inner, name)
    def begin_episode(self, key): self.inner.begin_episode(key); self._u0 = len(self.inner.usage_events); self.exhausted = False
    def used(self):
        return sum((u.get("input_tokens") or 0) + (u.get("output_tokens") or 0) for u in self.inner.usage_events[self._u0:])
    def _ok(self):
        if self.used() >= self.max_tokens: self.exhausted = True
        return not self.exhausted
    def plan(self, ph, ctx): return self.inner.plan(ph, ctx) if self._ok() else {"actions": []}
    def repair(self, ph, ctx, failure): return self.inner.repair(ph, ctx, failure) if self._ok() else {"actions": []}
    def plan_step(self, ph, ctx): return self.inner.plan_step(ph, ctx) if self._ok() else {"actions": []}
    def next_action(self, ph, ctx): return self.inner.next_action(ph, ctx) if self._ok() else {"tool": "done", "args": {}}
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        return self.inner.scratchpad_step(ph, ctx, actions, cursor, constrained) if self._ok() else {"tool": "done", "args": {}}
    def assess(self, ph, ctx): return self.inner.assess(ph, ctx) if self._ok() else True

class Deadline:
    """Per-episode wall-clock limit on model decisions (v0.13). After `seconds`, no further call is made (plans empty, steps `done`),
    so the episode ends; `timed_out` is logged and the runner sets outcome_code = timeout."""
    def __init__(self, backend, seconds, clock=None):
        import time
        self.inner, self.seconds, self.clock, self.timed_out, self.t0 = backend, float(seconds), clock or time.monotonic, False, None
    def __getattr__(self, name): return getattr(self.inner, name)
    def begin_episode(self, key): self.inner.begin_episode(key); self.t0 = self.clock(); self.timed_out = False
    def _ok(self):
        if self.t0 is not None and self.clock() - self.t0 > self.seconds: self.timed_out = True
        return not self.timed_out
    def plan(self, ph, ctx): return self.inner.plan(ph, ctx) if self._ok() else {"actions": []}
    def repair(self, ph, ctx, failure): return self.inner.repair(ph, ctx, failure) if self._ok() else {"actions": []}
    def plan_step(self, ph, ctx): return self.inner.plan_step(ph, ctx) if self._ok() else {"actions": []}
    def next_action(self, ph, ctx): return self.inner.next_action(ph, ctx) if self._ok() else {"tool": "done", "args": {}}
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        return self.inner.scratchpad_step(ph, ctx, actions, cursor, constrained) if self._ok() else {"tool": "done", "args": {}}
    def assess(self, ph, ctx): return self.inner.assess(ph, ctx) if self._ok() else True
