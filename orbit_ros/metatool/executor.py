"""The deterministic meta-tool executor (section 4.8).

MetaToolAgent(session, registry, llm, memory, config).run(instruction) -> RunResult:
  1. plan with the schema, validate, self-refine (<=3 rounds) BEFORE any motion;
  2. refusal (feasible:false) -> say the reason, 0 motions;
  3. execute in topological order, NO LLM between steps; per step: resolve ${refs} -> precondition -> loop guard ->
     session.robot_call (unchanged validation+guard+motion) -> verifier;
  4. on failure: skip | abort | repair (plan the remaining work only, validate with done_ids, splice);
  5. reflex: after a fall the only non-plan action is the real stand_up (Phase 1);
  6. steps whose deps failed are skipped;
  7. success = every non-skipped step done AND verified.

Nothing here bypasses session.robot_call. The guard never sees memory or LLM output."""
from __future__ import annotations
import json, time
from dataclasses import dataclass, field

from . import spec as mspec
from . import prompts
from .plan import Plan, resolve_refs
from .validator import validate
from .guard import LoopGuard, Budget
from .trace import Trace


@dataclass
class Config:
    max_refine: int = 3
    budget: Budget = field(default_factory=Budget)
    use_memory: bool = True
    allow_repair: bool = True
    # Phase 17 switches (all preserve the safety pipeline; none bypass robot_call/guard):
    check_complete: bool = False      # feature 1: drive bounded refinement on INCOMPLETE goal targets
    planning_mode: str = "single_constrained"   # feature 1: single_constrained | two_pass | single_free
    use_tool_retrieval: bool = False  # feature 8: plan with a retrieved tool subset (validator keeps full catalogue)
    retrieval_k: int = 12
    use_safety_gate: bool = False     # feature 3: deterministic AUTHORIZE/DEFER/REJECT gate + task contract
    vote_k: int = 1                   # feature 5: >1 enables consistency voting over normalized goals


@dataclass
class RunResult:
    success: bool
    status: str                      # done | refused | failed | aborted | invalid_plan
    plan: dict
    steps: list
    said: list
    metrics: dict


class MetaToolAgent:
    def __init__(self, session, registry, llm=None, memory=None, config=None, trace_path=None, hud=None):
        self.s = session
        self.registry = registry
        self.llm = llm
        self.memory = memory if memory is not None else getattr(session, "memory", None)
        self.cfg = config or Config()
        self.tr = Trace(trace_path)
        self.guard = LoopGuard(budget=self.cfg.budget)
        self.said = []
        self.hud = hud or (lambda **k: None)
        self._repairs = 0
        self._contract = None

    # ---- public ----
    def run(self, instruction) -> RunResult:
        self.tr.event("start", instruction=instruction)
        plan = self._plan(instruction)
        if plan is None:
            return self._finish(Plan(goal=instruction, feasible=False, refusal_reason="no valid plan could be produced"),
                                "invalid_plan", False)
        if not plan.feasible:
            self._say(plan.refusal_reason or "I cannot do that safely.")
            self.tr.event("rejected", reason=plan.refusal_reason)
            return self._finish(plan, "refused", False)
        # Feature 3: deterministic safety gate. Final AUTHORIZE/DEFER/REJECT is by code rules, not the LLM.
        if self.cfg.use_safety_gate:
            gate = self._safety_gate(instruction, plan)
            self.tr.event("safety_gate", **gate.as_dict())
            if gate.decision == "REJECT":
                self._say(gate.reason or "I cannot do that safely.")
                plan.feasible = False; plan.refusal_reason = gate.reason
                return self._finish(plan, "refused", False)
            if gate.decision == "DEFER":
                self._say(gate.question or gate.reason or "I need more information before I can act.")
                plan.feasible = False; plan.refusal_reason = "deferred: " + (gate.reason or "")
                return self._finish(plan, "deferred", False)
            self._contract = gate.contract      # monitored during execution; never overrides the guard
        return self._execute(plan, instruction)

    def _safety_gate(self, instruction, plan):
        from . import safety_gate as sg
        facts = mspec.facts(self.s)
        known = [f[len("known("):-1] for f in facts if f.startswith("known(")]
        # person sensing is NOT wired into the live metatool path -> report False (gate will DEFER if a person is implicated)
        return sg.evaluate(instruction, goal_struct=plan.goal_struct, plan=plan, facts=facts,
                           known_names=known, person_sensing=False)

    # ---- planning + refine (no motion) ----
    def _plan(self, instruction):
        from .schema import plan_json_schema
        facts = mspec.facts(self.s)
        # Feature 8: tool retrieval. The planner sees a retrieved SUBSET; the validator always uses the full
        # registry (self.registry). On a retrieval miss we retry ONCE with the full catalogue and record it.
        if self.cfg.use_tool_retrieval:
            from . import tool_retrieval as tret
            names = tret.retrieve(instruction, self.registry, k=self.cfg.retrieval_k)
            plan_registry = tret.subset_registry(self.registry, names)
            self.tr.event("retrieval", k=self.cfg.retrieval_k, chosen=names)
            plan = self._plan_voted(instruction, facts, plan_registry)
            if plan is not None and plan.feasible and tret.retrieval_miss(plan, names):
                self.tr.event("retrieval", miss=True, note="plan needs a tool outside the subset; retrying full catalogue")
                plan = self._plan_voted(instruction, facts, self.registry)
            return plan
        return self._plan_voted(instruction, facts, self.registry)

    def _plan_voted(self, instruction, facts, plan_registry):
        """Feature 5: when vote_k>1, generate K candidate plans and carry forward the winning (most-agreed) one.
        The winner still passes full validation downstream. vote_k<=1 is the normal single-plan path."""
        k = max(1, int(self.cfg.vote_k))
        if k <= 1:
            return self._plan_once(instruction, facts, plan_registry)
        from . import voting
        cands = []
        for i in range(k):
            p = self._plan_once(instruction, facts, plan_registry)
            if p is not None:
                cands.append(p)
        if not cands:
            return None
        winner, tally = voting.vote(cands)
        self.tr.event("vote", k=k, candidates=len(cands), tally=tally, agreement=voting.agreement(cands))
        return winner

    def _plan_once(self, instruction, facts, plan_registry):
        """Generate + bounded-refine a plan. plan_registry is what the PLANNER sees; validation uses self.registry."""
        from .schema import plan_json_schema
        schema = plan_json_schema(plan_registry, strict=True)
        state = self._state()
        brief = self._memory_brief(instruction) if self.cfg.use_memory else ""
        sys = prompts.system_prompt(plan_registry)
        usr = prompts.user_prompt(instruction, state, brief, facts)
        messages = [{"role": "system", "content": sys}, {"role": "user", "content": usr}]
        if self.llm is None or not self.llm.available():
            self.tr.event("llm", error="no provider available"); return None
        d, raw, mode = self.llm.plan(messages, schema)
        self.tr.event("llm", mode=mode, got_json=d is not None)
        if d is None: return None
        plan = Plan.from_dict(d)
        for rnd in range(self.cfg.max_refine):
            issues = validate(plan, self.registry, facts, check_complete=self.cfg.check_complete)
            self.tr.event("validate", round=rnd, issues=[i.as_dict() for i in issues])
            if not issues: break
            messages.append({"role": "assistant", "content": raw})
            messages.append({"role": "user", "content": prompts.refine_prompt(issues)})
            d, raw, mode = self.llm.plan(messages, schema)
            self.tr.event("llm", mode=mode, got_json=d is not None, refine=rnd + 1)
            if d is None: return None
            plan = Plan.from_dict(d)
        issues = validate(plan, self.registry, facts, check_complete=self.cfg.check_complete)
        if issues:
            self.tr.event("rejected", reason="plan still invalid after refine", issues=[i.as_dict() for i in issues])
            # a still-invalid plan is refused rather than executed
            plan.feasible = False
            plan.refusal_reason = "could not form a valid plan: " + "; ".join(i.msg for i in issues[:3])
        self.tr.event("plan", plan=plan.to_dict())
        return plan

    # ---- execution ----
    def _execute(self, plan, instruction):
        results = {}                       # step_id -> result dict (for ${refs})
        try:
            order = plan.topo_order()
        except ValueError as e:
            self.tr.event("rejected", reason=str(e)); return self._finish(plan, "invalid_plan", False)
        i = 0
        while i < len(order):
            s = order[i]
            # skip if a dependency failed/skipped
            if any((plan.by_id(a) and plan.by_id(a).status in ("failed", "skipped")) for a in s.after):
                s.status = "skipped"; self.tr.event("tool", step=s.id, tool=s.tool, status="skipped", reason="dependency failed")
                i += 1; continue
            over = self.guard.over_budget(self.tr.counts["llm"], self._repairs, self.tr.metrics()["wall_s"])
            if over:
                self.tr.event("rejected", reason=over); return self._finish(plan, "aborted", False)
            outcome = self._run_step(s, plan, results)
            if outcome == "ok":
                i += 1; continue
            # failure handling
            if s.on_fail == "skip":
                s.status = "skipped"; i += 1; continue
            if s.on_fail == "abort":
                return self._finish(plan, "aborted", False)
            # repair
            if self.cfg.allow_repair and self.llm is not None and self.llm.available() and self._repairs < self.cfg.budget.repairs:
                spliced = self._repair(plan, s, instruction)
                if spliced is not None:
                    plan = spliced
                    try: order = plan.topo_order()
                    except ValueError: return self._finish(plan, "aborted", False)
                    # continue from the first not-done step
                    i = next((k for k, st in enumerate(order) if st.status not in ("done", "skipped")), len(order))
                    continue
            s.status = "failed"
            return self._finish(plan, "failed", False)
        # success = every non-skipped step done and verified
        ok = all(st.status in ("done", "skipped") for st in plan.steps) and any(st.status == "done" for st in plan.steps)
        return self._finish(plan, "done" if ok else "failed", ok)

    def _run_step(self, s, plan, results):
        s.status = "running"; s.attempts += 1
        self.hud(step=f"{s.id}: {s.tool}")
        # resolve ${refs} from ancestors
        anc = self._ancestors(plan, s.id)
        args, ref_errs = resolve_refs(s.args, results, anc)
        if ref_errs:
            s.status = "failed"; s.result = {"status": "error", "message": "; ".join(ref_errs)}
            self.tr.event("tool", step=s.id, tool=s.tool, status="error", message="; ".join(ref_errs)); return "fail"
        # Feature 11 (fail-closed): the executor may only invoke a REGISTERED tool from the approved plan.
        reg = self.registry if isinstance(self.registry, dict) else {sp.name: sp for sp in self.registry}
        if s.tool not in reg:
            s.status = "failed"; s.result = {"status": "error", "message": f"unregistered tool {s.tool!r} blocked"}
            self.tr.event("tool", step=s.id, tool=s.tool, status="error", message="unregistered tool blocked"); return "fail"
        # live precondition check against current facts
        facts = mspec.facts(self.s); spec = mspec.spec_for(s.tool)
        for req in spec.requires:
            filled = mspec.fill(req, args)
            if filled is None: continue
            neg = filled.startswith("!"); pred = filled[1:] if neg else filled
            holds = pred in facts
            if (neg and holds) or (not neg and not holds):
                s.status = "failed"; s.result = {"status": "rejected", "message": f"precondition {filled} not met"}
                self.tr.event("verify", step=s.id, ok=False, detail=f"precondition {filled}")
                return "fail"
        # loop guard
        allowed, why = self.guard.check(s.tool, args, self.s)
        if not allowed:
            self.tr.event("guard", step=s.id, tool=s.tool, blocked=True, reason=why)
            s.status = "failed"; s.result = {"status": "blocked", "message": why}; return "fail"
        # execute through the unchanged robot path
        r = self.s.robot_call(s.tool, args)
        self.guard.record(s.tool, args, self.s)
        s.result = r; results[s.id] = (r.get("data") or {}) if isinstance(r.get("data"), dict) else {"data": r.get("data")}
        results[s.id].setdefault("status", r.get("status"))
        repeated = s.attempts > 1
        self.tr.event("tool", step=s.id, tool=s.tool, args=args, status=r.get("status"),
                      message=str(r.get("message", ""))[:160], repeated=repeated)
        # fall reflex (the only non-plan action)
        try: fallen = self.s._pose()["fallen"]
        except Exception: fallen = False
        if fallen:
            self.tr.event("reflex", kind="stand_up", step=s.id)
            self.s.robot_call("stand_up", {})
        # verify from the session's true observations
        ok, detail = spec.verify(args, r, self.s)
        self.tr.event("verify", step=s.id, ok=bool(ok), detail=detail)
        if ok: s.status = "done"; return "ok"
        s.status = "failed"; return "fail"

    def _repair(self, plan, failed, instruction):
        self._repairs += 1
        done_ids = [st.id for st in plan.steps if st.status == "done"]
        cause = self._attribute_cause(failed.result)
        # Feature 7: structured root-cause attribution over the whole trace (earliest contributing step).
        try:
            from . import root_cause as rc
            rcause = rc.attribute(plan.steps, failed_step_id=failed.id)
            self.tr.event("root_cause", **rcause.as_dict())
        except Exception:
            rcause = None
        self.tr.event("repair", step=failed.id, cause=cause, n=self._repairs,
                      root_cause=(rcause.label if rcause else None),
                      earliest_step=(rcause.earliest_step if rcause else None))
        from .schema import plan_json_schema
        schema = plan_json_schema(self.registry, strict=True)
        facts = mspec.facts(self.s)
        msg = prompts.repair_prompt(plan.pseudo_code(), {"id": failed.id, "tool": failed.tool, "args": failed.args},
                                    expected="step verified", observed=failed.result, cause=cause,
                                    state=self._state(), memory_brief=self._memory_brief(instruction) if self.cfg.use_memory else "",
                                    facts=facts, done_ids=done_ids)
        messages = [{"role": "system", "content": prompts.system_prompt(self.registry)}, {"role": "user", "content": msg}]
        d, raw, mode = self.llm.plan(messages, schema)
        self.tr.event("llm", mode=mode, got_json=d is not None, repair=self._repairs)
        if d is None: return None
        new = Plan.from_dict(d)
        issues = validate(new, self.registry, facts, done_ids=done_ids)
        self.tr.event("validate", repair=self._repairs, issues=[i.as_dict() for i in issues])
        if issues: return None
        # splice: keep done steps, append the new ones with fresh ids, bump revision
        done_steps = [st for st in plan.steps if st.status == "done"]
        existing = {st.id for st in plan.steps}
        for st in new.steps:
            if st.id in existing: st.id = f"{st.id}_r{self._repairs}"
        merged = Plan(goal=plan.goal, goal_struct=plan.goal_struct, feasible=True, steps=done_steps + new.steps,
                      done_when=plan.done_when, revision=plan.revision + 1)
        self.tr.event("plan", plan=merged.to_dict(), spliced=True)
        return merged

    @staticmethod
    def _attribute_cause(result):
        st = (result or {}).get("status"); msg = str((result or {}).get("message", "")).lower()
        if "fell" in msg or "fallen" in msg: return "fallen"
        if st == "vetoed" or "guard" in msg or "no-go" in msg or "too close" in msg: return "guard veto / obstacle"
        if st == "blocked": return "loop"
        if "no safe path" in msg or "not found" in msg or "not visible" in msg or "outside the house" in msg: return "not reachable / not found"
        if "precondition" in msg or "stopped" in msg: return "precondition"
        if "timeout" in msg or "no progress" in msg or "drift" in msg: return "timeout / drift"
        return "unknown"

    # ---- helpers ----
    def _ancestors(self, plan, sid):
        anc, stack = set(), list((plan.by_id(sid).after if plan.by_id(sid) else []))
        while stack:
            a = stack.pop()
            if a in anc: continue
            anc.add(a); st = plan.by_id(a)
            if st: stack += st.after
        return anc

    def _state(self):
        try: return self.s._pose()
        except Exception: return {}

    def _memory_brief(self, instruction):
        if self.memory is None: return ""
        try:
            tb = self.s.task_memory(instruction=instruction) if hasattr(self.s, "task_memory") else {}
            return tb.get("brief", "") if isinstance(tb, dict) else ""
        except Exception:
            return ""

    def _say(self, text):
        self.said.append(text)
        # prefer a real say tool if present, else just record it
        if "say" in (self.registry if isinstance(self.registry, dict) else {}):
            try: self.s.robot_call("say", {"text": text})
            except Exception: pass

    def _finish(self, plan, status, success):
        try:
            if self.memory is not None and hasattr(self.s, "record_episode"):
                self.s.record_episode(plan.goal or "", {"status": "ok" if success else status}, provenance={"source": "metatool"})
        except Exception: pass
        m = self.tr.metrics({"status": status, "revision": plan.revision})
        if self.llm is not None: m.update(self.llm.metrics())
        self.tr.event("done", status=status, success=success, metrics=m)
        self.tr.close()
        return RunResult(success=success, status=status, plan=plan.to_dict(),
                         steps=[{"id": s.id, "tool": s.tool, "status": s.status,
                                 "result_status": (s.result or {}).get("status")} for s in plan.steps],
                         said=list(self.said), metrics=m)
