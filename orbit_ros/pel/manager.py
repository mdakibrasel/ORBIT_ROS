"""ORBIT-PEL execution manager (v0.14 milestone M4): validated, guarded, single-step, observed and verified execution with bounded
retry and re-planning.

Path of every model-generated robot call:
    model reply -> schema.parse_plan -> registry.validate_plan_calls (plan level) -> per step: registry.validate_call (with
    preconditions) -> SafetyGuard.check -> duplicate check -> robot call (through the runtime's `do`) -> fresh observation ->
    outcome verification -> continue | retry (only if provably safe) | re-plan (bounded) | clarify | stop.
The manager is the only caller of `do` in a PEL configuration, so no model output can reach the robot without passing the checks of
the components that are switched on. Components can be switched off one at a time for ablation (`PelFlags`).

The manager is driven by `agent2d.run_episode` for configurations whose mode is "pel" (PEL, PEL-registry, PEL-guard, PEL-verify,
PEL-replan). It uses the same planner interface (`plan`, `repair`, `assess`), the same observation context and the same robot-call
bookkeeping as the existing configurations, so their rows and metrics stay comparable.

v0.14.1 (fixes from the v0.14.0 code review, CODE_REVIEW_v0.14.0.md):
* A plan that still fails plan-level validation after MAX_PLAN_FIXES repairs (or when re-planning is off / the budget is spent) is
  NOT executed: the instruction ends with terminal code safe_stop ("plan invalid after fixes"). [review M2]
* v1 step fields are enforced: `pre` is checked on a fresh observation before the step (unmet -> nothing is sent, the step fails);
  `on_failure: "stop"` ends the instruction (safe_stop) on any failure of that step; "replan", "retry" and "ask" all lead to a bounded
  re-plan (a retry happens only when it is provably safe, whatever the plan says). `expect.cleaned_cell` / `expect.reported` and
  `completion.cleaned_min` / `completion.reported` / `completion.visited` are verified. `constraints.max_steps` is a plan-level check
  (schema.py); `constraints.stay_in_room: false` is ignored because the guard always keeps the robot in the room. [review M3]
* Step verification predicts macro paths with the declared no-go cells, the same obstacle set the macro is expanded with. [review M4]
* Resume: the no-go cells of EVERY earlier revision are restored (stored in the exec state and rebuilt from all revisions), and an
  attempt that failed before the crash is handled as a failure (re-plan or stop) instead of being sent again. [review H2, M1]
* Terminal codes: unsupported -> "unsupported", an abstain step -> "abstained", a clarification that cannot be asked or answered ->
  "needs_clarification". [review L1]
* A clarification request goes through the registry (`asks_left` precondition) and is recorded as an attempt. [review L4]
Resume is a library feature (run_phase(..., resume=state)); no command-line runner resumes a half-finished instruction. [review M5]
"""
from __future__ import annotations
import os, time
from dataclasses import dataclass
from .schema import parse_plan, errors as schema_errors
from .registry import REGISTRY, validate_call, validate_plan_calls
from .guard import SafetyGuard
from .state import ExecutionState, fingerprint, stale_reason, resume_blocker, no_go_of, accumulated_no_go, pending_failure
from ..cognition import dry_run
from ..contract_api import status_of

# Per-instruction budgets. MAX_REPAIRS / MAX_REPLANS are the same as the S5 runtime (agent2d): up to 5 re-plans after a failure
# (rejection, veto, failed call, verification mismatch, duplicate, no progress, stale plan) and up to 6 after the plan ran out without
# completing the instruction, so PEL is not given more model calls than S5.
MAX_REPAIRS, MAX_REPLANS, MAX_PLAN_FIXES, MAX_RETRIES, NO_PROGRESS_LIMIT = 5, 6, 3, 2, 4

@dataclass(frozen=True)
class PelFlags:
    registry: bool = True     # typed tool validation (plan level and per call, with preconditions)
    guard: bool = True        # safety guard (room bounds, believed obstacles, declared no-go cells, per-call distance limit)
    verify: bool = True       # outcome verification after every step and at plan completion
    replan: bool = True       # bounded re-planning after a rejection, veto, failure or verification mismatch

    @classmethod
    def without(cls, component: str) -> "PelFlags":
        return cls(**{k: k != component for k in ("registry", "guard", "verify", "replan")})

COUNTERS = ("pel_plan_errors", "pel_rejections", "pel_vetoes", "pel_verify_failures", "pel_retries", "pel_duplicates_blocked",
            "pel_no_progress", "pel_stale_plans", "pel_replans", "pel_unrecovered", "pel_clarifications", "pel_pre_failures")

def _cells_between(a, b):
    """Cells on the straight (axis-aligned) segment from cell a to cell b, both included; just the two ends otherwise."""
    if a[0] == b[0]: return [(a[0], y) for y in range(min(a[1], b[1]), max(a[1], b[1]) + 1)]
    if a[1] == b[1]: return [(x, a[1]) for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1)]
    return [tuple(a), tuple(b)]

class ExecutionManager:
    def __init__(self, flags: PelFlags, backend, observe, do, ev, budget_ok, reg_ctx, stopped, start_pose, episode="", config="",
                 state_dir=None, guard=None, rep=0):
        self.f, self.backend, self.observe, self.do, self.ev = flags, backend, observe, do, ev
        self.budget_ok, self.reg_ctx, self.stopped, self.start_pose = budget_ok, reg_ctx, stopped, start_pose
        self.episode, self.config, self.state_dir, self.rep = episode, config, state_dir, rep
        self.guard = guard or SafetyGuard()
        self.c = {k: 0 for k in COUNTERS}
        self.state = None
        self.no_go, self.visited, self.cleaned0 = set(), set(), 0

    # ------------------------------------------------------------------------------------------------------------- helpers
    def _fp(self, ctx):
        return fingerprint(ctx["pose"], ctx.get("raw") or [], ctx.get("obstacles_known") or [], ctx.get("cleaned_known") or [], self.stopped())

    def _save(self):
        if self.state_dir and self.state is not None:
            name = f"{self.episode}#{self.config}#r{self.rep}#p{self.state.phase}".replace("|", "_").replace("/", "_")     # rep: review L6
            self.state.save(os.path.join(self.state_dir, name + ".json"))

    def _stop(self, code, reason):
        """End the instruction with an explicit terminal code (kept by run_phase) and save."""
        self.ev("pel_stop", code=code, reason=reason)
        if self.state is not None and self.state.terminal is None: self.state.terminate(code, reason); self._save()
        return None

    def _fail(self, ph, step, reason):
        """A step failed (rejected, vetoed, precondition unmet, duplicate, call failed, verification mismatch): stop if the step says
        on_failure "stop", otherwise a bounded re-plan."""
        if step is not None and step.on_failure == "stop":
            self.c["pel_unrecovered"] += 1
            return self._stop("safe_stop", f"step {step.id} failed and its on_failure is stop: {reason}")
        return self._replan(ph, reason)

    def _cleaned(self, ctx): return len({tuple(c) for c in ctx.get("cleaned_known") or []})

    def _track(self, pre_pose, ctx):
        """Cells on the robot's REPORTED path (consecutive reported poses, joined along the axis); persisted for completion.visited."""
        a, b = (pre_pose["x"], pre_pose["y"]), (ctx["pose"]["x"], ctx["pose"]["y"])
        for c in _cells_between(a, b): self.visited.add(c)
        if self.state is not None: self.state.visited_cells = sorted(map(list, self.visited))

    def _pre_unmet(self, step, ctx):
        """The step's own precondition (`pre`, v1 plans) on the current observation; None if it holds or there is none."""
        pre = step.pre if isinstance(step.pre, dict) else {}
        if "robot_stopped" in pre and bool(self.stopped()) != bool(pre["robot_stopped"]):
            return f"step {step.id}: precondition robot_stopped={pre['robot_stopped']} does not hold"
        for k, v in (pre.get("pose") or {}).items():
            if k in ctx["pose"] and ctx["pose"][k] != v: return f"step {step.id}: precondition {k}={v} does not hold ({k}={ctx['pose'][k]})"
        if pre.get("sees"):
            if ctx.get("view_stale"): return f"step {step.id}: precondition sees {pre['sees']} cannot be checked (stale camera frame)"
            if pre["sees"] not in {d.get("label") for d in ctx.get("raw") or []}: return f"step {step.id}: precondition sees {pre['sees']} does not hold"
        if pre.get("localised") is True and ctx.get("pose_stale"): return f"step {step.id}: precondition localised does not hold (pose not localised)"
        return None

    def _accept(self, ph, raw, ctx, reason, fixes=0):
        """Model reply -> stored plan revision (or None when the phase must end). Plan-level validation errors go back to the planner
        (bounded); a clarification request becomes an `ask`, an unsupported task an `abstain`."""
        self.last_ctx = ctx
        plan, issues = parse_plan(raw)
        errs = schema_errors(issues)
        if self.f.registry: errs += [i for i in validate_plan_calls(plan) if i.severity == "error"]
        if errs:
            self.c["pel_plan_errors"] += 1; self.ev("pel_plan_invalid", errors=[str(e) for e in errs[:5]])
            if self.f.replan and fixes < MAX_PLAN_FIXES and self.budget_ok():
                ctx2 = self.observe(self.phase_start, False)
                raw2 = self.backend.repair(ph, ctx2, "plan validation errors: " + "; ".join(e.message for e in errs[:3]))
                return self._accept(ph, raw2, ctx2, reason, fixes + 1)
            self.c["pel_unrecovered"] += 1                 # v0.14.1: an invalid plan is never executed (review M2)
            return self._stop("safe_stop", f"plan invalid after {fixes} fix(es): " + "; ".join(str(e) for e in errs[:3]))
        if plan.status == "needs_clarification":
            self.c["pel_clarifications"] += 1
            call = {"tool": "ask", "args": {"question": plan.clarification or ""}}
            if self.f.registry:                            # v0.14.1: the ask goes through the registry (asks_left), review L4
                bad = [i for i in validate_call(call, self.reg_ctx()) if i.severity == "error"]
                if bad:
                    self.c["pel_rejections"] += 1; why = "; ".join(i.message for i in bad)
                    self.state.record_attempt(0, call, "rejected", why, 0.0, None, robot_call=False)
                    return self._stop("needs_clarification", "clarification needed but cannot be asked: " + why)
            r = self.do(call, ctx)
            self.state.record_attempt(0, call, "ok" if r.get("ok") else "invalid", r.get("message", ""), 0.0, None, robot_call=False); self._save()
            if not r.get("ok") or not self.f.replan: return self._stop("needs_clarification", "clarification not answered: " + r.get("message", ""))
            return self._replan(ph, "clarification received: " + r.get("message", ""))
        if plan.status == "unsupported":
            call = {"tool": "abstain", "args": {"reason": plan.unsupported_reason or ""}}
            r = self.do(call, ctx); self.state.record_attempt(0, call, "ok" if r.get("ok") else "invalid", r.get("message", ""), 0.0, None, robot_call=False)
            return self._stop("unsupported", plan.unsupported_reason or "unsupported")
        self.no_go |= no_go_of(plan.constraints)
        self.state.no_go_cells = sorted(map(list, self.no_go))     # v0.14.1: the union over all revisions survives a resume (review H2)
        self.state.start_plan(plan.to_dict(), reason, self._fp(ctx)); self.retries = 0; self._save()
        return plan

    def _replan(self, ph, reason, exhausted=False):
        """exhausted=True: the plan ran out without completing the instruction (MAX_REPLANS); otherwise a failure (MAX_REPAIRS)."""
        used, cap = (self.replans, MAX_REPLANS) if exhausted else (self.repairs, MAX_REPAIRS)
        if not self.f.replan or used >= cap or not self.budget_ok():
            self.c["pel_unrecovered"] += 1; self.ev("pel_unrecovered", reason=reason); return None
        if exhausted: self.replans += 1
        else: self.repairs += 1
        self.c["pel_replans"] += 1; self.ev("pel_replan", reason=reason)
        ctx = self.observe(self.phase_start, False)
        raw = self.backend.plan(ph, ctx) if exhausted else self.backend.repair(ph, ctx, reason)     # as S5: a fresh plan after exhaustion
        return self._accept(ph, raw, ctx, reason)

    def _verify_step(self, step, call, pre_ctx, ctx):
        """Expected vs observed after an executed step. Uses the step's own `expect` (v1 plans) and the dry-run prediction from the pose
        before the call. Object expectations are skipped when the camera frame was stale (no claim from an old image)."""
        exp = step.expect or {}
        pose = ctx["pose"]
        for k, v in (exp.get("pose") or {}).items():
            if k in pose and pose[k] != v: return f"step {step.id}: {k}={pose[k]} but the plan expected {v}"
        if "cleaned_cell" in exp and isinstance(exp["cleaned_cell"], bool) and (self._cleaned(ctx) > self._cleaned(pre_ctx)) != exp["cleaned_cell"]:
            return f"step {step.id}: expected cleaned_cell={exp['cleaned_cell']}"
        if "reported" in exp and isinstance(exp["reported"], bool) and (len(ctx.get("reported") or []) > len(pre_ctx.get("reported") or [])) != exp["reported"]:
            return f"step {step.id}: expected reported={exp['reported']}"
        if call["tool"] in ("move_forward", "turn", "goto_cell", "cover_region"):
            # the same obstacle set `do` expands macros with: believed obstacles + declared no-go cells (v0.14.1, review M4)
            obst = sorted({tuple(o) for o in pre_ctx.get("obstacles_known") or []} | self.no_go)
            pred, _, perr = dry_run([call], pre_ctx["pose"], obst, self.start_pose)
            if not perr and (pred["x"], pred["y"], pred["yaw_deg"]) != (pose["x"], pose["y"], pose["yaw_deg"]):
                return f"step {step.id}: pose ({pose['x']},{pose['y']},{pose['yaw_deg']}) differs from the prediction ({pred['x']},{pred['y']},{pred['yaw_deg']})"
        if not ctx.get("view_stale"):
            labels = {d.get("label") for d in ctx.get("raw") or []}
            if exp.get("sees") and exp["sees"] not in labels: return f"step {step.id}: expected to see {exp['sees']}"
            if exp.get("not_sees") and exp["not_sees"] in labels: return f"step {step.id}: did not expect to see {exp['not_sees']}"
        return None

    def _completion_mismatch(self, plan, ctx):
        comp = plan.completion if isinstance(plan.completion, dict) else {}
        for k, v in (comp.get("pose") or {}).items():
            if k in ctx["pose"] and ctx["pose"][k] != v: return f"plan finished at {k}={ctx['pose'][k]}, completion expects {v}"
        cm = comp.get("cleaned_min")                       # v0.14.1 (review M3): cells cleaned since this instruction started
        if isinstance(cm, int) and not isinstance(cm, bool) and self._cleaned(ctx) - self.cleaned0 < cm:
            return f"plan finished with {self._cleaned(ctx) - self.cleaned0} cell(s) cleaned in this instruction, completion expects at least {cm}"
        if comp.get("reported") is True and not ctx.get("reported"): return "plan finished without a report, completion expects one"
        if isinstance(comp.get("visited"), list):
            missing = [list(c) for c in comp["visited"] if isinstance(c, (list, tuple)) and tuple(c) not in self.visited]
            if missing: return f"plan finished without visiting {missing[:5]}, completion expects them"
        return None

    # ------------------------------------------------------------------------------------------------------------- one phase
    def run_phase(self, ph, phase_start, phase_index: int, resume: ExecutionState | None = None):
        self._run_phase(ph, phase_start, phase_index, resume)
        if self.state is not None and self.state.terminal is None:          # every exit path leaves a terminal code in the saved state
            self.state.terminate("budget_exhausted" if not self.budget_ok() else "safe_stop", "instruction ended without completion"); self._save()

    def _run_phase(self, ph, phase_start, phase_index, resume):
        """Run one instruction. resume: a saved state of this instruction (ExecutionState.load); execution continues at its cursor only
        if a fresh observation matches the recorded one (state.resume_blocker); otherwise the stored plan is stale and is re-planned."""
        self.phase_start, self.replans, self.repairs, self.retries, self.no_go = phase_start, 0, 0, 0, set()
        ctx = self.observe(phase_start, resume is None)
        if resume is not None:
            if resume.terminal: self.state = resume; return                      # a finished instruction is never re-entered
            self.state = resume; fp = self._fp(ctx)
            # v0.14.1 (review H2): every no-go cell any earlier revision declared stays guarded after a resume
            self.no_go = accumulated_no_go(resume); resume.no_go_cells = sorted(map(list, self.no_go))
            self.cleaned0 = resume.cleaned_at_start if isinstance(resume.cleaned_at_start, int) else self._cleaned(ctx)
            self.visited = {tuple(c) for c in resume.visited_cells or []} | {(ctx["pose"]["x"], ctx["pose"]["y"])}
            why = resume_blocker(resume, fp) if resume.plan_revisions else "no stored plan"
            pending = None if why else pending_failure(resume)
            self.ev("pel_resume", cursor=resume.cursor, revision=resume.revision, blocked=why or "", pending_failure=pending or "")
            if not self.budget_ok(): return
            if why:
                self.c["pel_stale_plans"] += 1; resume.fingerprint = fp
                plan = self._replan(ph, "stale plan on resume: " + why)
            elif pending:                                   # v0.14.1 (review M1): handle the failure, never re-send the failed step
                plan, _ = parse_plan(resume.plan); self.last_ctx = ctx
                plan = self._fail(ph, plan.steps[resume.cursor] if resume.cursor < len(plan.steps) else None, "resumed after an unhandled failure: " + pending)
            else:
                plan, _ = parse_plan(resume.plan); self.last_ctx = ctx
        else:
            self.state = ExecutionState(episode=self.episode, config=self.config, phase=phase_index)
            self.state.fingerprint = self._fp(ctx)          # the state the first call is issued from (duplicate and progress checks)
            self.cleaned0 = self.state.cleaned_at_start = self._cleaned(ctx)
            self.visited = {(ctx["pose"]["x"], ctx["pose"]["y"])}; self.state.visited_cells = sorted(map(list, self.visited))
            if not self.budget_ok(): return
            plan = self._accept(ph, self.backend.plan(ph, ctx), ctx, "initial")
        while plan is not None and self.budget_ok():
            st = self.state
            if st.cursor == 0 and st.plan_revisions:          # a plan is used only from the state it was made for
                why = stale_reason(st.plan_revisions[-1]["made_for"], self._fp(ctx))
                if why:
                    self.c["pel_stale_plans"] += 1; self.ev("pel_stale_plan", reason=why)
                    plan = self._replan(ph, "stale plan: " + why); ctx = self.last_ctx; continue
            if st.cursor >= len(plan.steps) or plan.steps[st.cursor].tool == "done":
                why = self._completion_mismatch(plan, ctx) if self.f.verify else None
                if why: self.c["pel_verify_failures"] += 1
                elif self.backend.assess(ph, ctx): st.terminate("done"); self._save(); return
                plan = self._replan(ph, why or "plan finished, instruction not complete", exhausted=not why); ctx = self.last_ctx; continue
            step = plan.steps[st.cursor]; call = step.call()
            if step.tool == "replan":
                plan = self._replan(ph, "the plan asks for a re-plan"); ctx = self.last_ctx; continue
            if self.f.verify:                              # v0.14.1 (review M3): the step's own precondition, on the current observation
                why = self._pre_unmet(step, ctx)
                if why:
                    self.c["pel_pre_failures"] += 1; st.record_attempt(step.id, call, "precondition_failed", why, 0.0, None, robot_call=False)
                    self.ev("pel_precondition_failed", call=call, reason=why); self._save()
                    plan = self._fail(ph, step, why); ctx = self.last_ctx; continue
            if self.f.registry:
                bad = [i for i in validate_call(call, self.reg_ctx()) if i.severity == "error"]
                if bad:
                    self.c["pel_rejections"] += 1; st.record_attempt(step.id, call, "rejected", "; ".join(i.message for i in bad), 0.0, None, robot_call=False)
                    self.ev("pel_rejected", call=call, issues=[str(i) for i in bad])
                    plan = self._fail(ph, step, f"call {call} rejected: " + "; ".join(i.message for i in bad)); ctx = self.last_ctx; continue
            if self.f.guard:
                why = self.guard.check(call, ctx["pose"], ctx.get("obstacles_known") or [], self.no_go, self.start_pose, ctx.get("cleaned_known") or frozenset())
                if why:
                    self.c["pel_vetoes"] += 1; st.record_attempt(step.id, call, "vetoed", why, 0.0, None, robot_call=False)
                    self.ev("pel_vetoed", call=call, reason=why)
                    plan = self._fail(ph, step, f"safety guard vetoed {call}: {why}"); ctx = self.last_ctx; continue
            spec = REGISTRY.get(call["tool"])
            if st.is_duplicate(call) and not (spec and spec.retry_safe) and self.retries == 0:
                self.c["pel_duplicates_blocked"] += 1; self.ev("pel_duplicate_blocked", call=call)
                plan = self._fail(ph, step, f"{call} was already executed from this state without effect"); ctx = self.last_ctx; continue
            pre_ctx = ctx
            t0 = time.perf_counter()
            do_ctx = dict(ctx, obstacles_known=sorted({tuple(o) for o in ctx.get("obstacles_known") or []} | self.no_go)) if self.no_go else ctx
            r = self.do(call, do_ctx); dt = time.perf_counter() - t0
            ctx = self.observe(phase_start, False)
            status = status_of(r) if call["tool"] not in ("ask", "abstain", "remember") else ("ok" if r.get("ok") else "invalid")
            self._track(pre_ctx["pose"], ctx)
            st.record_attempt(step.id, call, status, r.get("message", ""), dt, self._fp(ctx)); self._save()
            if call["tool"] == "abstain" and r.get("ok"):  # v0.14.1 (review L1): an abstention ends with its own terminal code
                st.advance(); return self._stop("abstained", (call.get("args") or {}).get("reason") or "abstained")
            if not self.budget_ok():                   # interrupted here: an executed step is marked done so a resume never repeats it
                if status == "ok": st.advance(); self._save()
                return
            if call["tool"] == "ask" and r.get("ok"):
                st.advance(); plan = self._replan(ph, "clarification received: " + r.get("message", "")); ctx = self.last_ctx; continue
            failure = None
            if status != "ok":
                unchanged = (pre_ctx["pose"]["x"], pre_ctx["pose"]["y"], pre_ctx["pose"]["yaw_deg"]) == (ctx["pose"]["x"], ctx["pose"]["y"], ctx["pose"]["yaw_deg"])
                provably_safe = (spec is not None and spec.retry_safe) or (status == "fault" and unchanged)
                if provably_safe and self.retries < MAX_RETRIES:
                    self.retries += 1; self.c["pel_retries"] += 1; self.ev("pel_retry", call=call, status=status); continue
                failure = f"{call['tool']} {call.get('args')} failed ({status}): {r.get('message', '')}"
            elif self.f.verify:
                failure = self._verify_step(step, call, pre_ctx, ctx)
                if failure: self.c["pel_verify_failures"] += 1
            if failure:
                plan = self._fail(ph, step, failure); ctx = self.last_ctx; continue
            st.advance(); self.retries = 0; self._save()
            if st.no_progress_steps >= NO_PROGRESS_LIMIT:
                self.c["pel_no_progress"] += 1; self.ev("pel_no_progress", steps=st.no_progress_steps)
                st.no_progress_steps = 0
                plan = self._replan(ph, f"no progress in the last {NO_PROGRESS_LIMIT} steps"); ctx = self.last_ctx; continue
