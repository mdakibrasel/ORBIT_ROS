"""v0.7 extension: value-of-information (VOI) gate, risk-based abstention, and abstention diagnostics.

Built on top of v0.6.0 (`robust.py`, `agent_robust.py` are unchanged). `VoiStrategy` is a `RobustStrategy` subclass:

* risk-based verifier (`tau`): instead of "every hypothesis with weight >= min_weight must pass", accept a plan if the posterior mass
  of hypotheses under which it fails, for the best command translation, is <= tau.  tau = 0 is the strictest setting.
* VOI gate (`voi=True`): a candidate probe is executed only if its expected reduction in decision loss exceeds its cost.
      decision loss  D(w) = min( c_fail_eff * min_translation sum_h w_h * fails(plan | translation, h),  c_abstain )
      c_fail_eff     = c_fail + c_len * (travel of the remaining plan)
      probe cost     = c_travel * (metres + 0.5 per turn) + c_risk * (posterior mass of hypotheses where the probe itself would fail) + c_replan
      VOI(probe)     = D(w) - E_outcome[ D(w | outcome) ]       (same likelihood as the identifier: 1-eps on match, eps otherwise)
  The probe with the largest VOI - cost is run, and probing stops when that is <= min_net.
* diagnostics: every verification and every VOI decision is logged (posterior top-5, failing hypotheses and their weights, error categories,
  whether the rejection was driven only by low-weight tail hypotheses), so abstentions can be analysed afterwards.

All cost constants are in "metres of travel" units and are modelling choices, not measurements.
"""
from __future__ import annotations
from collections import Counter
from .robust import RobustStrategy, check_plan, RobustVerdict, _believed_world, _run_commands
from .embodiment_runtime import travel_of

TAIL_W = 0.05          # a rejection is "tail-driven" if every failing hypothesis has weight below this

def plan_cost(plan) -> float:
    acts = [a for a in plan.get("actions", []) if a.get("tool") in ("move_forward", "turn")]
    return travel_of(acts) + 0.5 * sum(a["tool"] == "turn" for a in acts)

def error_category(err: str) -> str:
    if "blocked" in err or "outside" in err or "occupied" in err: return "collision"
    if err.startswith("expect") or "expect." in err: return "final_pose"
    if "goal cell" in err: return "goal"
    if "unknown tool" in err: return "invalid_tool"
    return "other"

class _PhaseTap:
    """Task proxy that tells the strategy which phase is being planned (the v0.6 runner never passes it to plan_probe)."""
    def __init__(self, task, strategy): self._task, self._strategy = task, strategy
    def __getattr__(self, k): return getattr(self._task, k)
    @property
    def phases(self):
        for ph in self._task.phases:
            self._strategy.current_phase = ph
            yield ph

class VoiStrategy(RobustStrategy):
    def __init__(self, ident: str, verifier: str = "robust", tau: float | None = None, voi: bool = False,
                 c_fail=6.0, c_len=0.5, c_abstain=30.0, c_travel=1.0, c_risk=20.0, c_replan=1.0, min_net=0.0, lookahead=3, **kw):
        if verifier != "robust": raise ValueError("VoiStrategy extends the robust verifier")
        super().__init__(ident, "robust", **kw)
        self.tau, self.voi = tau, voi
        self.c_fail, self.c_len, self.c_abstain, self.c_travel, self.c_risk, self.c_replan, self.min_net, self.lookahead = c_fail, c_len, c_abstain, c_travel, c_risk, c_replan, min_net, lookahead
        self.name = f"{ident}_robust" + ("_risk" if tau is not None else "") + ("_voi" if voi else "")
        self.current_phase = None
        self.check_log: list[dict] = []; self.voi_log: list[dict] = []
        self.voi_calls = self.voi_skips = 0

    # ------------------------------------------------------------ risk-based verifier + logging
    def _sigs(self):
        seen, out = set(), []
        for h in [self.belief] + [h for h, _ in sorted(self.plausible(), key=lambda hw: -hw[1])] + list(self.ident.hypotheses):
            s = (h.left_right_swapped, h.move_gain)
            if s not in seen: seen.add(s); out.append(h)
        return out

    def check(self, plan, ctx, ph, common_obstacles, fov) -> RobustVerdict:
        if self.tau is None: v = super().check(plan, ctx, ph, common_obstacles, fov); accepted_mass = 0.0
        else:
            hw = [(h, self.ident.weights[h.key]) for h in self.ident.hypotheses if self.ident.weights[h.key] > 1e-12]
            best, best_cmd = None, None
            for cmd in self._sigs():
                v = check_plan(plan, ctx, ph, hw, cmd, common_obstacles, fov, min_weight=1e-12)
                if best is None or v.failing_mass < best.failing_mass - 1e-12: best, best_cmd = v, cmd
            v = best; accepted_mass = v.failing_mass
            if v.failing_mass <= self.tau + 1e-12: v.safe, v.errors, self._cmd_override = True, [], best_cmd
            else: v.safe, self._cmd_override = False, None
        top = sorted(self.ident.weights.items(), key=lambda kv: -kv[1])[:5]
        cats = Counter(error_category(e) for _, _, errs in v.failing for e in errs[:1])
        self.check_log.append({"safe": v.safe, "failing_mass": round(v.failing_mass, 4), "n_failing": len(v.failing), "n_checked": v.checked,
                               "accepted_risk": round(accepted_mass, 4) if v.safe else 0.0,
                               "tail_driven": bool(v.failing) and all(w < TAIL_W for _, w, _ in v.failing),
                               "categories": dict(cats), "top_weights": [(k, round(w, 3)) for k, w in top],
                               "failing": [(k, round(w, 3)) for k, w, _ in v.failing][:6], "phase": getattr(self.current_phase, "kind", None)})
        return v

    # ------------------------------------------------------------ VOI-gated probe choice
    def _failure_matrix(self, plan, ctx, common, fov):
        ph, F = self.current_phase, {}
        for cmd in self._sigs():
            F[(cmd.left_right_swapped, cmd.move_gain)] = {h.key: not check_plan(plan, ctx, ph, [(h, 1.0)], cmd, common, fov, 0.0).safe for h in self.ident.hypotheses}
        return F

    def _loss(self, w: dict, F, cf: float) -> float:
        """Decision loss under the SAME acceptance rule the verifier uses: act if some translation is accepted, else abstain."""
        best = self.c_abstain
        for row in F.values():
            m = sum(w[k] * f for k, f in row.items() if w[k])
            ok = (m <= self.tau + 1e-12) if self.tau is not None else all(not f for k, f in row.items() if w[k] >= self.min_weight)
            if ok: best = min(best, cf * m)
        return best

    def _posterior_loss(self, probe, w, F, cf, eps: float) -> float:
        out = probe["outcomes"]
        labels = {str(out.get(k, "unknown")) for k in w}
        joint = {}
        for o in labels:
            like = {k: ((1 - eps) if str(out.get(k, "unknown")) == o else eps) for k in w}
            joint[o] = {k: w[k] * like[k] for k in w}
        z = sum(sum(j.values()) for j in joint.values())
        if z <= 0: return self._loss(w, F, cf)
        exp = 0.0
        for o, j in joint.items():
            m = sum(j.values())
            if m > 0: exp += (m / z) * self._loss({k: v / m for k, v in j.items()}, F, cf)
        return exp

    def _eps_after(self, n: int) -> float:
        """Effective misread rate after n agreeing repeats of a probe: eps^n / ((1-eps)^n + eps^n)."""
        e = self.eps
        return 0.0 if e <= 0 else e ** n / ((1 - e) ** n + e ** n)

    def plan_probe(self, plan, ctx, common_obstacles, fov, force=False):
        if not self.voi: return super().plan_probe(plan, ctx, common_obstacles, fov, force)
        if not self.wants_probe() or not plan: return None
        captured = []
        self.ident.choose_probe = lambda cands: (captured.extend(cands), None)[1]     # reuse v0.6 candidate enumeration unchanged
        try: super().plan_probe(plan, ctx, common_obstacles, fov, force=True)
        finally: del self.ident.choose_probe
        captured = [c for c in captured if c.get("safe")]
        if not captured: return None
        self.voi_calls += 1
        w, F = dict(self.ident.weights), self._failure_matrix(plan, ctx, common_obstacles, fov)
        cf = self.c_fail + self.c_len * plan_cost(plan)
        d0 = self._loss(w, F, cf)
        best, best_net, rows = None, float("-inf"), []
        reps = max(1, min(self.lookahead, self.max_task_probes - self.task_probes))
        for cand in captured:
            risk = 0.0
            for h in self.ident.hypotheses:
                if not w[h.key]: continue
                sim, _ = _believed_world(h, ctx["pose"], ctx["raw"], common_obstacles, fov)
                if _run_commands(sim, cand["actions"], self.belief): risk += w[h.key]
            unit = self.c_travel * cand["cost"] + self.c_risk * risk + self.c_replan
            # k-repeat lookahead: one noisy reading rarely moves the posterior past the acceptance threshold, so a probe is also
            # valued as a campaign of n agreeing repeats costing n times as much (n = 1 recovers the myopic one-step VOI).
            for n in range(1, reps + 1):
                voi = d0 - self._posterior_loss(cand, w, F, cf, self._eps_after(n))
                net = voi - n * unit
                rows.append((cand["name"], n, round(voi, 3), round(n * unit, 3)))
                if net > best_net: best, best_net = cand, net
        chosen = best_net > self.min_net
        self.voi_log.append({"d0": round(d0, 3), "best_net": round(best_net, 3), "chosen": chosen, "n_cands": len(captured),
                             "top": sorted(rows, key=lambda r: r[2] - r[3], reverse=True)[:3]})
        if not chosen: self.voi_skips += 1; return None
        best = dict(best); best["voi_net"] = round(best_net, 4); best["expected_information_gain_bits"] = None
        return best

    # ------------------------------------------------------------ diagnostics
    def diag_row(self) -> dict:
        rej = [c for c in self.check_log if not c["safe"]]
        acc = [c for c in self.check_log if c["safe"]]
        cats = Counter(); [cats.update(c["categories"]) for c in rej]
        last = rej[-1] if rej else None
        return {"n_checks": len(self.check_log), "n_rejects": len(rej),
                "reject_tail_driven": sum(c["tail_driven"] for c in rej), "reject_mass_mean": round(sum(c["failing_mass"] for c in rej) / len(rej), 4) if rej else "",
                "accepted_risk_mean": round(sum(c["accepted_risk"] for c in acc) / len(acc), 4) if acc else "",
                "reject_cat_collision": cats.get("collision", 0), "reject_cat_final_pose": cats.get("final_pose", 0), "reject_cat_goal": cats.get("goal", 0),
                "last_reject_reason": ("tail_hypotheses" if last["tail_driven"] else (max(last["categories"], key=last["categories"].get) if last["categories"] else "other")) if last else "",
                "voi_calls": self.voi_calls, "voi_skips": self.voi_skips}

def run_episode_v07(world, task, cfg, backend, strategy: VoiStrategy, events=None):
    """v0.6 episode runner with the current phase exposed to the strategy; no other change."""
    from .agent_robust import run_episode_robust
    row = run_episode_robust(world, _PhaseTap(task, strategy), cfg, backend, strategy, events)
    row.update(strategy.diag_row())
    return row
