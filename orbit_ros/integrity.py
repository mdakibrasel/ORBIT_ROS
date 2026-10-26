"""v0.8.1: belief-integrity fixes for plan-as-probe identification (opt-in).

Nothing here changes v0.6 - v0.8.0 behaviour: the fixes only act when a strategy is built with `make_strategy_v081` /
`SafeRobustStrategy` / `SafeVoiStrategy`.  The shipped v0.7 and v0.8.0 results therefore stay reproducible from this code base.

Audit finding (docs/V0_8_1_AUDIT.md).  On the v0.7 held-out seeds at noise 0, 20 of 1,920 episodes per probe-using arm end with
EVERY hypothesis at weight 0 (final_confidence == 0).  Those 1% of episodes produce 15-25% of all collisions of the arm.  All 20 have the
same cause, a chain of three defects:

  1. A plan-as-probe whose move is blocked by an obstacle the believed world does not contain (an object the camera has not seen) stops
     early.  `RobustStrategy.finish_probe` still scores the truncated pose against the *unobstructed* per-hypothesis predictions.  If the
     truncated pose happens to equal the prediction of a wrong move gain (blocked after 1 m == gain 0.25 for a 3 m move), the true
     hypothesis is eliminated (eps = 0) or heavily down-weighted (eps > 0).
  2. With eps = 0, `EmbodimentIdentifier.update` keeps only the hypotheses that match an observation.  When the next valid observation
     matches only hypotheses that were already eliminated, the renormaliser divides by 0 and sets every weight to 0 (the `not keep`
     surprise branch does not cover this case).  The MAP belief then falls back to the nominal robot (tie-break on deviation).
  3. With no hypothesis left, the "strict" robust verifier checks nothing and returns safe=True (vacuous truth), the risk verifier sees a
     failing mass of 0, and the v0.8 step guard has no hypothesis to simulate and lets every step through.

Fixes (each can be switched off for ablation):
  censor_failed_probes  a probe during which a command was blocked or invalid is not used as identification evidence (it still counts
                        against the probe budget; env calls and RNG draws are the same as for an uncensored probe).  Blocked task commands
                        were already excluded from per-command evidence (`observe_command` ignores results with ok=False); this applies
                        the same rule to probes.
  no_collapse           when a hard (eps = 0) update would set every weight to 0, the current observation is kept as hard evidence and
                        all EARLIER evidence is re-scored with a small misread rate `eps_history`: the hypotheses that agree with the
                        current observation and contradict the fewest earlier observations win.  Handles both a corrupted earlier
                        observation and a genuine change of the embodiment (drift).  Each such event is counted (`collapse_resets`).
  fail_closed           a robust/risk verifier that has no hypothesis to check rejects the plan instead of accepting it.

What this does NOT do: model *why* a probe was blocked (a censored-likelihood update that keeps the information in a blocked probe is
possible but not implemented); detect embodiments outside the 12-hypothesis model; say anything about real robots or LLM planners.
"""
from __future__ import annotations
from dataclasses import dataclass
from .embodiment import EmbodimentIdentifier, SafeProbeError
from .embodiment_runtime import travel_of
from .robust import RobustStrategy, RobustVerdict, view_label
from .voi import VoiStrategy

COLLAPSE_MSG = "no plausible embodiment hypothesis left (belief collapsed): fail-closed rejection"

@dataclass(frozen=True)
class IntegrityConfig:
    censor_failed_probes: bool = True
    no_collapse: bool = True
    fail_closed: bool = True
    eps_history: float = 0.01          # misread rate assumed for PAST evidence after a contradiction (modelling choice, not tuned)

    @property
    def any(self) -> bool: return self.censor_failed_probes or self.no_collapse or self.fail_closed

# ------------------------------------------------------------------ identifier
class SafeIdentifier(EmbodimentIdentifier):
    """EmbodimentIdentifier whose hard (eps = 0) update can never leave every hypothesis at weight 0.

    Instances are created by `make_safe(ident)` (class swap), so construction stays exactly the same as `_new_identifier`."""
    def _init_safe(self, eps_history: float) -> None:
        if not 0.0 < eps_history < 0.5: raise ValueError("eps_history must be in (0, 0.5)")
        self._prior = dict(self.weights)
        self._evidence: list[tuple[dict, str]] = []
        self.eps_history, self.collapse_resets = eps_history, 0

    def update(self, probe, observed_outcome: str) -> None:
        if not probe.get("safe", False): raise SafeProbeError("Refusing to update from a probe not marked safe")
        outcomes, obs = probe.get("outcomes", {}), str(observed_outcome)
        if self.epsilon <= 0:
            keep = [h.key for h in self.hypotheses if str(outcomes.get(h.key, "unknown")) == obs]
            if keep and sum(self.weights[k] for k in keep) <= 0.0:
                self._reset_from_history(outcomes, obs, keep); return
        n_hist = len(self.history)
        super().update(probe, observed_outcome)
        if not (self.history[n_hist:] and self.history[-1].get("surprise")):
            self._evidence.append((dict(outcomes), obs))

    def _reset_from_history(self, outcomes: dict, obs: str, keep: list[str]) -> None:
        e, keep_set, w = self.eps_history, set(keep), {}
        for h in self.hypotheses:
            k = h.key
            if k not in keep_set: w[k] = 0.0; continue
            v = self._prior[k]
            for o, ob in self._evidence: v *= (1.0 - e) if str(o.get(k, "unknown")) == ob else e
            w[k] = v
        z = sum(w.values())                     # > 0: keep is non-empty, prior > 0, e > 0
        self.weights = {k: v / z for k, v in w.items()}
        self._evidence.append((dict(outcomes), obs))
        self.probes_used += 1; self.collapse_resets += 1
        self.history.append({"probe": "collapse_reset", "observed": obs, "surprise": True, "collapse_reset": True, "remaining_hypotheses": len(keep)})

def make_safe(ident: EmbodimentIdentifier, eps_history: float = 0.01) -> SafeIdentifier:
    if len(ident.history): raise ValueError("make_safe must be applied before the identifier has seen evidence")
    ident.__class__ = SafeIdentifier
    ident._init_safe(eps_history)
    return ident

# ------------------------------------------------------------------ strategies
class IntegrityMixin:
    """Put first in the bases: `class S(IntegrityMixin, RobustStrategy)`.  Call `_init_integrity` at the end of __init__."""
    def _init_integrity(self, icfg: IntegrityConfig | None) -> None:
        self.icfg = icfg or IntegrityConfig()
        if self.icfg.no_collapse: make_safe(self.ident, self.icfg.eps_history)
        self.task_probes_censored = self.fail_closed_rejects = 0
        self._world, self._probe_snap = None, None

    def start(self, world):
        self._world = world
        return super().start(world)

    def plan_probe(self, plan, ctx, common_obstacles, fov, force=False):
        p = super().plan_probe(plan, ctx, common_obstacles, fov, force)
        # The runner executes p["actions"] next and then calls finish_probe; a blocked or invalid command in between shows up in
        # these two counters (the robot sees the same thing as the tool result "blocked after k m").
        self._probe_snap = (self._world.collisions, self._world.invalid_calls) if (p is not None and self._world is not None) else None
        return p

    def finish_probe(self, probe: dict, world) -> None:
        snap, self._probe_snap = self._probe_snap, None
        failed = snap is not None and (world.collisions > snap[0] or world.invalid_calls > snap[1])
        if not (self.icfg.censor_failed_probes and failed): return super().finish_probe(probe, world)
        # Same bookkeeping, env calls and RNG draws as RobustStrategy.finish_probe, but the truncated outcome is not used as evidence.
        pose = world.execute("get_pose", {})["data"]; raw = world.execute("describe_view", {})["data"]
        view_label(pose, raw, self.known_labels, self.noise_p, self.rng)
        self.task_probes += 1; self.task_probe_travel += travel_of(probe["actions"]); self._cmd_override = None
        self.task_probes_censored += 1

    def _no_hypothesis_to_check(self) -> bool:
        if getattr(self, "tau", None) is not None: return not any(w > 1e-12 for w in self.ident.weights.values())
        return not self.plausible()

    def check(self, plan, ctx, ph, common_obstacles, fov) -> RobustVerdict:
        if self.icfg.fail_closed and self._no_hypothesis_to_check():
            self.fail_closed_rejects += 1; self._cmd_override = None
            v = RobustVerdict(False, 0, [], 1.0, [COLLAPSE_MSG])
            if hasattr(self, "check_log"):
                self.check_log.append({"safe": False, "failing_mass": 1.0, "n_failing": 0, "n_checked": 0, "accepted_risk": 0.0, "tail_driven": False,
                                       "categories": {"collapsed": 1}, "top_weights": [], "failing": [], "phase": getattr(getattr(self, "current_phase", None), "kind", None)})
            return v
        return super().check(plan, ctx, ph, common_obstacles, fov)

    def integrity_row(self) -> dict:
        return {"task_probes_censored": self.task_probes_censored, "collapse_resets": getattr(self.ident, "collapse_resets", 0),
                "fail_closed_rejects": self.fail_closed_rejects, "posterior_collapsed": int(max(self.ident.weights.values()) <= 0.0)}

class SafeRobustStrategy(IntegrityMixin, RobustStrategy):
    def __init__(self, *a, integrity: IntegrityConfig | None = None, **kw):
        super().__init__(*a, **kw); self._init_integrity(integrity)

class SafeVoiStrategy(IntegrityMixin, VoiStrategy):
    def __init__(self, *a, integrity: IntegrityConfig | None = None, **kw):
        super().__init__(*a, **kw); self._init_integrity(integrity)

INTEGRITY_KEYS = ("task_probes_censored", "collapse_resets", "fail_closed_rejects", "posterior_collapsed")

def integrity_row_of(strategy) -> dict:
    """Integrity columns for any strategy (old strategies: counters 0, collapse flag from the weights)."""
    if hasattr(strategy, "integrity_row"): return strategy.integrity_row()
    return {"task_probes_censored": 0, "collapse_resets": 0, "fail_closed_rejects": 0,
            "posterior_collapsed": int(max(strategy.ident.weights.values()) <= 0.0) if hasattr(strategy, "ident") else 0}
