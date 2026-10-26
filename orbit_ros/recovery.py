"""v0.8 extension: failure recovery as an alternative to abstention.

v0.7 diagnosis (OracleBackend, noise 0.1): the strict robust verifier abstained on ~65% of episodes, mostly because the plan's *declared
final pose* failed under some still-plausible hypothesis (866 of 1252 abstentions), not because a collision was likely.  Recovery therefore
replaces "refuse the whole plan" by a ladder of safe fallbacks that run only when verification has failed:

  rung 1  reprobe   a bounded number of extra plan-as-probe actions (same safe-probe machinery as v0.6), then re-verify the plan
  rung 2  guarded   execute the plan one motion at a time.  Before every move_forward the step must be collision-free under the plausible
                    hypotheses (posterior mass of failing hypotheses <= tau_action); if the full move is not, the move is shortened
                    ("creep").  After every step the execution monitor checks the pose; a trip stops the plan, the belief has already been
                    updated from the observed pose, and the planner replans from the real state.
  rung 3  abstain   only if not even a 1 m step is safe, or the recovery budget is spent.

The per-step check ignores the plan's final-pose / goal expectations on purpose: those are what made v0.7 abstain, and they are repaired by
replanning after observation instead.  What the guard does NOT give: any guarantee beyond the simulator's discrete model.
"""
from __future__ import annotations
from dataclasses import dataclass
from .robust import _believed_world, _run_commands

@dataclass(frozen=True)
class RecoveryConfig:
    enabled: bool = False              # recovery ladder on verification failure (implies monitoring)
    monitor: bool = False              # execution monitor stops a plan when its pose prediction is contradicted
    extra_probes: int = 2              # rung 1 budget: added to max_task_probes once per episode (0 disables the rung)
    guarded: bool = True               # rung 2
    tau_action: float = 0.0            # max posterior mass of plausible hypotheses under which one step may collide
    max_entries_per_phase: int = 4
    max_guarded_repairs: int = 12      # repair budget per guarded phase (the normal budget is agent2d.MAX_REPAIRS)

class StepGuard:
    """Per-step collision guard.  The landmark map under each plausible hypothesis is fixed when the plan is made (landmarks are static);
    for each step only the robot pose is replaced by the real current pose."""
    def __init__(self, strategy, ctx, common_obstacles, fov, tau_action: float):
        self.strategy, self.tau = strategy, tau_action
        self.sims = {}
        for h, _ in strategy.plausible():
            sim, _ = _believed_world(h, ctx["pose"], ctx["raw"], common_obstacles, fov)
            self.sims[h.key] = sim

    def risk(self, pose: dict, action: dict) -> float:
        if not self.sims: return float("inf")     # V081: fail closed - no plausible hypothesis to simulate (collapsed belief)
        cmd, mass = self.strategy.belief, 0.0
        for key, sim in self.sims.items():
            sim.x, sim.y, sim.yaw = pose["x"], pose["y"], pose["yaw_deg"]; sim.collisions = 0
            if _run_commands(sim, [action], cmd): mass += self.strategy.ident.weights[key]
        return mass

    def fit(self, pose: dict, action: dict):
        """Longest prefix of a move_forward that is acceptably safe.  Returns (action|None, risk, mode) with mode in full|creep|unsafe."""
        n = (action.get("args") or {}).get("distance")
        if action.get("tool") != "move_forward" or not isinstance(n, int) or isinstance(n, bool) or n <= 0: return action, 0.0, "full"
        r = 1.0
        for k in range(n, 0, -1):
            a = dict(action, args=dict(action["args"], distance=k)); r = self.risk(pose, a)
            if r <= self.tau + 1e-12: return a, r, ("full" if k == n else "creep")
        return None, r, "unsafe"
