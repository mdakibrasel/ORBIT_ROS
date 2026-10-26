import unittest
import numpy as np
from orbit_ros.world2d import World2D, Obj
from orbit_ros.embodiment import Hypothesis
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment, NOMINAL
from orbit_ros.tasks2d import make_task, Phase
from orbit_ros.agent2d import PRESETS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.robust import RobustStrategy
from orbit_ros.voi import VoiStrategy, run_episode_v07, error_category, plan_cost
from orbit_ros.stats07 import holm, cluster_bootstrap_diff

def act(tool, **a): return {"tool": tool, "args": a}
def ctx_for(world): return {"pose": world.pose(), "raw": world.describe_view(), "use_memory": False, "landmarks": {}}

def setup(ident="hybrid", **kw):
    h = Hypothesis(False, False, 0.5, 1.0)
    world = apply_embodiment(World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", 6, 6)]), h)
    st = VoiStrategy(ident, **kw); st.start(world); st.current_phase = Phase("go", "goto", {"label": "red box"})
    return world, st, ctx_for(world)

class RiskVerifierTests(unittest.TestCase):
    def test_tau_zero_matches_strict_acceptance_and_large_tau_accepts(self):
        world, st, ctx = setup(tau=0.0)
        plan = {"actions": [act("move_forward", distance=2)], "expect": {"x": 4, "y": 4}}
        v0 = st.check(plan, ctx, st.current_phase, set(), world.fov)
        _, st2, _ = setup(tau=1.0)
        v1 = st2.check(plan, ctx, st2.current_phase, set(), world.fov)
        self.assertTrue(v1.safe)                       # tau = 1 accepts anything
        self.assertEqual(v0.safe, v0.failing_mass <= 1e-12)

    def test_accepted_risk_is_logged_and_bounded_by_tau(self):
        world, st, ctx = setup(tau=0.5)
        plan = {"actions": [act("move_forward", distance=2)], "expect": {"x": 4, "y": 4}}
        v = st.check(plan, ctx, st.current_phase, set(), world.fov)
        rec = st.check_log[-1]
        if v.safe: self.assertLessEqual(rec["accepted_risk"], 0.5 + 1e-9)
        for k in ("failing_mass", "n_failing", "tail_driven", "categories", "top_weights", "failing"): self.assertIn(k, rec)

    def test_check_log_names_failing_hypotheses_and_categories(self):
        world = World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", 6, 6)])
        st = VoiStrategy("none"); st.start(world); st.current_phase = Phase("go", "goto", {"label": "red box"})
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        v = st.check(plan, ctx_for(world), st.current_phase, set(), world.fov)
        self.assertFalse(v.safe)
        rec = st.check_log[-1]; self.assertGreater(rec["n_failing"], 0); self.assertIn("final_pose", rec["categories"])
        self.assertFalse(rec["tail_driven"])          # uniform posterior: no tail

    def test_error_categories(self):
        self.assertEqual(error_category("action[0] move_forward{'distance': 3}: blocked after 1 m"), "collision")
        self.assertEqual(error_category("expect.x=6 but dry-run gives 5"), "final_pose")
        self.assertEqual(error_category("goal cell (4, 1) is not adjacent to final pose (6,2)"), "goal")

class VoiGateTests(unittest.TestCase):
    def test_voi_disabled_is_identical_to_v06_choice(self):
        world, st, ctx = setup(voi=False)
        ref = RobustStrategy("hybrid", "robust"); ref.start(world)
        plan = {"actions": [act("move_forward", distance=2), act("turn", direction="left", degrees=90)]}
        a = st.plan_probe(plan, ctx, set(), world.fov, force=True); b = ref.plan_probe(plan, ctx, set(), world.fov, force=True)
        self.assertEqual(a["name"], b["name"])

    def test_gate_probes_when_ambiguity_blocks_the_plan_and_skips_when_cost_dominates(self):
        world, st, ctx = setup(voi=True); st.current_phase = Phase("m", "motion", {})      # no goal-cell requirement
        plan = {"actions": [act("move_forward", distance=2)], "expect": {"x": 4, "y": 4}}
        p = st.plan_probe(plan, ctx, set(), world.fov, force=True)
        self.assertIsNotNone(p); self.assertGreater(p["voi_net"], 0)
        world2, st2, ctx2 = setup(voi=True, c_travel=1000.0); st2.current_phase = Phase("m", "motion", {})
        self.assertIsNone(st2.plan_probe(plan, ctx2, set(), world2.fov, force=True))
        self.assertEqual(st2.voi_skips, 1); self.assertFalse(st2.voi_log[-1]["chosen"])

    def test_gate_skips_probe_when_plan_is_already_acceptable(self):
        world = apply_embodiment(World2D(14, 9, (2, 4), 0, []), NOMINAL)
        st = VoiStrategy("probe", voi=True); st.start(world)
        st.ident.weights = {k: (1.0 if k == NOMINAL.key else 0.0) for k in st.ident.weights}      # certain belief
        st.current_phase = Phase("go", "motion", {})
        plan = {"actions": [act("move_forward", distance=2)], "expect": {"x": 4, "y": 4}}
        self.assertIsNone(st.plan_probe(plan, ctx_for(world), set(), world.fov, force=True))

    def test_choose_probe_attribute_is_restored(self):
        world, st, ctx = setup(voi=True)
        st.plan_probe({"actions": [act("move_forward", distance=2)]}, ctx, set(), world.fov, force=True)
        self.assertNotIn("choose_probe", st.ident.__dict__)

    def test_effective_eps_shrinks_with_repeats(self):
        st = VoiStrategy("hybrid", eps=0.1)
        e = [st._eps_after(n) for n in (1, 2, 3)]
        self.assertAlmostEqual(e[0], 0.1); self.assertTrue(e[0] > e[1] > e[2]); self.assertEqual(VoiStrategy("hybrid", eps=0.0)._eps_after(3), 0.0)

    def test_probe_budget_still_respected(self):
        world, st, ctx = setup(voi=True); st.task_probes = st.max_task_probes
        self.assertIsNone(st.plan_probe({"actions": [act("move_forward", distance=2)]}, ctx, set(), world.fov, force=True))

class EpisodeTests(unittest.TestCase):
    def _row(self, **kw):
        world, task = make_task("goto", 1, 90, 3, False, 1); apply_embodiment(world, Hypothesis(False, True, 0.5, 1.0))
        return run_episode_v07(world, task, PRESETS["S5"], OracleBackend(0, 0), VoiStrategy("hybrid", **kw))

    def test_v07_runner_reproduces_v06_when_gate_and_risk_are_off(self):
        from orbit_ros.agent_robust import run_episode_robust
        import itertools
        for fam, yaw, h in itertools.product(("goto", "clean_left"), (0, 90), hypothesis_space()):
            w1, t1 = make_task(fam, 1, yaw, 3, False, 1); apply_embodiment(w1, h)
            w2, t2 = make_task(fam, 1, yaw, 3, False, 1); apply_embodiment(w2, h)
            a = run_episode_robust(w1, t1, PRESETS["S5"], OracleBackend(0, 0), RobustStrategy("hybrid", "robust", seed=1))
            b = run_episode_v07(w2, t2, PRESETS["S5"], OracleBackend(0, 0), VoiStrategy("hybrid", seed=1))
            for k in ("success", "collisions", "aborted", "task_probes", "env_tool_calls", "llm_calls", "final_x", "final_y"): self.assertEqual(a[k], b[k], (fam, yaw, h.key, k))

    def test_diagnostic_columns_present(self):
        row = self._row(voi=True, tau=0.1)
        for k in ("n_checks", "n_rejects", "reject_tail_driven", "last_reject_reason", "voi_calls", "voi_skips", "task_probes"): self.assertIn(k, row)
        self.assertEqual(row["env_tool_calls"] >= 1, True)

    def test_voi_never_exceeds_probe_budget(self):
        self.assertLessEqual(self._row(voi=True)["task_probes"], 3)

class StatsTests(unittest.TestCase):
    def test_holm_matches_hand_computation(self):
        self.assertEqual([round(x, 6) for x in holm([0.01, 0.04, 0.03])], [0.03, 0.06, 0.06])
        self.assertEqual(holm([0.5]), [0.5])
        adj = holm([0.001, 0.2, 0.9]); self.assertTrue(all(0 <= x <= 1 for x in adj))

    def test_cluster_bootstrap_identical_arrays_give_zero_and_p_one(self):
        x = np.arange(48) % 2; c = np.repeat(np.arange(4), 12)
        d, lo, hi, p = cluster_bootstrap_diff(x, x, c, B=500)
        self.assertEqual((d, lo, hi), (0.0, 0.0, 0.0)); self.assertEqual(p, 1.0)

    def test_cluster_bootstrap_detects_consistent_shift(self):
        c = np.repeat(np.arange(30), 4); a = np.ones(120); b = np.zeros(120)
        d, lo, hi, p = cluster_bootstrap_diff(a, b, c, B=1000)
        self.assertEqual(d, 1.0); self.assertGreater(lo, 0.9); self.assertLessEqual(p, 0.01)

if __name__ == "__main__": unittest.main()
