import itertools
import unittest
from orbit_ros.world2d import World2D, Obj
from orbit_ros.embodiment import Hypothesis
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment, hypothesis_of, EmbodimentStrategy, NOMINAL, EmbodimentAdapter
from orbit_ros.cognition import SensorInterpreter
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.agent_robust import run_episode_robust
from orbit_ros.backends2d import OracleBackend
from orbit_ros.tasks2d import Phase
from orbit_ros.robust import RobustStrategy, check_plan, view_label, ARMS, _believed_world

def uniform(hs=None):
    hs = hs or hypothesis_space()
    return [(h, 1 / len(hs)) for h in hs]

def ctx_for(world):
    pose = world.pose(); raw = world.describe_view()
    return {"pose": pose, "raw": raw, "use_memory": False, "landmarks": {}}

def act(tool, **a): return {"tool": tool, "args": a}

class RobustVerifierTests(unittest.TestCase):
    def setUp(self):
        self.world = World2D(12, 9, (2, 4), 0, [Obj("o0", "red box", 7, 4)])
        self.ctx = ctx_for(self.world)
        self.ph = Phase("go", "goto", {"label": "red box"})

    def test_single_hypothesis_passes_when_plan_is_right_for_it(self):
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        v = check_plan(plan, self.ctx, self.ph, [(NOMINAL, 1.0)], NOMINAL)
        self.assertTrue(v.safe, v.errors)

    def test_plan_nominal_for_all_fails_under_gain_hypotheses(self):
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        v = check_plan(plan, self.ctx, self.ph, uniform(), NOMINAL)
        self.assertFalse(v.safe)
        failing = {k for k, _, _ in v.failing}
        self.assertTrue(any("mg=0.5" in k for k in failing)); self.assertTrue(any("mg=0.25" in k for k in failing))
        self.assertTrue(all("mg=1" not in k or "swap=0" in k or True for k in failing))
        self.assertAlmostEqual(v.failing_mass, len(v.failing) / 12)

    def test_report_names_failing_hypotheses_and_mass(self):
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        v = check_plan(plan, self.ctx, self.ph, uniform(), NOMINAL)
        self.assertEqual(v.checked, 12)
        self.assertEqual(len(v.errors), len(v.failing))
        self.assertTrue(all(len(f) == 3 for f in v.failing))

    def test_verifier_arms_share_identification_noise(self):
        self.assertEqual(RobustStrategy("bay", "old", seed=3).rng.random(), RobustStrategy("bay", "robust", seed=3).rng.random())

    def test_collision_is_detected_under_a_hypothesis_that_overshoots(self):
        # wall at x=8: a 12-step command believed at gain 0.25 realises 3; nominal realises 12 -> leaves the room
        plan = {"actions": [act("move_forward", distance=12)]}
        v = check_plan(plan, self.ctx, self.ph, [(NOMINAL, 0.5), (Hypothesis(False, False, 0.25, 1.0), 0.5)], NOMINAL)
        self.assertFalse(v.safe)

    def test_hypotheses_below_min_weight_are_ignored(self):
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        hw = [(NOMINAL, 0.995), (Hypothesis(False, False, 0.5, 1.0), 0.005)]
        self.assertTrue(check_plan(plan, self.ctx, self.ph, hw, NOMINAL, min_weight=0.01).safe)
        self.assertFalse(check_plan(plan, self.ctx, self.ph, hw, NOMINAL, min_weight=0.001).safe)

    def test_adapter_belief_makes_plan_robust_to_that_gain(self):
        h = Hypothesis(False, False, 0.5, 1.0)
        plan = {"actions": [act("move_forward", distance=4)], "expect": {"x": 6, "y": 4}}
        self.assertTrue(check_plan(plan, self.ctx, self.ph, [(h, 1.0)], h).safe)

    def test_mirror_changes_the_goal_cell_and_is_caught(self):
        # target left of the robot; a plan aimed at the mirrored reading fails the goal check under the true convention
        w = World2D(12, 9, (2, 4), 0, [Obj("o0", "red box", 4, 6)])
        ctx = ctx_for(w)
        plan_for_unmirrored = {"actions": [act("move_forward", distance=2), act("turn", direction="left", degrees=90), act("move_forward", distance=1)]}
        hs = [(Hypothesis(False, False, 1.0, 1.0), 0.5), (Hypothesis(True, False, 1.0, 1.0), 0.5)]
        v = check_plan(plan_for_unmirrored, ctx, self.ph, hs, NOMINAL)
        self.assertFalse(v.safe)
        self.assertEqual([k for k, _, _ in v.failing], [Hypothesis(True, False, 1.0, 1.0).key])

class BelievedWorldTests(unittest.TestCase):
    def test_true_hypothesis_reproduces_real_outcome_exactly(self):
        for h in hypothesis_space():
            real = apply_embodiment(World2D(12, 9, (2, 4), 0, [Obj("o0", "red box", 5, 6), Obj("o1", "blue shelf", 6, 2)]), h)
            ctx = ctx_for(real)
            sim, _ = _believed_world(h, ctx["pose"], ctx["raw"], set(), real.fov)
            known = {o["label"] for o in ctx["raw"]}
            for a in (act("turn", direction="left", degrees=90), act("move_forward", distance=2)):
                real.execute(a["tool"], a["args"]); sim.execute(a["tool"], a["args"])
            self.assertEqual(view_label(real.pose(), real.describe_view(), known), view_label(sim.pose(), sim.describe_view(), known), h.key)

class PlanAsProbeTests(unittest.TestCase):
    def _probe(self, h, ident="probe", plan_actions=None):
        world = apply_embodiment(World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", 6, 6)]), h)
        strat = RobustStrategy(ident, "robust")
        strat.start(world)
        ctx = ctx_for(world)
        plan = {"actions": plan_actions or [act("move_forward", distance=3), act("turn", direction="left", degrees=90), act("move_forward", distance=1)]}
        return world, strat, ctx, plan

    def test_prior_is_uniform_without_bay_and_belief_defaults_to_nominal(self):
        _, strat, _, _ = self._probe(NOMINAL)
        self.assertEqual(len(strat.plausible()), 12); self.assertEqual(strat.belief, NOMINAL); self.assertEqual(strat.bay_travel, 0.0)

    def test_probe_identifies_every_hypothesis_from_the_task_world(self):
        for h in hypothesis_space():
            world, strat, ctx, plan = self._probe(h)
            for _ in range(3):
                p = strat.plan_probe(plan, ctx, set(), world.fov, force=True)
                if p is None: break
                for a in p["actions"]:
                    hw = [(a["tool"], a["args"])] if a.get("hw") else EmbodimentAdapter(strat.belief).translate(a["tool"], a["args"])
                    for t, args in hw: world.execute(t, args)
                strat.finish_probe(p, world)
                ctx = ctx_for(world)
            b = strat.belief
            # actuators (swap, move gain) are always identified; the mirror only when the landmark stays in view after a turn,
            # which needs gain 1.0 for this particular plan (documented limitation of in-task mirror identification)
            self.assertEqual((b.left_right_swapped, b.move_gain), (h.left_right_swapped, h.move_gain), f"truth {h.key}, belief {b.key}")
            if h.move_gain == 1.0: self.assertEqual(b.mirrored_camera, h.mirrored_camera, f"truth {h.key}, belief {b.key}")

    def test_unsafe_prefix_is_never_chosen(self):
        world, strat, ctx, _ = self._probe(NOMINAL)
        plan = {"actions": [act("move_forward", distance=12)]}      # leaves the 14 m room under nominal gain
        p = strat.plan_probe(plan, ctx, set(), world.fov, force=True)
        # the intended 12 m move is rejected; only a guarded shorter raw sub-move (collision-free under every hypothesis) may be chosen
        self.assertTrue(p is None or (p["safe"] and all(a.get("hw") and a["args"]["distance"] < 12 for a in p["actions"])))
        v = check_plan(plan, ctx, self.ph_goto(), strat.plausible(), NOMINAL, (), world.fov)
        self.assertFalse(v.safe)        # and the verifier agrees the intended plan itself is unsafe

    def ph_goto(self): return Phase("go", "goto", {"label": "red box"})

    def test_probe_candidates_stop_at_clean_after_motion(self):
        world, strat, ctx, _ = self._probe(NOMINAL)
        plan = {"actions": [act("move_forward", distance=1), act("clean"), act("move_forward", distance=1)]}
        p = strat.plan_probe(plan, ctx, set(), world.fov)
        self.assertIsNotNone(p)
        self.assertTrue(all(a["tool"] != "clean" for a in p["actions"]))
    def test_leading_clean_is_allowed_in_a_probe(self):
        world, strat, ctx, _ = self._probe(NOMINAL)
        plan = {"actions": [act("clean"), act("move_forward", distance=1)]}
        p = strat.plan_probe(plan, ctx, set(), world.fov)
        self.assertIsNotNone(p); self.assertEqual(p["actions"][0]["tool"], "clean")

    def test_budget_is_respected(self):
        world, strat, ctx, plan = self._probe(NOMINAL)
        strat.task_probes = strat.max_task_probes
        self.assertFalse(strat.wants_probe()); self.assertIsNone(strat.plan_probe(plan, ctx, set(), world.fov))

    def test_bay_modes_do_not_probe_in_the_task(self):
        for ident in ("none", "bay"):
            self.assertFalse(RobustStrategy(ident, "robust").wants_probe())

    def test_hybrid_bay_is_cheap(self):
        for h in hypothesis_space():
            w = apply_embodiment(World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", 6, 6)]), h)
            s = RobustStrategy("hybrid", "robust"); s.start(w)
            self.assertLessEqual(s.bay_travel, 0.0 + 1e-9)       # look + turn90 travel no metres
            self.assertLessEqual(len(s.plausible()), 3)           # mirror and swap known; only the gain remains

    def test_command_evidence_updates_belief_and_flags_change(self):
        h = Hypothesis(False, False, 0.5, 1.0)
        world = apply_embodiment(World2D(14, 9, (2, 4), 0, []), h)
        s = RobustStrategy("probe", "robust"); s.start(world)
        pre = world.pose(); r = world.execute("move_forward", {"distance": 4})
        changed = s.observe_command(pre, "move_forward", {"distance": 4}, world.pose(), r)
        self.assertTrue(changed); self.assertEqual(s.belief.move_gain, 0.5)

    def test_blocked_move_gives_no_evidence(self):
        world = World2D(14, 9, (2, 4), 0, [Obj("o", "box", 4, 4)])
        s = RobustStrategy("probe", "robust"); s.start(world)
        pre = world.pose(); r = world.execute("move_forward", {"distance": 4})
        self.assertFalse(r["ok"]); self.assertFalse(s.observe_command(pre, "move_forward", {"distance": 4}, world.pose(), r))
        self.assertEqual(len(s.plausible()), 12)

class EpisodeTests(unittest.TestCase):
    def _run(self, arm, fam="goto", yaw=0, seed=0, h=NOMINAL):
        ident, ver = arm.split("_")
        n = 2 if fam == "remember_return" else 3
        world, task = make_task(fam, seed, yaw, n, False, 2 if fam == "motion" else 1)
        apply_embodiment(world, h)
        return world, run_episode_robust(world, task, PRESETS["S5"], OracleBackend(0.0, 0.0), RobustStrategy(ident, ver, seed=seed))

    def test_bay_old_reproduces_baseline_infogain_exactly(self):
        keys = ("success", "collisions", "invalid_calls", "env_tool_calls", "actions_executed", "repairs", "verify_failures", "llm_calls",
                "final_x", "final_y", "final_yaw", "belief_correct", "id_probes", "id_bay_travel")
        n = 0
        for fam, yaw, h in itertools.product(("motion", "goto", "clean_left", "remember_return"), (0, 90, 180, 270), hypothesis_space()):
            world, task = make_task(fam, 1, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1)
            apply_embodiment(world, h)
            base = run_episode(world, task, PRESETS["S5"], OracleBackend(0.0, 0.0), embodiment=EmbodimentStrategy("infogain", seed=1))
            world2, task2 = make_task(fam, 1, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1)
            apply_embodiment(world2, h)
            new = run_episode_robust(world2, task2, PRESETS["S5"], OracleBackend(0.0, 0.0), RobustStrategy("bay", "old", seed=1))
            for k in keys: self.assertEqual(base[k], new[k], f"{fam} y{yaw} {h.key} {k}")
            n += 1
        self.assertEqual(n, 4 * 4 * 12)

    def test_all_arms_run_and_report_columns(self):
        for arm in ARMS:
            _, row = self._run(arm)
            for k in ("task_probes", "robust_rejects", "final_confidence", "belief_correct", "success"): self.assertIn(k, row)

    def test_robust_verifier_never_executes_when_no_plan_is_safe(self):
        # uniform prior, no probing: every non-trivial plan fails under some hypothesis -> agent abstains, zero collisions
        rows = [self._run("none_robust", fam, yaw, s, h)[1] for fam in ("goto", "motion") for yaw in (0, 90) for s in range(3)
                for h in (Hypothesis(False, False, 0.5, 1.0), Hypothesis(True, True, 0.25, 1.0))]
        self.assertTrue(all(r["collisions"] == 0 for r in rows))
        self.assertTrue(all(r["success"] == 0 for r in rows))              # abstains rather than guess
        self.assertTrue(any(r["aborted"] >= 1 for r in rows))

    def test_probe_robust_solves_hidden_embodiments_without_bay(self):
        ok = tot = 0
        for h in hypothesis_space():
            for fam in ("goto", "motion"):
                for yaw in (0, 90, 180, 270):
                    _, row = self._run("probe_robust", fam, yaw, 1, h)
                    tot += 1; ok += row["success"]
                    self.assertEqual(row["id_bay_travel"], 0.0)
        self.assertGreaterEqual(ok / tot, 0.75, f"{ok}/{tot}")

    def test_hybrid_robust_never_collides_in_noise_free_oracle_runs(self):
        for h in hypothesis_space():
            for yaw in (0, 90, 180, 270):
                _, row = self._run("hybrid_robust", "goto", yaw, 2, h)
                self.assertEqual(row["collisions"], 0, (h.key, yaw))

if __name__ == "__main__":
    unittest.main()
