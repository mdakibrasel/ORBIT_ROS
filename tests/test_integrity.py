"""v0.8.1 tests: belief-integrity fixes (orbit_ros/integrity.py), fail-closed step guard, runner refactor, and reproduction of v0.8.0.

Seeds used here are dev seeds (0-4) or the already-used v0.7/v0.8.0 seeds 100-109; never the v0.8.1 held-out seeds 200-209.
"""
import csv, itertools, json, os, unittest
from orbit_ros.embodiment import EmbodimentIdentifier, Hypothesis, make_probe
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment, _new_identifier
from orbit_ros.world2d import World2D, Obj
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import PRESETS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.robust import RobustStrategy
from orbit_ros.voi import VoiStrategy
from orbit_ros.agent_robust import run_episode_robust
from orbit_ros.agent_recovery import run_episode_v08
from orbit_ros.recovery import RecoveryConfig, StepGuard
from orbit_ros.integrity import (IntegrityConfig, SafeIdentifier, SafeRobustStrategy, SafeVoiStrategy, make_safe, integrity_row_of, COLLAPSE_MSG)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
H = {h.key: h for h in hypothesis_space()}
OFF = IntegrityConfig(False, False, False)

def probe(outcomes): return make_probe("p", True, outcomes)

def episode(fam, yaw, seed, hkey, noise, safe=None, rcfg=None, drift=None, old=False):
    """safe=None -> v0.8.0 strategy; safe=IntegrityConfig -> v0.8.1 strategy."""
    world, task = make_task(fam, seed, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1)
    apply_embodiment(world, H[hkey])
    if drift is not None: world.drift_schedule.append((drift, {"move_gain": 0.5 if world.move_gain != 0.5 else 1.0}))
    kw = dict(eps=noise, noise_p=noise, seed=seed)
    if old: st = (RobustStrategy("hybrid", "old", **kw) if safe is None else SafeRobustStrategy("hybrid", "old", integrity=safe, **kw))
    else: st = (VoiStrategy("hybrid", voi=False, **kw) if safe is None else SafeVoiStrategy("hybrid", voi=False, integrity=safe, **kw))
    be = OracleBackend(0, 0)
    row = run_episode_robust(world, task, PRESETS["S5"], be, st) if old else run_episode_v08(world, task, PRESETS["S5"], be, st, rcfg or RecoveryConfig())
    row.update(integrity_row_of(st)); return row

# ------------------------------------------------------------------ identifier
class SafeIdentifierTests(unittest.TestCase):
    def two_step(self, cls_safe):
        ident = _new_identifier(10 ** 6, 0.0, 0.0, 0.95)
        if cls_safe: make_safe(ident)
        keys = [h.key for h in ident.hypotheses]; a, b = keys[0], keys[1]
        ident.update(probe({k: ("A" if k == a else "x") for k in keys}), "A")           # eliminates everything but a
        ident.update(probe({k: ("B" if k == b else "y") for k in keys}), "B")           # only b (already at weight 0) matches
        return ident, a, b

    def test_base_identifier_collapses_to_all_zero(self):
        """Documents the v0.5-v0.8.0 defect: a hard update matching only eliminated hypotheses zeroes every weight."""
        ident, _, _ = self.two_step(False)
        self.assertEqual(max(ident.weights.values()), 0.0)

    def test_safe_identifier_keeps_current_observation_and_renormalises(self):
        ident, a, b = self.two_step(True)
        self.assertAlmostEqual(sum(ident.weights.values()), 1.0)
        self.assertEqual(ident.weights[b], 1.0)                                       # the only hypothesis consistent with the latest reading
        self.assertEqual(ident.collapse_resets, 1); self.assertTrue(ident.history[-1]["collapse_reset"])

    def test_reset_prefers_hypotheses_with_fewest_past_contradictions(self):
        ident = make_safe(_new_identifier(10 ** 6, 0.0, 0.0, 0.95)); keys = [h.key for h in ident.hypotheses]
        g, b, x = keys[2], keys[3], keys[7]
        ident.update(probe({k: ("M" if k == g else "n") for k in keys}), "M")             # only g survives; b and x contradict this reading
        ident.update(probe({k: ("P" if k in (g, b) else "q") for k in keys}), "P")        # consistent with g -> ordinary hard update
        self.assertEqual(ident.weights[g], 1.0); self.assertEqual(ident.collapse_resets, 0)
        ident.update(probe({k: ("R" if k in (b, x) else "s") for k in keys}), "R")        # only b and x match, both at weight 0 -> reset
        self.assertEqual(ident.collapse_resets, 1); self.assertEqual(ident.weights[g], 0.0)   # current reading stays hard evidence
        self.assertGreater(ident.weights[b], ident.weights[x])                             # b contradicted 1 earlier reading, x contradicted 2
        self.assertEqual(max(ident.weights, key=ident.weights.get), b)
        self.assertAlmostEqual(sum(ident.weights.values()), 1.0)

    def test_identical_to_base_when_no_collapse_occurs(self):
        import random
        for eps in (0.0, 0.1):
            rng = random.Random(7); base = _new_identifier(10 ** 6, eps, 0.0, 0.95); safe = make_safe(_new_identifier(10 ** 6, eps, 0.0, 0.95))
            keys = [h.key for h in base.hypotheses]; truth = keys[5]
            for _ in range(6):
                out = {k: rng.choice("abc") for k in keys}; out[truth] = "a"
                base.update(probe(out), "a"); safe.update(probe(out), "a")
            self.assertEqual(base.weights, safe.weights); self.assertEqual(safe.collapse_resets, 0)

    def test_make_safe_refuses_used_identifier_and_bad_eps(self):
        ident = _new_identifier(10 ** 6, 0.0, 0.0, 0.95); keys = [h.key for h in ident.hypotheses]
        ident.update(probe({k: "a" for k in keys}), "a")
        with self.assertRaises(ValueError): make_safe(ident)
        with self.assertRaises(ValueError): make_safe(_new_identifier(10 ** 6, 0.0, 0.0, 0.95), eps_history=0.0)

# ------------------------------------------------------------------ fail-closed verification
class FailClosedTests(unittest.TestCase):
    def collapsed(self, cls, **kw):
        world = apply_embodiment(World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", 9, 4)]), Hypothesis(False, False, 1.0, 1.0))
        st = cls("hybrid", voi=False, **kw); st.start(world)
        st.ident.weights = {k: 0.0 for k in st.ident.weights}                          # the state the defect leaves behind
        return world, st, {"pose": world.pose(), "raw": world.describe_view(), "use_memory": False, "landmarks": {}}

    def test_base_verifiers_accept_vacuously(self):
        """Documents the defect: with no plausible hypothesis the strict and the risk verifier both accept."""
        for kw in ({}, {"tau": 0.0}):
            world, st, ctx = self.collapsed(VoiStrategy, **kw)
            self.assertTrue(st.check({"actions": [{"tool": "move_forward", "args": {"distance": 12}}]}, ctx, None, set(), world.fov).safe)

    def test_safe_verifiers_reject(self):
        for kw in ({}, {"tau": 0.0}, {"tau": 0.5}):
            world, st, ctx = self.collapsed(SafeVoiStrategy, **kw)
            v = st.check({"actions": [{"tool": "move_forward", "args": {"distance": 1}}]}, ctx, None, set(), world.fov)
            self.assertFalse(v.safe); self.assertEqual(v.errors, [COLLAPSE_MSG]); self.assertEqual(st.fail_closed_rejects, 1)
            self.assertEqual(st.diag_row()["last_reject_reason"], "collapsed")

    def test_step_guard_fails_closed_without_hypotheses(self):
        world, st, ctx = self.collapsed(VoiStrategy)
        g = StepGuard(st, ctx, set(), world.fov, 0.0)
        self.assertEqual(g.sims, {}); a, risk, mode = g.fit(world.pose(), {"tool": "move_forward", "args": {"distance": 2}})
        self.assertEqual(mode, "unsafe"); self.assertIsNone(a)

# ------------------------------------------------------------------ episodes
class EpisodeTests(unittest.TestCase):
    CASE = ("clean_left", 90, 100, "mirror=0;swap=1;mg=1;tg=1")      # v0.7 test episode whose posterior collapses after a blocked probe

    def test_blocked_probe_collapses_v080_and_is_censored_in_v081(self):
        f, y, s, h = self.CASE
        base = episode(f, y, s, h, 0.0)
        self.assertEqual(base["posterior_collapsed"], 1); self.assertEqual(base["belief_correct"], 0)
        fixed = episode(f, y, s, h, 0.0, safe=IntegrityConfig())
        self.assertEqual(fixed["posterior_collapsed"], 0); self.assertEqual(fixed["belief_correct"], 1)
        self.assertGreaterEqual(fixed["task_probes_censored"], 1); self.assertLess(fixed["collisions"], base["collisions"])
        nc = episode(f, y, s, h, 0.0, safe=IntegrityConfig(False, True, False))     # no_collapse alone also recovers the belief
        self.assertEqual(nc["posterior_collapsed"], 0); self.assertGreaterEqual(nc["collapse_resets"], 1); self.assertEqual(nc["belief_correct"], 1)

    def test_old_verifier_arm_is_fixed_too(self):
        f, y, s, h = self.CASE
        self.assertEqual(episode(f, y, s, h, 0.0, old=True)["posterior_collapsed"], 1)
        self.assertEqual(episode(f, y, s, h, 0.0, old=True, safe=IntegrityConfig())["posterior_collapsed"], 0)

    def test_drift_at_noise_zero_no_longer_collapses(self):
        base = episode("clean_left", 0, 3, "mirror=0;swap=0;mg=1;tg=1", 0.0, drift=12)
        fixed = episode("clean_left", 0, 3, "mirror=0;swap=0;mg=1;tg=1", 0.0, drift=12, safe=IntegrityConfig())
        self.assertEqual(base["posterior_collapsed"], 1)
        self.assertEqual(fixed["posterior_collapsed"], 0); self.assertGreaterEqual(fixed["collapse_resets"], 1); self.assertEqual(fixed["success"], 1)

    def test_all_fixes_off_reproduces_v080_strategies(self):
        cols = ("success", "collisions", "aborted", "task_probes", "env_tool_calls", "llm_calls", "final_x", "final_y", "actions_executed", "repairs", "belief_correct")
        for fam, yaw, h, noise in itertools.product(("goto", "clean_left", "remember_return", "motion"), (0, 270), H, (0.0, 0.1)):
            for old, rc in ((True, None), (False, RecoveryConfig()), (False, RecoveryConfig(enabled=True))):
                a = episode(fam, yaw, 4, h, noise, old=old, rcfg=rc); b = episode(fam, yaw, 4, h, noise, safe=OFF, old=old, rcfg=rc)
                for k in cols: self.assertEqual(a[k], b[k], (fam, yaw, h, noise, old, k))

    def test_no_collapse_never_triggers_with_soft_updates(self):
        for fam, yaw, h in itertools.product(("goto", "clean_left"), (0, 180), H):
            r = episode(fam, yaw, 2, h, 0.1, safe=IntegrityConfig(), rcfg=RecoveryConfig(enabled=True))
            self.assertEqual(r["collapse_resets"], 0); self.assertEqual(r["posterior_collapsed"], 0)

# ------------------------------------------------------------------ runner
class RunnerTests(unittest.TestCase):
    def cfg(self, name="v0_8.json"):
        with open(os.path.join(ROOT, "configs", name)) as f: return json.load(f)

    def test_v080_config_yields_the_v080_arm_list(self):
        import run_v08
        A = run_v08.arms(self.cfg()); r = self.cfg()["recovery"]
        self.assertEqual([a["arm"] for a in A], ["hybrid_old", "hybrid_robust", "hybrid_robust+monitor", "hybrid_robust+recovery", "hybrid_robust+rec_reprobe_only",
                                                 "hybrid_robust+rec_guarded_only", "hybrid_robust_voi", "hybrid_robust_voi+recovery", "hybrid_risk", "hybrid_risk+recovery"])
        rc = {a["arm"]: a["rcfg"] for a in A}
        self.assertIsNone(rc["hybrid_old"]); self.assertEqual(rc["hybrid_robust"], RecoveryConfig()); self.assertEqual(rc["hybrid_robust+monitor"], RecoveryConfig(monitor=True))
        self.assertEqual(rc["hybrid_robust+recovery"], RecoveryConfig(enabled=True, **r))
        self.assertEqual(rc["hybrid_robust+rec_reprobe_only"], RecoveryConfig(enabled=True, **dict(r, guarded=False)))
        self.assertEqual(rc["hybrid_robust+rec_guarded_only"], RecoveryConfig(enabled=True, **dict(r, extra_probes=0)))
        self.assertFalse(any(a["integrity"] for a in A))

    def test_v081_config_is_consistent_and_uses_fresh_seeds(self):
        import run_v08
        c, c7, c8 = self.cfg("v0_8_1.json"), self.cfg("v0_7.json"), self.cfg("v0_8.json")
        names = {a["arm"] for a in run_v08.arms(c)}
        for A, B in c["primary_comparisons"]: self.assertIn(A, names); self.assertIn(B, names)
        self.assertIn(c["sensitivity_arm"], names)
        held = set(c["test_seeds"])
        self.assertFalse(held & set(c["dev_seeds"])); self.assertFalse(held & set(c7["test_seeds"])); self.assertFalse(held & set(c8["test_seeds"]))

    def test_parallel_cells_equal_sequential(self):
        import run_v08
        cfg = dict(self.cfg(), families=["goto", "motion"], yaws=[0, 90]); A = {a["arm"]: a for a in run_v08.arms(cfg)}
        jobs = [(A["hybrid_old"], cfg["conditions"][1], [0, 1]), (A["hybrid_robust+recovery"], cfg["conditions"][2], [0, 1])]
        seq, par = run_v08.run_cells(cfg, jobs, 1), run_v08.run_cells(cfg, jobs, 2)
        strip = lambda rows: [{k: v for k, v in r.items() if k != "elapsed_s"} for r in rows]
        self.assertEqual(strip(seq), strip(par))

    def test_parse_seeds(self):
        import run_v08
        self.assertEqual(run_v08.parse_seeds("100-103,7"), [100, 101, 102, 103, 7])

    def test_shipped_v080_quick_results_are_reproduced(self):
        """Re-run one task cell per condition and arm and compare with results_v08_quick/test_results.csv (v0.8.0 as shipped)."""
        import run_v08
        path = os.path.join(ROOT, "results_v08_quick", "test_results.csv")
        if not os.path.exists(path): self.skipTest("results_v08_quick not present")
        with open(path, newline="") as f:
            shipped = {(r["arm"], r["condition"], r["task_id"], r["true_embodiment"]): r for r in csv.DictReader(f) if r["seed"] == "101" and r["yaw"] == "90"}
        cfg = dict(self.cfg(), families=["clean_left", "goto"], yaws=[90])
        cols = ("success", "collisions", "aborted", "actions_executed", "env_tool_calls", "llm_calls", "final_x", "final_y", "task_probes", "repairs", "monitor_trips", "rec_entries", "guarded_steps")
        n = 0
        for arm in run_v08.arms(cfg):
            for cond in cfg["conditions"]:
                for r in run_v08.run_cell(cfg, arm, cond, [101]):
                    s = shipped[(r["arm"], r["condition"], r["task_id"], r["true_embodiment"])]
                    for k in cols: self.assertEqual(str(r[k]), s[k], (r["arm"], r["condition"], r["task_id"], r["true_embodiment"], k))
                    n += 1
        self.assertEqual(n, 10 * 3 * 2 * 12)

if __name__ == "__main__": unittest.main()
