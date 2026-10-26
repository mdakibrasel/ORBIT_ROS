"""v0.13: outcome codes, simulator safety gate, fail-closed observation, deadline, action horizon, C5 feedback re-planning, oscillation
metric, run manifest, JSON configs, held-out seeds, ground-truth leak checks, effect sizes, contract layer and conformance suite."""
import csv, io, json, os, re, tempfile, unittest
from contextlib import redirect_stdout
from orbit_ros.tasks2d import make_task, ALL_FAMILIES
from orbit_ros.world2d import World2D, Obj, Realism
from orbit_ros.agent2d import run_episode, PRESETS, C_LADDER, OUTCOME_CODES, outcome_code
from orbit_ros.backends2d import OracleBackend, LLMBackend
from orbit_ros.ensemble import Deadline
from orbit_ros.llm import ScriptedClient
from orbit_ros.manifest import code_hash, prompt_hash, run_manifest
from orbit_ros.contract_api import ScenarioSpec, world2d_adapter, fakesim_adapter, status_of, STATUSES
from orbit_ros.conformance import run_suite, report_md
from orbit_ros.ros2_adapter import load_ros2_config, ROS2_CONFIG_KEYS
from compare_runs import paired_diff_ci
import run_spatial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def quiet(fn, *a):
    with redirect_stdout(io.StringIO()): return fn(*a)

class OutcomeCodeTests(unittest.TestCase):
    def test_first_match_wins(self):
        self.assertEqual(outcome_code({"success": 1, "abstained": 1}), "success")
        self.assertEqual(outcome_code({"abstained": 1, "safety_violations": 2}), "refused")
        self.assertEqual(outcome_code({"observation_failure": 1, "invalid_calls": 1}), "observation_failure")
        self.assertEqual(outcome_code({"env_tool_calls": 300, "tool_call_cap": 300, "safety_violations": 1}), "step_budget_exhausted")
        self.assertEqual(outcome_code({"safety_violations": 1, "invalid_calls": 3}), "unsafe_failure")
        self.assertEqual(outcome_code({"invalid_calls": 1}), "invalid_call_failure")
        self.assertEqual(outcome_code({}), "task_failure")
        self.assertEqual(len(set(OUTCOME_CODES)), len(OUTCOME_CODES))
    def test_every_episode_row_has_a_code(self):
        for fam in ("goto", "prohibit", "impossible"):
            w, t = make_task(fam, 0, 90, 3, False, 1)
            r = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0))
            self.assertIn(r["outcome_code"], OUTCOME_CODES)
            self.assertEqual(r["outcome_code"] == "success", bool(r["success"]))

class SafetyGateTests(unittest.TestCase):
    def test_gate_stops_before_the_obstacle_without_a_collision(self):
        for gate in (False, True):
            w = World2D(7, 3, (0, 1), 0, [Obj("o0", "red box", 3, 1)]); w.safety_gate = gate
            r = w.execute("move_forward", {"distance": 5})
            self.assertFalse(r["ok"]); self.assertEqual(w.x, 2)
            if gate:
                self.assertTrue(r["message"].startswith("safety stop")); self.assertEqual((w.collisions, w.safety_rejections), (0, 1))
                self.assertEqual(status_of(r), "safety_stop")
            else:
                self.assertEqual(status_of(r), "blocked")
    def test_gate_is_independent_of_the_planner(self):
        """The gate lives in the simulator: the same scripted (bad) model collides without it and is stopped with it."""
        out = {}
        for gate in (False, True):
            w = World2D(7, 3, (0, 1), 0, [Obj("o0", "red box", 3, 1)]); w.safety_gate = gate
            w.execute("move_forward", {"distance": 9}); out[gate] = (w.collisions, w.safety_rejections)
        self.assertGreater(out[False][0], 0); self.assertEqual(out[True], (0, 1))

class FailClosedTests(unittest.TestCase):
    def test_stale_camera_stops_the_robot_and_abstains(self):
        w, t = make_task("goto", 0, 90, 3, False, 1); w.set_realism(Realism(stale_p=1.0, seed="stale"))
        r = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0), fail_closed=True)
        self.assertEqual(r["outcome_code"], "observation_failure"); self.assertGreater(r["stale_retries"], 0)
        self.assertEqual(r["success"], 0)
    def test_stepwise_configs_end_cleanly_after_a_fail_closed_stop(self):
        """Regression: after the stop, the robot's tools refuse; step-wise configs must end the episode, not crash on an empty view."""
        stops = 0
        for cfg in ("C1", "C3", "C4", "C5"):
            for s in range(4):
                w, t = make_task("motion", s, 0, 3, False, 3); w.set_realism(Realism(stale_p=0.3, seed=t.task_id))
                r = run_episode(w, t, PRESETS[cfg], OracleBackend(), fail_closed=True); stops += r["observation_failure"]
                if r["observation_failure"]: self.assertIn(r["outcome_code"], ("success", "observation_failure"))   # done before the stop counts
        self.assertGreater(stops, 0)
    def test_gated_episodes_have_no_collisions(self):
        for cfg in ("S2", "C4"):
            for s in range(3):
                w, t = make_task("goto", s, 0, 3, False, 1); w.safety_gate = True
                self.assertEqual(run_episode(w, t, PRESETS[cfg], OracleBackend())["collisions"], 0)
    def test_without_fail_closed_nothing_changes(self):
        w, t = make_task("goto", 0, 90, 3, False, 1)
        a = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0))
        w, t = make_task("goto", 0, 90, 3, False, 1)
        b = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0), fail_closed=True)
        self.assertEqual((a["success"], a["actions_executed"]), (b["success"], b["actions_executed"]))

class DeadlineTests(unittest.TestCase):
    def test_no_calls_after_the_deadline(self):
        now = [0.0]
        cl = ScriptedClient(lambda p, tag: (now.__setitem__(0, now[0] + 10), '{"tool":"turn","args":{"direction":"left","degrees":90}}')[1])
        be = Deadline(LLMBackend(cl, False, False), 25, clock=lambda: now[0])
        w, t = make_task("goto", 0, 90, 3, False, 1)
        run_episode(w, t, PRESETS["C1"], be)
        self.assertTrue(be.timed_out); self.assertLessEqual(len(cl.prompts), 4)

class HorizonAndFeedbackTests(unittest.TestCase):
    def test_horizon_replans_and_keeps_success(self):
        ok0 = okh = rep = 0
        for s in range(2):
            for yaw in (0, 90, 180, 270):
                for h in (0, 4):
                    w, t = make_task("clean_left", s, yaw, 3, False, 1)
                    r = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0), horizon=h)
                    if h: okh += r["success"]; rep += r["horizon_replans"]
                    else: ok0 += r["success"]; self.assertEqual(r["horizon_replans"], 0)
        self.assertGreater(rep, 0); self.assertGreaterEqual(okh, ok0 - 1)
    def test_c5_is_registered_and_replans_on_feedback(self):
        self.assertEqual(C_LADDER[-1], "C5"); self.assertTrue(PRESETS["C5"].feedback)
        fb = 0
        for s in range(3):
            for yaw in (0, 90, 180, 270):
                w, t = make_task("goto", s, yaw, 3, False, 1); w.set_realism(Realism(fail_moves=(1, 3), seed=f"f{s}{yaw}"))
                fb += run_episode(w, t, PRESETS["C5"], OracleBackend(0.05, 0.3))["feedback_replans"]
        self.assertGreater(fb, 0)
    def test_c4_never_uses_feedback(self):
        w, t = make_task("goto", 0, 90, 3, False, 1); w.set_realism(Realism(fail_moves=(1, 3), seed="f"))
        self.assertEqual(run_episode(w, t, PRESETS["C4"], OracleBackend(0.05, 0.3))["feedback_replans"], 0)

class OscillationTests(unittest.TestCase):
    def test_backtrack_counted(self):
        w = World2D(7, 3, (1, 1), 0, [])
        w.execute("move_forward", {"distance": 1}); w.execute("turn", {"direction": "left", "degrees": 180})
        w.execute("move_forward", {"distance": 1}); self.assertEqual(w.backtracks, 1)
        w.execute("turn", {"direction": "left", "degrees": 180}); w.execute("move_forward", {"distance": 2})
        self.assertEqual(w.backtracks, 2)

class GroundTruthLeakTests(unittest.TestCase):
    """No prompt may carry what only the evaluator knows: object ids, Task.expected keys, or the true pose when odometry has drifted."""
    EVAL_KEYS = ("target_id", "alt_target_id", "condition_true", "interpretations", "final_pose", "forbidden")
    def test_prompts_carry_no_evaluator_data(self):
        reply = lambda p, tag: ('{"actions":[{"tool":"move_forward","args":{"distance":1}}]}' if tag in ("plan", "repair", "plan_step")
                                else '{"tool":"move_forward","args":{"distance":1}}')
        for fam in ALL_FAMILIES:
            for cfg in ("S0", "S2", "S5", "C1", "C4", "C5"):
                w, t = make_task(fam, 1, 180, 3, False, 1); w.set_realism(Realism(odom_p=1.0, seed=t.task_id))
                seen = []
                def fn(p, tag, w=w):
                    seen.append((p, (w.x, w.y), (w.pose()["x"], w.pose()["y"]))); return reply(p, tag)
                run_episode(w, t, PRESETS[cfg], LLMBackend(ScriptedClient(fn), PRESETS[cfg].grounded, PRESETS[cfg].memory))
                self.assertTrue(seen, f"{fam}/{cfg} made no model call")
                for p, true_xy, est_xy in seen:
                    self.assertIsNone(re.search(r"\bo\d\b", p), f"object id in {fam}/{cfg} prompt")
                    for k in self.EVAL_KEYS: self.assertNotIn(k, p, f"evaluator key {k} in {fam}/{cfg} prompt")
                    if true_xy != est_xy:
                        self.assertNotIn(f'"x": {true_xy[0]}, "y": {true_xy[1]}, "yaw_deg"', p, f"true pose in {fam}/{cfg} prompt")

class RunnerTests(unittest.TestCase):
    def run_cli(self, *argv):
        d = tempfile.mkdtemp(); quiet(run_spatial.main, list(argv) + ["--out", d])
        rows = list(csv.DictReader(open(os.path.join(d, "trial_results.csv"), encoding="utf-8")))
        return rows, json.load(open(os.path.join(d, "run_metadata.json")))
    def test_heldout_seeds_are_new_worlds(self):
        a, ma = self.run_cli("--families", "goto", "--limit", "4", "--configs", "S5")
        b, mb = self.run_cli("--families", "goto", "--limit", "4", "--configs", "S5", "--heldout")
        self.assertEqual((ma["seed_offset"], mb["seed_offset"]), (0, run_spatial.HELDOUT_SEED0))
        self.assertTrue(mb["heldout_seeds"]); self.assertFalse(set(r["task_id"] for r in a) & set(r["task_id"] for r in b))
    def test_config_file_sets_defaults_and_rejects_unknown_keys(self):
        d = tempfile.mkdtemp(); p = os.path.join(d, "c.json")
        json.dump({"_comment": "x", "families": ["goto"], "limit": 2, "configs": ["S5", "C5"], "horizon": 3, "safety_gate": True}, open(p, "w"))
        rows, meta = self.run_cli("--config", p)
        self.assertEqual((meta["horizon"], meta["safety_gate"], meta["config_file"]), (3, True, p))
        self.assertEqual({r["config"] for r in rows}, {"S5", "C5"}); self.assertTrue(all(r["outcome_code"] in OUTCOME_CODES for r in rows))
        json.dump({"no_such_option": 1}, open(p, "w"))
        with self.assertRaises(SystemExit), redirect_stdout(io.StringIO()), unittest.mock.patch("sys.stderr", io.StringIO()):
            run_spatial.main(["--config", p, "--out", d])
    def test_metadata_has_the_manifest(self):
        _, m = self.run_cli("--families", "goto", "--limit", "1", "--configs", "S0")
        for k in ("code_hash", "prompt_hash", "environment"): self.assertIn(k, m)
        self.assertEqual(m["code_hash"], code_hash()); self.assertEqual(m["prompt_hash"], prompt_hash())
    def test_shipped_configs_load(self):
        for name in ("run_pilot_groq.json", "run_closed_loop_oracle.json"):
            conf = json.load(open(os.path.join(ROOT, "configs", name)))
            self.assertNotIn("api_key", json.dumps(conf).lower())

class ManifestTests(unittest.TestCase):
    def test_hashes_are_stable_and_change_with_the_prompt(self):
        self.assertEqual(code_hash(), code_hash()); h = prompt_hash()
        from orbit_ros import backends2d
        old = backends2d.CONVENTIONS
        try:
            backends2d.CONVENTIONS = old + " "; self.assertNotEqual(prompt_hash(), h)
        finally: backends2d.CONVENTIONS = old
        self.assertEqual(prompt_hash(), h)
        self.assertIn("python", run_manifest()["environment"])

class EffectSizeTests(unittest.TestCase):
    def test_paired_difference_and_interval(self):
        d, lo, hi = paired_diff_ci([1] * 50 + [0] * 50, [1] * 50 + [0] * 50); self.assertEqual(d, 0); self.assertLess(lo, 0); self.assertGreater(hi, 0)
        d, lo, hi = paired_diff_ci([0] * 100, [1] * 40 + [0] * 60); self.assertAlmostEqual(d, 40); self.assertTrue(lo > 0 and lo < d < hi)
        d, lo, hi = paired_diff_ci([1] * 30 + [0] * 70, [0] * 100); self.assertAlmostEqual(d, -30); self.assertLess(hi, 0)

class ContractTests(unittest.TestCase):
    def test_all_local_adapters_pass_the_suite(self):
        for ad in (world2d_adapter(), fakesim_adapter("grid"), fakesim_adapter("continuous")):
            res = run_suite(ad); bad = [r for r in res if not r["passed"]]
            self.assertFalse(bad, f"{ad.name}: {bad}"); self.assertEqual(len(res), 11)
            self.assertIn("11/11 checks passed", report_md(ad, res))
    def test_suite_catches_a_broken_adapter(self):
        class Mirrored(type(world2d_adapter())):
            pass
        ad = Mirrored(lambda spec: ScenarioSpec(spec.width, spec.height, spec.start, spec.yaw, spec.objects, mirrored_camera=True).world(), "grid", "mirrored")
        res = {r["check"]: r["passed"] for r in run_suite(ad)}
        self.assertFalse(res["camera_convention"]); self.assertTrue(res["frame_units"])
    def test_statuses_and_ground_truth_split(self):
        ad = world2d_adapter(); o = ad.reset(ScenarioSpec(5, 5, (2, 2), 0, [("o0", "red box", 4, 2)]))
        self.assertTrue(all(set(d) <= {"label", "image_x", "apparent_size"} for d in o.view))
        self.assertEqual(ad.act("fly", {}).status, "invalid"); self.assertIn(ad.act("move_forward", {"distance": 3}).status, STATUSES)
        self.assertEqual(ad.state()["objects"][0]["id"], "o0")

class Ros2ConfigTests(unittest.TestCase):
    def test_shipped_config_is_valid_and_typos_fail(self):
        conf = load_ros2_config(os.path.join(ROOT, "configs", "ros2_jazzy_tb3.json"))
        self.assertTrue(set(conf) <= set(ROS2_CONFIG_KEYS)); self.assertEqual(load_ros2_config(None), {})
        d = tempfile.mkdtemp(); p = os.path.join(d, "bad.json"); json.dump({"cmd_topik": "/x"}, open(p, "w"))
        with self.assertRaises(ValueError): load_ros2_config(p)

import unittest.mock  # noqa: E402  (used by RunnerTests)

if __name__ == "__main__": unittest.main()
