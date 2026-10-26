"""Scenario sets A-D (v0.12.1): membership, instance counts, D's realism, sequence level 4, report section."""
import contextlib, io, os, tempfile, unittest
from orbit_ros.scenario_sets import SCENARIOS, instances, scenarios_of, realism_for
from orbit_ros.tasks2d import make_task, movable_ids
from orbit_ros.world2d import World2D, Realism
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend

class ScenarioTests(unittest.TestCase):
    def test_instances_are_unique_and_membership_is_consistent(self):
        inst = instances(list(SCENARIOS), 2)
        self.assertEqual(len(inst), len(set(inst)))
        for i in inst: self.assertTrue(scenarios_of(i[0], *i[1:], list(SCENARIOS), 2))
        a = [i for i in inst if "A" in scenarios_of(i[0], *i[1:], ["A"], 2)]
        self.assertEqual(len(a), 2 * 4 * 2)                                      # 2 object counts x 4 x 2 seeds, heading 0 only
        self.assertTrue(all(i[2] == 0 for i in a))
        b_yaws = {i[2] for i in inst if "B" in scenarios_of(i[0], *i[1:], ["B"], 2)}
        self.assertEqual(b_yaws, {0, 90, 180, 270})
        self.assertEqual(sum(1 for i in inst if i[0] == "goto" and i[2] == 0 and "B" in scenarios_of(i[0], *i[1:], ["B"], 2)), 2 * 2)
    def test_only_D_gets_faults_and_moving_obstacles(self):
        self.assertEqual(realism_for("goto", 0, 0, 3, False, 1, list(SCENARIOS), 1), {})
        self.assertEqual(realism_for("sequence", 0, 90, 5, False, 4, list(SCENARIOS), 1), SCENARIOS["D"]["realism"])
    def test_sequence_level4_has_three_targets_and_a_return_and_is_solvable(self):
        w, t = make_task("sequence", 0, 90, 5, False, 4)
        self.assertEqual(len(t.phases), 4); self.assertEqual(len(t.expected["target_ids"]), 3)
        self.assertEqual(run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0))["success"], 1)
    def test_motor_fault_is_transient_and_counted(self):
        w = World2D(9, 3, (0, 1), 0, []); w.set_realism(Realism(fail_moves=(2,), seed="f"))
        self.assertTrue(w.execute("move_forward", {"distance": 1})["ok"])
        r = w.execute("move_forward", {"distance": 2}); self.assertFalse(r["ok"]); self.assertEqual((w.x, w.injected_failures), (1, 1))
        self.assertTrue(w.execute("move_forward", {"distance": 2})["ok"]); self.assertEqual(w.x, 3)
        self.assertEqual(w.collisions, 0)                                          # a motor fault is not a collision
    def test_repairing_agent_recovers_from_injected_faults(self):
        w, t = make_task("sequence", 1, 0, 5, False, 4); w.set_realism(Realism(fail_moves=(2, 5), seed="x")); w.movable = movable_ids(w, t)
        r = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0))
        self.assertGreater(r["injected_failures"], 0); self.assertEqual(r["success"], 1)
    def test_runner_tags_rows_and_report_has_the_section(self):
        import run_spatial, analyze_spatial, csv
        with tempfile.TemporaryDirectory() as d:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                run_spatial.main(["--scenario", "A", "D", "--seeds", "1", "--configs", "S5", "C1", "--out", d]); analyze_spatial.main(d)
            rows = list(csv.DictReader(open(os.path.join(d, "trial_results.csv"))))
            self.assertTrue(all(r["scenarios"] for r in rows)); self.assertIn("injected_failures", rows[-1])
            txt = open(os.path.join(d, "REPORT.md"), encoding="utf-8").read()
            self.assertIn("## 6. Scenarios A-D", txt); self.assertIn("D: recovery", txt)

if __name__ == "__main__": unittest.main()
