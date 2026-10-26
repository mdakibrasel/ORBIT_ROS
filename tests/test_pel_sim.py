"""ORBIT-PEL M5: the PEL path on simulator backends. World2D and FakeSim (grid) must give identical episodes; FakeSim continuous runs
the same path on wheel physics. Webots and ROS 2 are not installed in the test environment: choosing them must fail loudly, never fall
back to World2D."""
import unittest
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.sim_select import in_simulator
from orbit_ros.webots_adapter import WebotsWorld2D
from tools.sim_agreement import episode, compare

CASES = [("goto", 0, 90, 3, False, 1), ("clean_left", 0, 180, 3, False, 1), ("sequence", 0, 0, 3, False, 1), ("prohibit", 0, 270, 3, False, 1),
         ("report", 0, 0, 1, False, 1), ("conditional", 1, 90, 3, False, 1)]

class SimSelectTests(unittest.TestCase):
    def test_world2d_is_the_unchanged_world(self):
        w, _ = make_task("goto", 0, 0, 3, False, 1); self.assertIs(in_simulator(w), w)
        with self.assertRaises(ValueError): in_simulator(w, "world2d", "continuous")
    def test_fake_grid_agrees_with_world2d_for_pel_and_c4(self):
        for inst in CASES:
            for cfg in ("PEL", "PEL-guard", "C4"):
                a, b = episode(inst, PRESETS[cfg], "world2d", "grid"), episode(inst, PRESETS[cfg], "fake", "grid")
                self.assertEqual(compare(a, b), [], (inst, cfg)); self.assertEqual(b["_diag"]["sync_error_max"], 0.0)
    def test_pel_runs_on_continuous_motion(self):
        for inst in CASES[:3]:
            r = episode(inst, PRESETS["PEL"], "fake", "continuous")
            self.assertIn(r["success"], (0, 1)); self.assertTrue(r["pel_terminal"])
            self.assertLess(r["_diag"]["odom_error_max"], 0.1)
    def test_missing_simulators_fail_loudly(self):
        from orbit_ros.webots_env import find_webots_home
        # With Webots installed, the webots case would connect to it for real: Webots' controller library waits 50 s for a running
        # Webots and then ends the whole test process (seen on Windows in v0.16.1). Only the missing-simulator case is tested here.
        for sim in ("ros2",) if find_webots_home() else ("webots", "ros2"):
            w, _ = make_task("goto", 0, 0, 3, False, 1)
            with self.assertRaises(Exception): in_simulator(w, sim, "grid", ros2_config="configs/ros2_jazzy_tb3.json")

if __name__ == "__main__": unittest.main()
