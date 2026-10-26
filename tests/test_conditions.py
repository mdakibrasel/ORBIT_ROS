import unittest
from orbit_ros.runner import TrialRunner
from orbit_ros.planner import MockPlanner

class ConditionExecutionTests(unittest.TestCase):
    def setUp(self):
        self.runner = TrialRunner(MockPlanner())

    def test_c1_and_c2_do_not_retain_plan_actions(self):
        c1, e1 = self.runner.run("C1", "T1", 1001)
        c2, e2 = self.runner.run("C2", "T1", 1001)
        self.assertEqual(c1["planned_actions"], 0)
        self.assertEqual(c2["planned_actions"], 0)
        self.assertTrue(any(e["event"] == "action_selected_without_persistent_plan" for e in e1))
        self.assertTrue(any(e["event"] == "transient_plan_reconstructed" for e in e2))

    def test_c3_can_deviate_from_plan_without_gate(self):
        _, events = self.runner.run("C3", "T1", 1001)
        self.assertTrue(any(e["event"] == "unconstrained_action_selected" and '"deviated": true' in e["details"] for e in events))

    def test_c4_has_execution_gate(self):
        _, events = self.runner.run("C4", "T1", 1000)
        self.assertTrue(any(e["event"] == "execution_gate_block" for e in events))

    def test_c3_and_c4_log_different_control_policies(self):
        _, e3 = self.runner.run("C3", "T1", 1001)
        _, e4 = self.runner.run("C4", "T1", 1001)
        self.assertTrue(any(e["event"] == "unconstrained_action_selected" for e in e3))
        self.assertTrue(any(e["event"] == "planned_action_selected" for e in e4))

if __name__ == "__main__": unittest.main()
