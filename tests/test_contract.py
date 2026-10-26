import unittest
from orbit_ros.models import Action, Plan
from orbit_ros.contract import ContractVerifier

class ContractTests(unittest.TestCase):
    def setUp(self): self.v = ContractVerifier()
    def test_unknown_tool_rejected(self):
        errors = self.v.verify_plan(Plan("x", [Action("go_to", {})]))
        self.assertTrue(any("unknown tool" in e for e in errors))
    def test_missing_goal_dependency_rejected(self):
        errors = self.v.verify_plan(Plan("x", [Action("navigate_to", {"waypoint": 1})]))
        self.assertTrue(any("without prior set_goal" in e for e in errors))
    def test_valid_pair_accepted(self):
        errors = self.v.verify_plan(Plan("x", [Action("set_goal", {"goal": "A"}), Action("navigate_to", {"waypoint": 1})]))
        self.assertEqual(errors, [])

if __name__ == "__main__": unittest.main()
