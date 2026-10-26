import random
from dataclasses import dataclass
from .models import Action
from .scenarios import Scenario

@dataclass
class ToolResult:
    ok: bool
    message: str
    value: object = None

class MockRobotEnvironment:
    """Deterministic per-seed mock; not a physics simulator."""
    def __init__(self, scenario: Scenario, seed: int):
        self.scenario = scenario
        self.rng = random.Random(seed)
        self.goal_set = False
        self.stopped = False
        self.completed_waypoints = 0
        self.tool_calls = 0
        self.invalid_calls = 0
        self.safety_violations = 0
        self.disturbances = 0

    def execute(self, action: Action) -> ToolResult:
        self.tool_calls += 1
        if action.tool not in {"get_goal_odometry", "set_goal", "describe_image", "navigate_to", "stop_robot"}:
            self.invalid_calls += 1
            return ToolResult(False, f"Unknown tool: {action.tool}")
        if action.tool == "set_goal":
            if "goal" not in action.args:
                self.invalid_calls += 1
                return ToolResult(False, "set_goal requires goal")
            self.goal_set = True
            return ToolResult(True, "Goal stored", action.args["goal"])
        if action.tool == "get_goal_odometry":
            return ToolResult(True, "Odometry available", {"x": self.completed_waypoints, "y": 0})
        if action.tool == "describe_image":
            return ToolResult(True, "Mock scene description", {"visible_objects": self.scenario.horizon})
        if action.tool == "stop_robot":
            self.stopped = True
            return ToolResult(True, "Robot stopped")
        if action.tool == "navigate_to":
            if not self.goal_set:
                return ToolResult(False, "Precondition failed: goal has not been set")
            # Safety violations are blocked by the runtime guard, not by this mock tool.
            if self.rng.random() < self.scenario.disturbance_rate:
                self.disturbances += 1
                return ToolResult(False, "Environment changed / navigation action failed")
            self.completed_waypoints += 1
            return ToolResult(True, "Waypoint reached", self.completed_waypoints)
        return ToolResult(False, "Unhandled tool")
