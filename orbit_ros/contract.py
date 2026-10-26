from .models import Action, Plan, ALLOWED_TOOLS

REQUIRED_ARGS = {
    "get_goal_odometry": set(),
    "set_goal": {"goal"},
    "describe_image": set(),
    "navigate_to": {"waypoint"},
    "stop_robot": set(),
}

class ContractVerifier:
    """Deterministic syntax, tool, argument, dependency and safety contract."""
    def verify_plan(self, plan: Plan) -> list[str]:
        errors = []
        goal_ready = False
        for index, action in enumerate(plan.actions):
            if action.tool not in ALLOWED_TOOLS:
                errors.append(f"action[{index}]: unknown tool {action.tool}")
                continue
            missing = REQUIRED_ARGS[action.tool] - set(action.args)
            if missing:
                errors.append(f"action[{index}]: missing args {sorted(missing)}")
            if action.tool == "set_goal" and "goal" in action.args:
                goal_ready = True
            if action.tool == "navigate_to" and not goal_ready:
                errors.append(f"action[{index}]: navigate_to without prior set_goal")
        return errors

    def safety_check(self, action: Action, env) -> tuple[bool, str]:
        # Deterministic simulation guard: navigation requires a goal and robot must not be stopped.
        if action.tool == "navigate_to" and not env.goal_set:
            return False, "safety/precondition gate: goal is not set"
        if env.stopped and action.tool != "stop_robot":
            return False, "safety gate: robot is stopped"
        if action.tool not in ALLOWED_TOOLS:
            return False, "safety gate: unknown tool"
        return True, "accepted"
