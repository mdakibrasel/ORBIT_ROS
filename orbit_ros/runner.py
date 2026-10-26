import json, time
from .models import Action, Plan
from .planner import MockPlanner
from .contract import ContractVerifier
from .mock_environment import MockRobotEnvironment
from .scenarios import SCENARIOS

CONDITIONS = {
    "C1": "prompt-only: choose one action per decision; no explicit plan retained",
    "C2": "transient plan: re-derive action from task and current observation each step",
    "C3": "persistent plan and cursor; execution is not plan-constrained",
    "C4": "persistent plan and cursor; only the next planned action may execute",
    "C5": "persistent plan + deterministic verification (proposed extension)",
    "C6": "C5 + bounded trigger-based partial repair (proposed extension)",
}

class TrialRunner:
    def __init__(self, planner=None):
        self.planner = planner or MockPlanner()
        self.verifier = ContractVerifier()

    def run(self, condition: str, task_level: str, seed: int):
        scenario = SCENARIOS[task_level]
        env = MockRobotEnvironment(scenario, seed)
        task_id = f"{task_level}-{condition}-seed{seed}"
        start = time.perf_counter()
        # C1/C2 do not retain a plan object as their execution state. The empty Plan is only a log carrier.
        if condition in {"C1", "C2"}:
            plan = Plan(task_id, [])
        else:
            plan = self.planner.make_plan(task_id, scenario, seed, condition)
        events = []
        def log(event, **kwargs):
            events.append({"task_id": task_id, "condition": condition, "task_level": task_level,
                           "seed": seed, "event": event, "details": json.dumps(kwargs, sort_keys=True)})
        log("condition_started", policy=CONDITIONS[condition], planner=getattr(self.planner, "provider", "unknown"))

        errors = self.verifier.verify_plan(plan) if condition in {"C5", "C6"} else []
        repairs = safety_stops = 0
        if errors:
            log("plan_verification_failed", errors=errors)
            if condition == "C5":
                elapsed = time.perf_counter() - start
                return self._metrics(task_id, condition, task_level, seed, env, plan, False, elapsed, 0, 0, events)
            plan.actions = self.planner.repair_suffix(plan, Action("contract_failure", {}, ""), scenario)
            plan.revision += 1
            errors = self.verifier.verify_plan(plan)
            log("plan_repaired", revision=plan.revision, errors_after_repair=errors)
            if errors:
                elapsed = time.perf_counter() - start
                return self._metrics(task_id, condition, task_level, seed, env, plan, False, elapsed, 0, 1, events)

        max_actions = scenario.horizon * 5 + 10
        steps = 0
        while steps < max_actions:
            # C1/C2 request a fresh action every decision; C3 can choose any tool and logs deviation;
            # C4/C5/C6 execute only the next stored action (C4 gate, C5/C6 verified gate).
            if condition in {"C1", "C2"}:
                if env.completed_waypoints >= scenario.horizon:
                    break
                observation = {"completed_waypoints": env.completed_waypoints, "goal_set": env.goal_set,
                               "stopped": env.stopped, "scenario": task_level}
                action = self.planner.choose_action(task_id, scenario, seed, condition, observation, [])
                if condition == "C2":
                    log("transient_plan_reconstructed", retained_after_decision=False,
                        observation=observation, selected_action=action.tool)
                log("action_selected_without_persistent_plan", action=action.tool, args=action.args)
            elif condition == "C3":
                if plan.cursor >= len(plan.actions): break
                planned_action = plan.actions[plan.cursor]
                observation = {"completed_waypoints": env.completed_waypoints, "goal_set": env.goal_set,
                               "stopped": env.stopped, "scenario": task_level}
                chooser = getattr(self.planner, "choose_unconstrained_action", None)
                action = chooser(task_id, scenario, seed, observation, planned_action) if chooser else planned_action
                log("unconstrained_action_selected", cursor=plan.cursor, planned_tool=planned_action.tool,
                    selected_tool=action.tool, deviated=(action.tool != planned_action.tool))
            else:
                if plan.cursor >= len(plan.actions): break
                action = plan.actions[plan.cursor]
                log("planned_action_selected", cursor=plan.cursor, action=action.tool)

            if condition in {"C4", "C5", "C6"}:
                allowed, reason = self.verifier.safety_check(action, env)
                if not allowed:
                    safety_stops += 1
                    log("execution_gate_block", cursor=plan.cursor, reason=reason, tool=action.tool)
                    if condition == "C6" and repairs < 2:
                        plan.actions = plan.actions[:plan.cursor] + self.planner.repair_suffix(plan, action, scenario)
                        plan.revision += 1; repairs += 1
                        log("partial_repair", revision=plan.revision, repairs=repairs, trigger=reason)
                        continue
                    break
            result = env.execute(action)
            log("tool_result", cursor=(plan.cursor if condition not in {"C1", "C2"} else None),
                tool=action.tool, ok=result.ok, message=result.message)
            steps += 1
            if condition in {"C1", "C2"}:
                # A successful set_goal does not advance waypoint; navigation success does.
                if not result.ok:
                    if condition == "C2" and repairs < 1:
                        repairs += 1
                        log("transient_replan_after_failure", trigger=result.message, repairs=repairs)
                        continue
                    break
            else:
                if result.ok:
                    plan.cursor += 1
                elif condition == "C6" and repairs < 2:
                    plan.actions = plan.actions[:plan.cursor] + self.planner.repair_suffix(plan, action, scenario)
                    plan.revision += 1; repairs += 1
                    log("partial_repair", revision=plan.revision, repairs=repairs, trigger=result.message)
                else:
                    break
        success = env.completed_waypoints >= scenario.horizon
        elapsed = time.perf_counter() - start
        return self._metrics(task_id, condition, task_level, seed, env, plan, success, elapsed, safety_stops, repairs, events)

    @staticmethod
    def _metrics(task_id, condition, task_level, seed, env, plan, success, elapsed, safety_stops, repairs, events):
        return ({"task_id": task_id, "condition": condition, "task_level": task_level, "seed": seed,
                 "success": int(success), "planned_actions": len(plan.actions), "executed_tool_calls": env.tool_calls,
                 "invalid_calls": env.invalid_calls, "safety_blocks": safety_stops,
                 "safety_violations": env.safety_violations, "repairs": repairs, "plan_revisions": plan.revision,
                 "completed_waypoints": env.completed_waypoints, "target_waypoints": SCENARIOS[task_level].horizon,
                 "disturbances": env.disturbances, "elapsed_seconds": round(elapsed, 6),
                 "plan_adherence": round(plan.cursor / max(1, len(plan.actions)), 4),
                 "planner_provider": getattr(getattr(TrialRunner, '_active_planner', None), 'provider', 'see_run_metadata')}, events)
