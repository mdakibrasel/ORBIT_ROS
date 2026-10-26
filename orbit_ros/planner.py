"""Planner adapters. MockPlanner is for harness tests, not research performance."""
from __future__ import annotations
import json, os, time
from typing import Any
from .models import Action, Plan
from .scenarios import Scenario

class MockPlanner:
    """Deterministic planner with controlled defects to test controller differences."""
    provider = "mock"
    model_id = "MockPlanner-v1"

    def __init__(self):
        self.usage_events: list[dict[str, Any]] = []

    def _full_actions(self, scenario: Scenario, seed: int, condition: str) -> list[Action]:
        actions = []
        for i in range(scenario.horizon):
            actions.extend([Action("set_goal", {"goal": f"goal_{i+1}"}, "Set waypoint"),
                            Action("navigate_to", {"waypoint": i+1}, "Navigate to waypoint")])
        # Deliberate deterministic defects exercise failure paths; not a model-quality simulation.
        if seed % 4 == 0 and actions:
            actions[0] = Action("navigate_to", {"waypoint": 1}, "Injected missing-goal precondition")
        if condition in {"C1", "C2"} and scenario.ambiguity > 0 and seed % 3 == 0:
            actions.append(Action("go_to", {"destination": "unclear"}, "Injected invalid tool"))
        return actions

    def make_plan(self, task_id: str, scenario: Scenario, seed: int, condition: str) -> Plan:
        return Plan(task_id=task_id, actions=self._full_actions(scenario, seed, condition))

    def choose_action(self, task_id: str, scenario: Scenario, seed: int, condition: str,
                      observation: dict[str, Any], remaining: list[Action]) -> Action:
        """Return one action for C1/C2. C1 is direct action selection; C2 replans a transient suffix."""
        step = int(observation.get("completed_waypoints", 0))
        if condition == "C1":
            # Prompt-only: choose directly from task + observation; no decomposition is created.
            if not observation.get("goal_set", False):
                return Action("set_goal", {"goal": f"goal_{step+1}"}, "Direct prompt action")
            return Action("navigate_to", {"waypoint": step+1}, "Direct prompt action")
        # C2: construct a disposable mini-plan at each decision, select its first action, then discard it.
        transient_plan = [Action("set_goal", {"goal": f"goal_{step+1}"}, "Transient plan")]
        if observation.get("goal_set", False):
            transient_plan = [Action("navigate_to", {"waypoint": step+1}, "Transient plan")]
        return transient_plan[0]

    def choose_unconstrained_action(self, task_id: str, scenario: Scenario, seed: int,
                                    observation: dict[str, Any], planned_action: Action) -> Action:
        """Mock C3 agent may choose a valid tool unrelated to the next stored plan action."""
        if seed % 2 == 1 and planned_action.tool == "set_goal":
            return Action("describe_image", {}, "Injected off-plan choice for C3 controller test")
        return planned_action

    def repair_suffix(self, plan: Plan, failed_action: Action, scenario: Scenario) -> list[Action]:
        start = min(scenario.horizon, plan.cursor // 2)
        suffix = []
        for i in range(start, scenario.horizon):
            suffix.extend([Action("set_goal", {"goal": f"repair_goal_{i+1}"}, "Repaired goal"),
                           Action("navigate_to", {"waypoint": i+1}, "Repaired navigation")])
        return suffix

class GeminiPlanner:
    """Google Gen AI planner. Requires google-genai and GEMINI_API_KEY."""
    provider = "google-gemini"

    def __init__(self, model_id: str = "gemini-2.5-flash", temperature: float = 0.0,
                 max_output_tokens: int = 2048):
        try:
            from google import genai
            from google.genai import types
        except ImportError as exc:
            raise RuntimeError("Gemini planner requires: pip install google-genai") from exc
        from .env_config import load_env; load_env()          # reads .env from the project root; real environment variables win
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError("Set GEMINI_API_KEY as an environment variable; do not put it in source code.")
        self.client = genai.Client(api_key=api_key)
        self.types = types
        self.model_id = model_id
        self.temperature = temperature
        self.max_output_tokens = max_output_tokens
        self.usage_events: list[dict[str, Any]] = []

    def _generate_json(self, prompt: str, task_id: str, condition: str) -> dict[str, Any]:
        start = time.perf_counter()
        response = self.client.models.generate_content(
            model=self.model_id,
            contents=prompt,
            config=self.types.GenerateContentConfig(
                temperature=self.temperature,
                max_output_tokens=self.max_output_tokens,
                response_mime_type="application/json",
            ),
        )
        latency = time.perf_counter() - start
        usage = getattr(response, "usage_metadata", None)
        input_tokens = getattr(usage, "prompt_token_count", None) if usage else None
        output_tokens = getattr(usage, "candidates_token_count", None) if usage else None
        total_tokens = getattr(usage, "total_token_count", None) if usage else None
        self.usage_events.append({
            "task_id": task_id, "condition": condition, "provider": self.provider,
            "model_id": self.model_id, "input_tokens": input_tokens,
            "output_tokens": output_tokens, "total_tokens": total_tokens,
            "latency_seconds": round(latency, 6), "usage_metadata_available": usage is not None,
            "estimated_cost_usd": None, "cost_note": "Configure pricing externally; no rate assumed",
        })
        text = getattr(response, "text", None)
        if not text:
            raise RuntimeError("Gemini returned an empty response")
        try:
            return json.loads(text)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Gemini response was not valid JSON: {text[:300]}") from exc

    @staticmethod
    def _actions_from_payload(payload: dict[str, Any]) -> list[Action]:
        raw = payload.get("actions")
        if not isinstance(raw, list):
            raise ValueError("Planner JSON must contain an 'actions' list")
        actions = []
        for item in raw:
            if not isinstance(item, dict) or not isinstance(item.get("tool"), str) or not isinstance(item.get("args", {}), dict):
                raise ValueError("Each action must contain a string 'tool' and object 'args'")
            actions.append(Action(item["tool"], item.get("args", {}), str(item.get("purpose", ""))))
        return actions

    def make_plan(self, task_id: str, scenario: Scenario, seed: int, condition: str) -> Plan:
        prompt = f"""Create a robot-tool plan as JSON only. Task: navigate through {scenario.horizon} ordered waypoints, setting each goal before navigation. Available tools: get_goal_odometry(args={{}}), set_goal(args={{'goal': string}}), describe_image(args={{}}), navigate_to(args={{'waypoint': integer}}), stop_robot(args={{}}). Return {{\"actions\":[{{\"tool\":string,\"args\":object,\"purpose\":string}}]}}. Include exactly one set_goal then navigate_to for each waypoint, in order. Do not include markdown."""
        payload = self._generate_json(prompt, task_id, condition)
        return Plan(task_id, self._actions_from_payload(payload))

    def choose_action(self, task_id: str, scenario: Scenario, seed: int, condition: str,
                      observation: dict[str, Any], remaining: list[Action]) -> Action:
        policy = ("prompt-only action selection; do not create or retain an explicit plan" if condition == "C1"
                  else "reconstruct a disposable task plan from the current observation, select its first action, then discard that plan")
        prompt = f"""Policy: {policy}. Choose the next robot tool action for this task. Task: navigate through {scenario.horizon} ordered waypoints. Available tools: get_goal_odometry(args={{}}), set_goal(args={{'goal': string}}), describe_image(args={{}}), navigate_to(args={{'waypoint': integer}}), stop_robot(args={{}}). Current observation JSON: {json.dumps(observation)}. Return JSON only: {{\"actions\":[{{\"tool\":string,\"args\":object,\"purpose\":string}}]}} with exactly one action. If goal_set is false, call set_goal; otherwise navigate to the next unfinished waypoint. No markdown."""
        payload = self._generate_json(prompt, task_id, condition)
        actions = self._actions_from_payload(payload)
        if len(actions) != 1:
            raise ValueError("Action-selection prompt must return exactly one action")
        return actions[0]

    def choose_unconstrained_action(self, task_id: str, scenario: Scenario, seed: int,
                                    observation: dict[str, Any], planned_action: Action) -> Action:
        # C3 explicitly asks the model to choose from the available tools without a plan-alignment gate.
        return self.choose_action(task_id, scenario, seed, "C3", observation, [planned_action])

    def repair_suffix(self, plan: Plan, failed_action: Action, scenario: Scenario) -> list[Action]:
        # A separate LLM repair call can be added as an explicitly measured repair policy later.
        start = min(scenario.horizon, plan.cursor // 2)
        suffix = []
        for i in range(start, scenario.horizon):
            suffix.extend([Action("set_goal", {"goal": f"repair_goal_{i+1}"}, "Repaired goal"),
                           Action("navigate_to", {"waypoint": i+1}, "Repaired navigation")])
        return suffix
