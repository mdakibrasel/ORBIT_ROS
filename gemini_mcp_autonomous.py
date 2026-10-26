from dotenv import load_dotenv
import json
import os
import re
import sys
from pathlib import Path

from google import genai
from orbit_mcp_server import _Client

ROOT = Path(__file__).resolve().parent
MODEL_CANDIDATES = ["gemini-3.6-flash", "gemini-3.8-flash"]
MAX_ACTIONS_PER_PLAN = 6

# Read-only tools are always permitted. Action tools must also match the
# movement/cleaning/stop name rules below and pass their MCP JSON schema.
READ_ONLY_NAMES = {"get_pose", "get_status", "get_capabilities", "house_info", "scan_obstacles"}
ACTION_KEYWORDS = (
    "walk", "move", "turn", "side_step", "sidestep", "go_to", "navigate",
    "clean", "vacuum", "mop", "brush", "stop_robot", "stop_motion"
)


def mcp_call(client, name, arguments=None):
    result = client.request("tools/call", {
        "name": name,
        "arguments": arguments or {},
    })
    if "error" in result:
        raise RuntimeError(f"MCP error calling {name}: {result['error']}")
    return result.get("result", result)


def start_mcp():
    command = [
        sys.executable, str(ROOT / "orbit_mcp_server.py"),
        "--robot", "nao", "--sim", "webots",
    ]
    client = _Client(command)
    init = client.request("initialize", {
        "protocolVersion": "2025-06-18",
        "capabilities": {},
        "clientInfo": {"name": "gemini-orbit-autonomous", "version": "1.0"},
    })
    if "error" in init:
        client.close(wait=True)
        raise RuntimeError(f"MCP initialize failed: {init['error']}")
    client.notify("notifications/initialized")
    response = client.request("tools/list")
    if "error" in response:
        client.close(wait=True)
        raise RuntimeError(f"MCP tools/list failed: {response['error']}")
    tools = response.get("result", {}).get("tools", [])
    if not tools:
        client.close(wait=True)
        raise RuntimeError("MCP server returned no tools.")
    return client, tools


def is_action_tool(name):
    lower = name.lower()
    return any(word in lower for word in ACTION_KEYWORDS)


def tool_catalog(tools):
    # Use only tools actually advertised by this running MCP server.
    catalog = []
    for tool in tools:
        name = tool.get("name", "")
        if name in READ_ONLY_NAMES or is_action_tool(name):
            catalog.append({
                "name": name,
                "description": tool.get("description", ""),
                "inputSchema": tool.get("inputSchema", {"type": "object", "properties": {}}),
            })
    return catalog


def validate_value(value, schema, path="arguments"):
    """Small JSON-Schema validator for common MCP tool schemas."""
    if not isinstance(schema, dict):
        return
    expected = schema.get("type")
    if expected == "object":
        if not isinstance(value, dict):
            raise ValueError(f"{path} must be an object.")
        required = schema.get("required", [])
        for key in required:
            if key not in value:
                raise ValueError(f"{path}.{key} is required.")
        props = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extras = set(value) - set(props)
            if extras:
                raise ValueError(f"{path} contains unsupported fields: {sorted(extras)}")
        for key, item in value.items():
            if key in props:
                validate_value(item, props[key], f"{path}.{key}")
    elif expected == "string":
        if not isinstance(value, str):
            raise ValueError(f"{path} must be a string.")
        if "enum" in schema and value not in schema["enum"]:
            raise ValueError(f"{path} must be one of {schema['enum']}.")
    elif expected == "integer":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{path} must be an integer.")
    elif expected == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{path} must be a number.")
    elif expected == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{path} must be a boolean.")
    elif expected == "array":
        if not isinstance(value, list):
            raise ValueError(f"{path} must be a list.")
        item_schema = schema.get("items", {})
        for index, item in enumerate(value):
            validate_value(item, item_schema, f"{path}[{index}]")

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            raise ValueError(f"{path} is below minimum {schema['minimum']}.")
        if "maximum" in schema and value > schema["maximum"]:
            raise ValueError(f"{path} exceeds maximum {schema['maximum']}.")


def find_tool(catalog, name):
    for tool in catalog:
        if tool["name"] == name:
            return tool
    return None


def safety_check(action, tool):
    name = action["tool"].lower()
    args = action.get("arguments", {})

    # Never execute arbitrary/non-advertised tools.
    if tool is None:
        raise ValueError("Tool is not advertised by the connected MCP server.")
    if name not in READ_ONLY_NAMES and not is_action_tool(name):
        raise ValueError("Tool is outside the movement/cleaning safety allowlist.")

    # Restrict navigation coordinates to the known 10 m x 8 m house.
    for key, value in args.items():
        key_l = key.lower()
        if key_l in {"x", "target_x", "goal_x"} and isinstance(value, (int, float)):
            if not 0 <= value <= 10:
                raise ValueError(f"{key} must be within house bounds 0..10 m.")
        if key_l in {"y", "target_y", "goal_y"} and isinstance(value, (int, float)):
            if not 0 <= value <= 8:
                raise ValueError(f"{key} must be within house bounds 0..8 m.")
    validate_value(args, tool.get("inputSchema", {"type": "object", "properties": {}}))


def gemini_json(gemini, prompt):
    last_error = None
    for index, model in enumerate(MODEL_CANDIDATES):
        print(f"\nGemini model: {model}")
        try:
            response = gemini.models.generate_content(
                model=model,
                contents=prompt,
                config={"response_mime_type": "application/json"},
            )
            if not response.text:
                raise RuntimeError(f"{model} returned an empty response.")
            try:
                return model, json.loads(response.text)
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"Gemini did not return valid JSON: {response.text}") from exc
        except Exception as exc:
            last_error = exc
            msg = str(exc).lower()
            unavailable = any(s in msg for s in (
                "model not found", "not supported for generatecontent",
                "is not found", "unsupported model", "unknown model",
            ))
            if unavailable and index < len(MODEL_CANDIDATES) - 1:
                print(f"{model} unavailable; trying fallback.")
                continue
            raise
    raise RuntimeError(f"All configured models failed: {last_error}")


def call_gemini_for_plan(gemini, catalog, user_command, observations, history):
    prompt = f"""
You are a cautious Webots NAO robot task planner. Return ONLY valid JSON:
{{
  "actions": [{{"tool": "exact_MCP_tool_name", "arguments": {{}}}}],
  "message": "short user-facing explanation"
}}

User request: {user_command}

Tools available (use only exact names and schemas below):
{json.dumps(catalog, ensure_ascii=False)}

Latest robot observations:
{json.dumps(observations, ensure_ascii=False)}

Actions already executed in this task:
{json.dumps(history, ensure_ascii=False)}

Rules:
- Plan at most {MAX_ACTIONS_PER_PLAN} actions.
- Use only tools in the provided catalog. Never invent a tool or argument.
- Arguments must match the tool's inputSchema exactly.
- For movement/cleaning, prefer short, incremental actions and observe status/pose after actions.
- For a room-cleaning request, navigate to that room first if a suitable navigation tool exists, then use the advertised cleaning tool.
- Do not claim cleaning succeeded unless a tool result supports that.
- If required information is unavailable, use a read-only tool or return an empty actions list and explain.
- Never issue repeated movement blindly. If an action fails, stop and explain rather than retrying indefinitely.
"""
    return gemini_json(gemini, prompt)


def main():
    load_dotenv(ROOT / ".env", override=True)
    api_key = os.environ.get("GEMINI_API_KEY", "").strip()
    if not api_key:
        raise RuntimeError("GEMINI_API_KEY is missing from the project's .env file.")

    user_command = " ".join(sys.argv[1:]).strip()
    if not user_command:
        user_command = input('Robot instruction (e.g. "Go to the kitchen and clean it"): ').strip()
    if not user_command:
        print("No instruction provided.")
        return

    gemini = genai.Client(api_key=api_key)
    client, raw_tools = start_mcp()
    try:
        catalog = tool_catalog(raw_tools)
        print("\n")
        for item in catalog:
            print(f"- {item['name']}: {item.get('description', '')[:120]}")

        observations = {}
        for name in READ_ONLY_NAMES:
            if find_tool(catalog, name):
                try:
                    observations[name] = mcp_call(client, name, {})
                except Exception as exc:
                    observations[name] = {"error": str(exc)}

        history = []
        model, plan = call_gemini_for_plan(
            gemini, catalog, user_command, observations, history
        )
        print(f"\nUsing model: {model}")
        print("\n")
        print(json.dumps(plan, indent=2, ensure_ascii=False))

        actions = plan.get("actions", [])
        if not isinstance(actions, list):
            raise RuntimeError("Gemini plan field 'actions' must be a list.")
        if len(actions) > MAX_ACTIONS_PER_PLAN:
            raise RuntimeError(f"Gemini proposed more than {MAX_ACTIONS_PER_PLAN} actions.")

        for index, action in enumerate(actions, start=1):
            if not isinstance(action, dict) or not isinstance(action.get("tool"), str):
                raise RuntimeError(f"Invalid action at index {index}.")
            name = action["tool"]
            args = action.get("arguments", {})
            tool = find_tool(catalog, name)
            if not isinstance(args, dict):
                raise RuntimeError(f"Arguments for {name} must be a JSON object.")
            safety_check({"tool": name, "arguments": args}, tool)

            # Stop is always available as an emergency action when advertised.
            print(f"\n[{index}/{len(actions)}] Calling {name} with {json.dumps(args, ensure_ascii=False)}")
            result = mcp_call(client, name, args)
            print("Tool result:")
            print(json.dumps(result, indent=2, ensure_ascii=False, default=str))
            history.append({"tool": name, "arguments": args, "result": result})

            # Stop on tool-level failures instead of blindly continuing.
            result_text = json.dumps(result, ensure_ascii=False, default=str).lower()
            if any(word in result_text for word in ('"iserror": true', '"error":', '"success": false')):
                print("Possible tool failure detected; stopping the action sequence.")
                break

        print("\n")
        final_observations = {}
        for name in ("get_pose", "get_status"):
            if find_tool(catalog, name):
                try:
                    final_observations[name] = mcp_call(client, name, {})
                except Exception as exc:
                    final_observations[name] = {"error": str(exc)}
        print(json.dumps(final_observations, indent=2, ensure_ascii=False, default=str))

        final_prompt = f"""
Summarize this Webots NAO task honestly and briefly in Bengali.
User request: {user_command}
Planned message: {plan.get('message', '')}
Executed tool history: {json.dumps(history, ensure_ascii=False, default=str)}
Final observations: {json.dumps(final_observations, ensure_ascii=False, default=str)}
Do not say the room is clean unless the tool results explicitly establish that.
"""
        _, final_response = gemini_json(gemini, final_prompt)
        if isinstance(final_response, dict):
            print("\n")
            print(final_response.get("message", json.dumps(final_response, ensure_ascii=False)))
        else:
            print("\n")
            print(final_response)
    finally:
        client.close(wait=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n")
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        sys.exit(1)
