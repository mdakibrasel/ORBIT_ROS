
import os
import sys
import json
from pathlib import Path

from dotenv import load_dotenv
from groq import Groq
from orbit_mcp_server import _Client

ROOT = Path(__file__).resolve().parent
MODEL = "openai/gpt-oss-120b"


def compact_schema(schema):
    """Keep only useful schema fields to reduce prompt size."""
    if not isinstance(schema, dict):
        return {}

    result = {}
    for key in ("type", "required", "enum", "minimum", "maximum"):
        if key in schema:
            result[key] = schema[key]

    if isinstance(schema.get("properties"), dict):
        result["properties"] = {
            name: compact_schema(value)
            for name, value in schema["properties"].items()
        }

    return result


def extract_result(response):
    """Extract the result text returned by an MCP tool."""
    result = response.get("result", {})
    if result.get("isError"):
        raise RuntimeError(str(result.get("content", result)))

    for item in result.get("content", []):
        if item.get("type") == "text":
            try:
                return json.loads(item["text"])
            except (ValueError, TypeError):
                return {"text": item["text"]}

    return result


def find_room_argument(tool, room):
    """Find the room argument using the server's actual schema."""
    schema = tool.get("inputSchema", {})
    properties = schema.get("properties", {})

    for name, spec in properties.items():
        allowed = spec.get("enum", [])
        if name.lower() in ("room", "room_name", "destination"):
            if not allowed or room in allowed:
                return {name: room}

        if room in allowed:
            return {name: room}

    raise RuntimeError(
        f"Could not identify the room argument for {tool['name']}. "
        f"Schema: {json.dumps(schema)}"
    )


def choose_tool(instruction, tools):
    """Use deterministic routing for clear commands."""
    text = instruction.lower()

    by_name = {tool["name"]: tool for tool in tools}

    # Room-navigation commands must not be converted into turns.
    rooms = ("living_room", "kitchen", "bedroom", "bathroom")
    room_aliases = {
        "living_room": ("living room", "living_room"),
        "kitchen": ("kitchen",),
        "bedroom": ("bedroom", "bed room"),
        "bathroom": ("bathroom", "bath room"),
    }

    navigation_words = (
        "go to", "walk to", "move to", "navigate to",
        "enter", "go into", "head to", "reach",
    )

    for room in rooms:
        mentions_room = any(
            alias in text for alias in room_aliases[room]
        )
        if mentions_room and any(word in text for word in navigation_words):
            if "go_to_room" not in by_name:
                raise RuntimeError("MCP tool 'go_to_room' is unavailable.")

            tool = by_name["go_to_room"]
            args = find_room_argument(tool, room)
            return tool, args

    # Clear turn instructions are routed to the turn tool.
    if "turn" in text or "rotate" in text:
        if "turn" in by_name:
            tool = by_name["turn"]
            schema = tool.get("inputSchema", {})
            props = schema.get("properties", {})
            args = {}

            direction = (
                "left" if "left" in text
                else "right" if "right" in text
                else None
            )
            if direction is None:
                raise RuntimeError(
                    "Please specify left or right for a turn."
                )

            for name, spec in props.items():
                if name.lower() in ("direction", "side"):
                    args[name] = direction
                elif name.lower() in ("degrees", "angle", "angle_deg"):
                    allowed = spec.get("enum", [])
                    requested = 40 if "40" in text else (
                        60 if "60" in text else (
                            180 if "180" in text else None
                        )
                    )
                    if requested is None:
                        requested = allowed[0] if allowed else 40
                    if allowed and requested not in allowed:
                        raise RuntimeError(
                            f"Unsupported turn angle: {requested}"
                        )
                    args[name] = requested

            return tool, args

    return None, None


def main():
    load_dotenv(ROOT / ".env", override=True)
    api_key = os.getenv("GROQ_API_KEY", "").strip()

    if not api_key:
        raise RuntimeError("GROQ_API_KEY missing from .env")

    instruction = " ".join(sys.argv[1:]).strip()
    if not instruction:
        instruction = input("Robot instruction: ").strip()

    if not instruction:
        raise RuntimeError("No robot instruction provided.")

    client = _Client([
        sys.executable,
        str(ROOT / "orbit_mcp_server.py"),
        "--robot", "nao",
        "--sim", "webots",
    ])

    try:
        response = client.request("initialize", {
            "protocolVersion": "2025-06-18",
            "capabilities": {},
            "clientInfo": {
                "name": "groq-orbit-planner",
                "version": "2.0",
            },
        })

        if "error" in response:
            raise RuntimeError(str(response["error"]))

        client.notify("notifications/initialized")

        response = client.request("tools/list")
        if "error" in response:
            raise RuntimeError(str(response["error"]))

        tools = response.get("result", {}).get("tools", [])
        by_name = {tool["name"]: tool for tool in tools}

        print(f"\nGroq model: {MODEL}")
        print(f"Instruction: {instruction}")

        # First handle clear commands without asking the LLM
        # to choose from an unrelated tool.
        selected_tool, args = choose_tool(instruction, tools)

        if selected_tool:
            print(
                "\nPlanner selected:",
                selected_tool["name"],
                json.dumps(args),
            )
        else:
            # Send a small catalogue of available tool names.
            # The selected tool's schema is checked afterwards.
            catalogue = [
                {
                    "name": tool["name"],
                    "description": str(tool.get("description", ""))[:100],
                }
                for tool in tools
            ]

            groq = Groq(api_key=api_key)
            result = groq.chat.completions.create(
                model=MODEL,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "Select the single best MCP tool for the user's "
                            "robot instruction. Return JSON with keys "
                            "'tool' and 'arguments'. Use only listed tool "
                            "names. Do not invent arguments."
                        ),
                    },
                    {
                        "role": "user",
                        "content": (
                            "Available tools:\n"
                            + json.dumps(catalogue, separators=(",", ":"))
                            + "\nInstruction: "
                            + instruction
                        ),
                    },
                ],
                response_format={"type": "json_object"},
                temperature=0,
                max_completion_tokens=250,
            )

            content = result.choices[0].message.content
            if not content:
                raise RuntimeError("Groq returned an empty response.")

            plan = json.loads(content)
            tool_name = plan.get("tool")
            args = plan.get("arguments", {})

            if tool_name not in by_name:
                raise RuntimeError(
                    f"Groq selected an unknown tool: {tool_name}"
                )

            selected_tool = by_name[tool_name]

            if not isinstance(args, dict):
                raise RuntimeError("Tool arguments must be a JSON object.")

            print("\nGroq plan:")
            print(json.dumps(plan, indent=2))

        tool_name = selected_tool["name"]
        schema = compact_schema(selected_tool.get("inputSchema", {}))
        properties = schema.get("properties", {})
        required = schema.get("required", [])

        # Reject missing or unknown arguments before execution.
        for key in required:
            if key not in args:
                raise RuntimeError(f"Missing required argument: {key}")

        for key, value in args.items():
            if key not in properties:
                raise RuntimeError(f"Unsupported argument: {key}")

            allowed = properties[key].get("enum")
            if allowed and value not in allowed:
                raise RuntimeError(
                    f"Invalid value for {key}: {value}. "
                    f"Allowed values: {allowed}"
                )

        print(f"\nExecuting: {tool_name} {json.dumps(args)}")

        movement = client.request("tools/call", {
            "name": tool_name,
            "arguments": args,
        })

        print("\nMovement result:")
        print(json.dumps(
            extract_result(movement),
            indent=2,
            ensure_ascii=False,
        ))

        # Read the actual final pose after the action.
        if "get_pose" in by_name:
            pose_response = client.request("tools/call", {
                "name": "get_pose",
                "arguments": {},
            })
            pose = extract_result(pose_response)

            print("\nFinal robot pose:")
            print(json.dumps(pose, indent=2, ensure_ascii=False))

            data = pose.get("data", pose)
            if tool_name == "go_to_room":
                requested_room = next(
                    (
                        value for value in args.values()
                        if isinstance(value, str)
                    ),
                    None,
                )
                actual_room = data.get("room")
                if requested_room and actual_room:
                    print(
                        "\nNavigation verification:",
                        "PASS" if actual_room == requested_room else "FAIL",
                    )
                    if actual_room != requested_room:
                        print(
                            f"Requested {requested_room}; "
                            f"actual room is {actual_room}."
                        )

    finally:
        client.close(wait=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        sys.exit(1)
