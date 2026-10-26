"""Model Context Protocol (MCP) server for ORBIT ROS (v0.15), standard library only.

    python orbit_mcp_server.py --sim world2d        # started by the AI client (Claude Desktop, VS Code, Gemini CLI, ...)

Transport: stdio, newline-delimited JSON-RPC 2.0, as the MCP specification defines it. Implemented: initialize (protocol versions
2024-11-05 to 2025-11-25), notifications/initialized, ping, tools/list, tools/call. This is the "tools" part of MCP, which is all an AI
client needs to drive the robot. Written without the `mcp` package on purpose: ORBIT keeps running on the standard library + numpy, and
nothing extra has to be installed for the AI client to start the server. tests/test_mcp_server.py checks it against the official MCP
Python SDK client when that package is installed.

The robot tools are generated from the PEL tool registry (orbit_ros/pel/registry.py), and every one of them is executed by
OrbitMcpSession.robot_call, i.e. through the registry and the safety guard (see orbit_ros/mcp_bridge.py). The JSON schemas below only help
the AI client form a call; the registry is the authority and re-checks every argument.
"""
from __future__ import annotations
import json, sys, traceback
from .pel.registry import REGISTRY
from .mcp_bridge import OrbitMcpSession, scene_list
from . import house

SERVER_NAME = "orbit-ros"
PROTOCOL_VERSIONS = ("2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05")     # newest first

INSTRUCTIONS = """You control ORBIT, a TurtleBot3 robot in a simulated room (World2D, or Webots when the server was started with --sim webots).
Map: grid of 1 m cells, x to the right, y up. yaw 0 = facing +x, 90 = +y, 180 = -x, 270 = -y. Turning LEFT adds 90 degrees, RIGHT subtracts.
move_forward moves along the current heading. Objects occupy one cell; the robot cannot enter them or leave the room.
Typical loop for a request in natural language: get_pose and describe_view (objects come with an estimated world cell), decide, then use
move_forward / turn, or goto_cell for a target cell; check the result and observe again. Use set_no_go_cells when the person says to keep
away from somewhere. Every robot call is checked by the ORBIT tool registry and safety guard: a call that is "rejected" (invalid) or
"vetoed" (unsafe) did NOT move the robot; read the message and choose another action. stop_robot halts the robot; only load_scene restarts it.
"""

ROBOT_DOC = {
    "move_forward": "Drive straight ahead along the current heading by a whole number of 1 m cells (0..12).",
    "turn": "Turn on the spot. LEFT = counter-clockwise (+90 deg per quarter turn), RIGHT = clockwise.",
    "clean": "Clean the cell the robot is standing on.",
    "describe_view": "Camera: objects in view with label, image_x (-1 left edge .. +1 right edge), apparent_size, bearing_deg (left positive), "
                     "distance_m and the estimated world cell world_xy.",
    "get_pose": "The robot's reported pose: x, y (cell), yaw_deg (0/90/180/270), room_w, room_h.",
    "localize": "Map-based localiser: corrects the pose estimate after odometry drift.",
    "report": "State an answer to a question the person asked (for example how many boxes are in the room).",
    "stop_robot": "Emergency stop. The robot refuses every motion afterwards until load_scene.",
    "goto_cell": "Drive to map cell (x, y) on a shortest path through cells not believed blocked (many robot actions, each one checked).",
    "cover_region": "Clean every reachable cell of one half of the room (halves relative to the heading at the start of the scene); "
                    "avoid = cells never to enter.",
}

def _arg_schema(a) -> dict:
    d = {"description": a.doc} if a.doc else {}
    if a.kind == "int":
        d.update(type="integer"); d.update({k: v for k, v in (("minimum", a.min), ("maximum", a.max)) if v is not None})
    elif a.kind == "enum":
        d.update(enum=list(a.values)); d.update(type="integer" if all(isinstance(v, int) for v in a.values) else "string")
    elif a.kind == "intlike":
        d.update(type="integer")
    elif a.kind == "cells":
        d.update(type="array", items={"type": "array", "items": {"type": "integer"}, "minItems": 2, "maxItems": 2})
    else:
        d.update(type="string")
    return d

def robot_tool_def(name: str) -> dict:
    s = REGISTRY[name]
    schema = {"type": "object", "properties": {k: _arg_schema(a) for k, a in s.args.items()},
              "required": [k for k, a in s.args.items() if a.required]}
    ann = {"title": name.replace("_", " "), "readOnlyHint": s.category == "sensing", "destructiveHint": False,
           "idempotentHint": bool(s.retry_safe), "openWorldHint": False}
    return {"name": name, "description": ROBOT_DOC.get(name, name) + " Checked by the ORBIT registry and safety guard before it is sent.",
            "inputSchema": schema, "annotations": ann}

_RO = {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}
SESSION_TOOLS = {
    "list_scenes": ("List the rooms that load_scene can build (demo, empty, and benchmark task families task:<family>).",
                    {"type": "object", "properties": {}}, _RO),
    "load_scene": ("Build a room in the simulator and place the robot at its start (also the only way to restart after stop_robot). "
                   "scene: demo | empty | task:<family>; seed, yaw (0/90/180/270), n_objects and level apply to task scenes.",
                   {"type": "object", "properties": {"scene": {"type": "string", "default": "demo"}, "seed": {"type": "integer", "default": 0},
                                                     "yaw": {"type": "integer", "enum": [0, 90, 180, 270], "default": 0},
                                                     "n_objects": {"type": "integer", "minimum": 1, "maximum": 5, "default": 1},
                                                     "level": {"type": "integer", "minimum": 1, "maximum": 4, "default": 1}}},
                   {"readOnlyHint": False, "destructiveHint": True, "idempotentHint": False, "openWorldHint": False}),
    "get_status": ("Session status: pose, stop state, believed obstacles, no-go cells, cleaned cells, call counts, last safety veto.",
                   {"type": "object", "properties": {}}, _RO),
    "get_map": ("ASCII map of what the robot believes about the room (not the simulator's ground truth).", {"type": "object", "properties": {}}, _RO),
    "set_no_go_cells": ("Declare cells the robot must never enter, e.g. around an object the person said to keep away from. The safety guard "
                        "enforces them until the next load_scene.",
                        {"type": "object", "properties": {"cells": {"type": "array", "items": {"type": "array", "items": {"type": "integer"},
                                                                                              "minItems": 2, "maxItems": 2}}},
                         "required": ["cells"]},
                        {"readOnlyHint": False, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}),
    "get_capabilities": ("The robot, the simulator and every robot tool with its argument limits and preconditions (from the PEL registry).",
                         {"type": "object", "properties": {}}, _RO),
    "check_task": ("Evaluator score of the current task:<family> scene (uses ground truth; for checking a run, not for steering).",
                   {"type": "object", "properties": {}}, _RO),
}

class McpServer:
    instructions = INSTRUCTIONS
    def __init__(self, session: OrbitMcpSession, version: str = "0"):
        self.s, self.version = session, version
        self.protocol = None

    # ------------------------------------------------------------------------------------------------------------ tools
    def tool_defs(self) -> list:
        out = [robot_tool_def(t) for t in self.s.robot_tools()]
        for name, (desc, schema, ann) in SESSION_TOOLS.items():
            out.append({"name": name, "description": desc, "inputSchema": schema, "annotations": {"title": name.replace("_", " "), **ann}})
        return out

    def call_tool(self, name: str, args: dict):
        """-> (payload, is_error). Robot tools go to OrbitMcpSession.robot_call and nowhere else."""
        if name in self.s.robot_tools():
            r = self.s.robot_call(name, args)
            return r, r["status"] != "ok"
        if name == "list_scenes": return scene_list(), False
        if name == "load_scene":
            return self.s.load_scene(str(args.get("scene", "demo")), int(args.get("seed", 0)), int(args.get("yaw", 0)),
                                     int(args.get("n_objects", 1)), int(args.get("level", 1))), False
        if name == "get_status": return self.s.status(), False
        if name == "get_map": return self.s.believed_map(), False
        if name == "set_no_go_cells": return self.s.set_no_go(args.get("cells") or []), False
        if name == "get_capabilities": return self.s.capabilities(), False
        if name == "check_task": return self.s.check_task(), False
        raise KeyError(name)

    # ------------------------------------------------------------------------------------------------------------ JSON-RPC
    def handle(self, msg):
        """One JSON-RPC message -> response dict, or None for a notification."""
        if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0" or not isinstance(msg.get("method"), str):
            if isinstance(msg, dict) and "method" not in msg and ("result" in msg or "error" in msg): return None   # a response to us: ignore
            return _err(msg.get("id") if isinstance(msg, dict) else None, -32600, "Invalid Request")
        mid, method, params = msg.get("id"), msg["method"], msg.get("params") or {}
        notification = "id" not in msg
        try:
            result = self._dispatch(method, params)
        except _RpcError as e:
            return None if notification else _err(mid, e.code, e.message)
        except Exception as e:                                       # never let one bad request kill the server
            print(traceback.format_exc(), file=sys.stderr)
            return None if notification else _err(mid, -32603, f"Internal error: {type(e).__name__}: {e}")
        if notification: return None
        return {"jsonrpc": "2.0", "id": mid, "result": result}

    def _dispatch(self, method, params):
        if method == "initialize":
            want = params.get("protocolVersion")
            self.protocol = want if want in PROTOCOL_VERSIONS else PROTOCOL_VERSIONS[0]
            return {"protocolVersion": self.protocol, "capabilities": {"tools": {"listChanged": False}},
                    "serverInfo": {"name": SERVER_NAME, "title": "ORBIT ROS robot (TurtleBot3 or NAO humanoid, World2D / Webots)", "version": self.version},
                    "instructions": self.instructions}
        if method.startswith("notifications/"): return None
        if method == "ping": return {}
        if method == "tools/list": return {"tools": self.tool_defs()}
        if method == "tools/call":
            name, args = params.get("name"), params.get("arguments") or {}
            if not isinstance(args, dict): raise _RpcError(-32602, "arguments must be an object")
            known = {t["name"] for t in self.tool_defs()}
            if name not in known: raise _RpcError(-32602, f"Unknown tool: {name}")
            try:
                payload, is_error = self.call_tool(name, args)
            except (ValueError, TypeError) as e:
                payload, is_error = {"status": "error", "message": str(e)}, True
            except Exception as e:                                   # simulator failure: report to the model, keep serving
                print(traceback.format_exc(), file=sys.stderr)
                payload, is_error = {"status": "error", "message": f"{type(e).__name__}: {e}"}, True
            content = []
            if isinstance(payload, dict) and payload.get("_image"):            # v0.16: camera picture for the AI client to look at
                content.append({"type": "image", "data": payload["_image"], "mimeType": payload.get("_mime", "image/png")})
                payload = {k: v for k, v in payload.items() if not k.startswith("_")}
            content.insert(0, {"type": "text", "text": json.dumps(payload, indent=1, default=str)})
            return {"content": content, "isError": bool(is_error)}
        raise _RpcError(-32601, f"Method not found: {method}")

    def serve(self, rfile, wfile):
        """Read newline-delimited JSON-RPC from rfile (binary), write responses to wfile (binary). Returns at end of input."""
        for raw in rfile:
            line = raw.strip()
            if not line: continue
            try:
                msg = json.loads(line.decode("utf-8") if isinstance(line, bytes) else line)
            except (ValueError, UnicodeDecodeError):
                self._send(wfile, _err(None, -32700, "Parse error")); continue
            if isinstance(msg, list):                                # JSON-RPC batch (protocol 2025-03-26)
                out = [r for r in (self.handle(m) for m in msg) if r is not None]
                if out: self._send(wfile, out)
            else:
                r = self.handle(msg)
                if r is not None: self._send(wfile, r)

    @staticmethod
    def _send(wfile, obj):
        data = (json.dumps(obj, ensure_ascii=False, default=str) + "\n").encode("utf-8")
        wfile.write(data); wfile.flush()

class _RpcError(Exception):
    def __init__(self, code, message): super().__init__(message); self.code, self.message = code, message

def _err(mid, code, message): return {"jsonrpc": "2.0", "id": mid, "error": {"code": code, "message": message}}


# ---------------------------------------------------------------------------------------------------------------- v0.16 humanoid

HUMANOID_INSTRUCTIONS = """You control a NAO humanoid robot (0.58 m tall, walks on two legs) in a furnished one-floor house (Webots, or a kinematic stand-in).
Use go_to_room/go_to_object for destinations; these plan guarded routes. Coordinates are metres (x east, y north); yaw 0 faces +x,
90 faces +y. Every motion is checked against walls, furniture and no-go zones; rejected/vetoed means no motion occurred. If fallen,
call stand_up. stop_robot requires load_scene to resume.
Use clean(mode="vacuum"|"mop") to select the visible attachment; cleaning mode persists until changed or scene reload. Use verify_cleaning
to check brush placement and estimated coverage, verify_task for task completion. Coverage is simulated, not sensed dirt.
For multiple destinations/actions, call run_plan once; preserve the user's order only when specified. Prefer one relevant tool call at a time,
avoid redundant status reads, and keep replies concise. Consult house_info/describe_view/scan_obstacles only when needed; memory is advisory,
while live pose/sensors and the safety guard take precedence. greet waves without speech; join_tv_watching approaches the sofa safely.
"""

PLAN_STEPS_DOC = ("Ordered steps, 1..12. Each has exactly one target: room, or object (furniture name), or x and y (metres), and do = visit (default), "
                  "clean (sweep a room, or make a stationary brush pass at an object or point) or wave.")
PLAN_DOC = ("break a request like 'clean the kitchen, the bedroom and the area by the sofa, then come back' into one step each "
            "(steps), e.g. [{\"room\": \"kitchen\", \"do\": \"clean\"}, {\"object\": \"sofa\", \"do\": \"clean\"}], finish = start to return to the start. "
            "order = optimize (default) puts the steps in the order that needs the least walking, order = given keeps your order (use it when the "
            "person said first/then/before/after). The reply lists each step with where the robot stands, the walking length and its completion condition; "
            "run_plan reports for every step whether that condition was met. A step with no safe route is reported, not forced.")

HUMANOID_DOC = {
    "walk_forward": "Walk forward a number of walking cycles (1..6). The distance per cycle varies; check get_pose afterwards.",
    "walk_backward": "Walk backward 1..3 cycles.",
    "turn": "Turn on the spot by 40, 60 or 180 degrees, left (counter-clockwise) or right.",
    "side_step": "One step sideways, left or right.",
    "go_to": "Walk to the point (x, y) in metres of the house: plans a path around walls and furniture, then walks it one checked motion at a time.",
    "go_to_room": "Walk into a room of the house (living_room, kitchen, bedroom, bathroom) along a planned path through the doorways.",
    "go_to_object": "Walk next to a named house item (e.g. fridge, sofa, bed) and turn to face it.",
    "turn_to_object": "Turn in place to face a named house item without walking to it.",
    "look_around": "Turn in place through a full circle and report every house item seen at any heading (one describe_view only covers a 61 degree cone). Returns to the starting heading; does not walk.",
    "wave": "Wave a hand (HandWave motion).",
    "greet": "Greet the person with the NAO's supported hand-wave motion (no spoken audio).",
    "join_tv_watching": "Safely navigate to a clear place beside the sofa and turn toward the television.",
    "get_pose": "Position x, y (m), heading yaw_deg, the room the robot is in, and whether it has fallen.",
    "describe_view": "House items in front of the robot (61 degree view, 5 m, walls hide what is behind them): label, bearing_deg (left positive), distance_m, x, y, room.",
    "look": "The picture from the NAO's top camera (Webots only), so you can see the scene yourself.",
    "stand_up": "Put the robot back on its feet at the same place after a fall (simulator reset).",
    "stop_robot": "Emergency stop: every motion is refused until load_scene.",
    "clean": "Use the Webots floor attachment for one pass. mode selects the visible vacuum or mop head; the selection persists until changed or the scene reloads.",
    "clean_room": "Navigate safely to the selected room, sweep with the selected floor attachment and report measured coverage. pattern quick or full; mode optionally selects vacuum or mop.",
    "clean_house": "Sweep every room with the selected floor attachment (pattern quick = rows 1 m apart, full = 0.34 m apart), replanning around obstacles; reports floor coverage. mode optionally selects vacuum or mop.",
    "clean_corners": "Clean every corner of one room or of the whole house (room = all): walks diagonally into each reachable corner and uses the selected floor attachment. Lists corners blocked by furniture.",
    "clean_direction": "Clean while moving with the selected floor attachment: direction forward, backward, left or right (left/right are side steps), steps 1..6 (backward 1..3); mode optionally selects vacuum or mop.",
    "scan_obstacles": "Distances to the nearest wall, furniture or bumped obstacle at five bearings (left positive) plus the live sonar reading ahead and any unmapped obstacles the sonar found.",
    "motion_accuracy": "Measured turn error and walking heading drift of all motions so far, with pass/fail against the tolerances.",
    "verify_cleaning": "Check the brush position against where it should be and report the share of each room's free floor the brush has covered.",
    "verify_task": "Independently check the last task: robot upright, at its goal (within 0.45 m), rooms covered, brush in place.",
    "clean_status": "Report cleaning mode, paused state, rooms visited and cleaning progress.",
    "pause_cleaning": "Pause the cleaning job without moving the robot.",
    "resume_cleaning": "Resume a paused cleaning job.",
    "return_to_start": "Return to the room entry point where the current scene started.",
    "tour_house": "Visit each of the four rooms in sequence using guarded paths. finish=start returns to the session start point; finish=stay leaves the NAO in the last room.",
    "plan_tasks": "Plan several targets without moving: " + PLAN_DOC,
    "run_plan": "Plan several targets, then run them in order and check each step's completion condition: " + PLAN_DOC,
}

def humanoid_tool_def(name):
    from .humanoid import HUMANOID_TOOLS
    s = HUMANOID_TOOLS[name]
    props = {}
    for k, a in s.args.items():
        props[k] = {"type": "number", "description": a.doc} if name == "go_to" else _arg_schema(a)
        if k == "steps": props[k] = {"type": "array", "minItems": 1, "maxItems": 12, "description": PLAN_STEPS_DOC, "items": {
            "type": "object", "additionalProperties": False, "properties": {
                "room": {"type": "string", "enum": sorted(house.ROOMS)}, "object": {"type": "string", "description": "furniture name, see house_info"},
                "x": {"type": "number"}, "y": {"type": "number"}, "do": {"type": "string", "enum": ["visit", "clean", "wave"], "default": "visit"}}}}
    ann = {"title": name.replace("_", " "), "readOnlyHint": s.category == "sensing", "destructiveHint": False,
           "idempotentHint": bool(s.retry_safe), "openWorldHint": False}
    return {"name": name, "description": HUMANOID_DOC[name] + " Checked by the ORBIT argument check and humanoid safety guard.",
            "inputSchema": {"type": "object", "properties": props, "required": [k for k, a in s.args.items() if a.required]}, "annotations": ann}

H_SESSION_TOOLS = {
    "list_scenes": ("List the rooms of the house where load_scene can stand the robot.", {"type": "object", "properties": {}}, _RO),
    "load_scene": ("Stand the robot at the entry point of a room and start a new session (also the only restart after stop_robot).",
                   {"type": "object", "properties": {"scene": {"type": "string", "enum": sorted(house.ROOMS), "default": "living_room"}}},
                   {"readOnlyHint": False, "destructiveHint": True, "idempotentHint": False, "openWorldHint": False}),
    "house_info": ("The house plan: rooms with their area and entry point, and every piece of furniture/person with position, size and room.",
                   {"type": "object", "properties": {}}, _RO),
    "get_status": ("Pose, fallen and stop state, bumped obstacles, no-go zones, call counts, last safety veto.", {"type": "object", "properties": {}}, _RO),
    "get_map": ("Text map of the house as the robot knows it (one character = 0.25 m).", {"type": "object", "properties": {}}, _RO),
    "set_no_go_zone": ("Declare a circle the robot must never enter (x, y, radius in metres), e.g. around something the person said to avoid.",
                       {"type": "object", "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "radius": {"type": "number", "default": 0.5}},
                        "required": ["x", "y"]}, {"readOnlyHint": False, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}),
    "get_capabilities": ("The robot, its tools with argument limits, and the rooms.", {"type": "object", "properties": {}}, _RO),
    "connection_status": ("Transport and simulator health (read-only, never moves the robot): which backend is live, whether the Webots "
                          "extern controller is actually connected on tcp://127.0.0.1:1235, the robot's ready/stopped state, pose when live, and "
                          "the remaining call budget. Call this first when a command returns ok but the robot does not appear to move.",
                          {"type": "object", "properties": {}}, _RO),
    "debug_view": ("Visual debugging: the house map with the planned route (*), the current target (T), unmapped obstacles found by bump or sonar (?) "
                   "and the robot, plus the task status, target, route, pose and obstacle locations. Does not move the robot.",
                   {"type": "object", "properties": {}}, _RO),
    "camera": ("Adaptive display camera for the live Webots view (DISPLAY ONLY: never moves the robot, never calls the LLM, never changes "
               "memory). Choose a framing mode and optionally dolly the view. mode: overview (angled bird's-eye of the whole house) | "
               "follow (keep the walking robot centred) | closeup (tight shot of the NAO) | target (frame the robot and a point x,y together) "
               "| human (frame the animated human actor if present) | demo (follow for a walkthrough) | reset (restore the saved overview). "
               "zoom: 'in', 'out' or a positive number to dolly closer/further (clamped). For mode 'target' pass target:[x,y]. "
               "In --sim fake or without a Webots viewpoint this is a safe no-op (available:false).",
               {"type": "object", "properties": {
                   "mode": {"type": "string", "enum": ["overview", "follow", "closeup", "target", "human", "demo", "reset", "zoom"], "default": "overview"},
                   "zoom": {"description": "dolly amount: 'in', 'out' or a positive number", "oneOf": [{"type": "string", "enum": ["in", "out"]}, {"type": "number", "exclusiveMinimum": 0}]},
                   "target": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2, "description": "[x, y] for mode 'target'"}}},
               _RO),
    "validate_plan": ("Statically validate a plan JSON (tools, args, dependencies, preconditions) WITHOUT moving the robot. Returns the list "
                      "of issues (empty = admissible). A plan with feasible:false and a refusal_reason is valid (an explicit refusal).",
                      {"type": "object", "properties": {"plan": {"type": "object"}}, "required": ["plan"]}, _RO),
    "execute_plan": ("Validate a plan JSON, then execute it deterministically through the safety guard with per-step verification and "
                     "bounded repair. Returns a RunResult (success, status, per-step results, metrics). Repair uses the configured provider if available.",
                     {"type": "object", "properties": {"plan": {"type": "object"}}, "required": ["plan"]},
                     {"readOnlyHint": False, "destructiveHint": True, "idempotentHint": False, "openWorldHint": False}),
    "plan_and_execute": ("Run the full meta-tool loop for a natural-language instruction: the provider produces a schema-constrained plan, "
                         "it is validated and self-refined BEFORE any motion, then executed with the guard, verification and repair. Returns a RunResult.",
                         {"type": "object", "properties": {"instruction": {"type": "string"}}, "required": ["instruction"]},
                         {"readOnlyHint": False, "destructiveHint": True, "idempotentHint": False, "openWorldHint": False}),
    "recall_memory": ("What the robot remembers about where it is now: failed or refused actions, obstacles found, items last seen, what worked, "
                      "recent positions (remembered and possibly out of date: current sensors and the safety guard always win). Call it before planning.",
                      {"type": "object", "properties": {"limit": {"type": "integer", "minimum": 1, "maximum": 20, "default": 5}}}, _RO),
    "task_memory": ("Episodic + semantic memory for a NEW task: past task outcomes (succeeded/FAILED, with feedback) and verified strategies "
                    "relevant to the given instruction. EVIDENCE for planning only - current sensors and the safety guard override it, and a failed "
                    "past strategy should not be blindly repeated. Pass the user's instruction. Empty when memory is off or retrieval is disabled.",
                    {"type": "object", "properties": {"instruction": {"type": "string", "description": "the task instruction to look up"},
                                                      "limit": {"type": "integer", "minimum": 1, "maximum": 10, "default": 3}}}, _RO),
    "search_github": ("Search public GitHub repositories on the internet (read-only, does not move the robot). Give GitHub search terms in query "
                      "(you may use qualifiers like language:python or stars:>100); limit 1..20 results; sort = best (relevance), stars, updated or forks. "
                      "Returns each repository's full_name, description, url, stars, forks, language and last update. Needs internet access.",
                      {"type": "object", "properties": {"query": {"type": "string", "description": "GitHub search terms, e.g. 'nao humanoid robot language:python'"},
                                                        "limit": {"type": "integer", "minimum": 1, "maximum": 20, "default": 5},
                                                        "sort": {"type": "string", "enum": ["best", "stars", "updated", "forks"], "default": "best"}},
                       "required": ["query"]},
                      {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True, "openWorldHint": True}),
}

class HumanoidMcpServer(McpServer):
    """The same MCP server, offering the NAO humanoid's tools (orbit_ros/humanoid.py) in the house (orbit_ros/house.py)."""
    instructions = HUMANOID_INSTRUCTIONS
    def tool_defs(self) -> list:
        out = [humanoid_tool_def(t) for t in self.s.robot_tools()]
        for name, (desc, schema, ann) in H_SESSION_TOOLS.items():
            out.append({"name": name, "description": desc, "inputSchema": schema, "annotations": {"title": name.replace("_", " "), **ann}})
        return out

    def _manifest_blocked(self):
        """Feature 11: when ORBIT_TOOL_MANIFEST_CHECK=1, verify the live tool catalogue against configs/tool_manifest.lock.
        Returns a reason string if robot actions must be BLOCKED (fail-closed), else None. Checked once, then cached."""
        import os
        if os.getenv("ORBIT_TOOL_MANIFEST_CHECK", "") not in ("1", "true", "True"):
            return None
        cached = getattr(self, "_manifest_reason", "unchecked")
        if cached != "unchecked":
            return cached
        try:
            from orbit_ros.humanoid import HUMANOID_TOOLS
            from orbit_ros.metatool import tool_manifest as tm
            ok, detail = tm.verify(HUMANOID_TOOLS)
            self._manifest_reason = None if ok else detail
        except Exception as e:
            self._manifest_reason = f"tool manifest check failed to run: {e}"
        return self._manifest_reason

    def call_tool(self, name, args):
        if name in self.s.robot_tools():
            blocked = self._manifest_blocked()
            if blocked:
                return {"status": "error", "message": "robot actions blocked: " + blocked,
                        "tool": name}, True
            r = self.s.robot_call(name, args); return r, r["status"] != "ok"
        if name == "list_scenes":
            return [{"name": k, "entry_point": list(v[0][:2]), "furniture": sorted({it["label"] for it in house.ITEMS if house.room_of(it["x"], it["y"]) == k})}
                    for k, v in house.ROOMS.items()], False
        if name == "load_scene": return self.s.load_scene(str(args.get("scene", "living_room"))), False
        if name == "house_info": return self.s.house_info(), False
        if name == "get_status": return self.s.status(), False
        if name == "get_map": return self.s.believed_map(), False
        if name == "set_no_go_zone": return self.s.set_no_go_zone(args.get("x"), args.get("y"), args.get("radius", 0.5)), False
        if name == "get_capabilities": return self.s.capabilities(), False
        if name == "connection_status": return self.s.connection_status(), False
        if name == "recall_memory": return self.s.recall_memory(int(args.get("limit", 5))), False
        if name == "task_memory": return self.s.task_memory(instruction=args.get("instruction"), limit=int(args.get("limit", 3))), False
        if name == "search_github":
            r = self.s.search_github(args.get("query"), args.get("limit", 5), args.get("sort", "best"))
            return r, r.get("status") == "error"
        if name == "debug_view": return self.s.debug_view(), False
        if name == "camera":
            r = self.s.camera(mode=args.get("mode", "overview"), zoom=args.get("zoom"), target=args.get("target"))
            return r, r.get("status") == "error"
        if name in ("validate_plan", "execute_plan", "plan_and_execute"):
            return self._metatool(name, args)
        raise KeyError(name)

    def _metatool(self, name, args):
        """Meta-tool MCP tools. validate_plan never moves the robot; execute_plan/plan_and_execute go through the guarded executor."""
        from orbit_ros.humanoid import HUMANOID_TOOLS
        from orbit_ros.metatool import spec as mspec
        from orbit_ros.metatool.validator import validate
        from orbit_ros.metatool.plan import Plan
        if name == "validate_plan":
            issues = validate(Plan.from_dict(args.get("plan") or {}), HUMANOID_TOOLS, mspec.facts(self.s))
            return {"status": "ok", "issues": [i.as_dict() for i in issues], "admissible": not issues}, False
        from orbit_ros.metatool.executor import MetaToolAgent, Config
        # Feature 11: fail-closed manifest gate also protects the metatool executing paths.
        if name in ("execute_plan", "plan_and_execute"):
            blocked = self._manifest_blocked()
            if blocked:
                return {"status": "error", "message": "robot actions blocked: " + blocked}, True
        llm = None
        try:
            from orbit_ros.metatool.llm_io import LlmClient
            import os
            prov = os.getenv("ORBIT_PROVIDER", "groq")
            c = LlmClient(provider=prov)
            if c.available(): llm = c
        except Exception:
            llm = None
        agent = MetaToolAgent(self.s, HUMANOID_TOOLS, llm=llm, memory=getattr(self.s, "memory", None), config=Config())
        if name == "execute_plan":
            plan = Plan.from_dict(args.get("plan") or {})
            issues = validate(plan, HUMANOID_TOOLS, mspec.facts(self.s))
            if issues:
                return {"status": "rejected", "issues": [i.as_dict() for i in issues], "note": "not executed; fix the issues"}, True
            rr = agent._execute(plan, plan.goal or "")
            payload = {"status": "ok" if rr.success else "failed", "result": _runresult_dict(rr)}
            return payload, not rr.success
        # plan_and_execute
        rr = agent.run(str(args.get("instruction", "")))
        payload = {"status": "ok" if rr.success else (rr.status), "result": _runresult_dict(rr)}
        return payload, not rr.success and rr.status not in ("refused", "deferred")


def _runresult_dict(rr):
    """Serialise a metatool RunResult for an MCP reply."""
    return {"success": rr.success, "status": rr.status, "steps": rr.steps, "said": rr.said,
            "metrics": rr.metrics, "plan": rr.plan}
