# ORBIT MCP server: legacy TurtleBot backend and current NAO interface

`orbit_mcp_server.py` exposes the NAO tools through MCP; the supported Webots world is `webots/worlds/orbit_house.wbt`. Start Webots
with `.\start_webots.ps1`, then start the NAO server with `--robot nao --sim webots --keep-alive`.
As of v0.16.5 the MCP server **is itself the single Webots extern controller** and connects to Webots directly on the controller
endpoint `tcp://127.0.0.1:1235`. There is no relay process and **no TCP port 1240**: the MCP client keeps the one server process
alive over stdio for the whole session, so the extern controller stays connected. `--keep-alive` additionally holds the controller
connected after the AI client disconnects or a self-test finishes (it keeps stepping the world until Webots closes or Ctrl+C, and
retries once per second on a transient error); use `.\run.ps1 -Mode hold` to hold the controller in a standalone window. The old
TurtleBot Webots world, controller and VS Code MCP entry were removed. The TurtleBot/World2D material below is retained as
research/reference code only; it is not the project's current robot controller. For NAO setup, safety behavior and available
commands, see README sections 4 and 5.

**Status.** Tested here with World2D and FakeSim (grid and continuous motion), with a scripted client and with the official MCP Python
SDK client (versions 1.30 and 2.2). **Not run in Webots, and not run with Claude Desktop or VS Code**: neither is available in the
environment where this release was built. On the user's computer the Webots C1 gate currently fails (5 of 192 episodes agree with
World2D, gate needs 95%; 187 first-guessed as perception), so camera answers in Webots cannot be trusted yet (see "Webots" below).

## Where it came from

The supervisor pointed to `premananda108/webots-mcp`, an MCP server for a NAO humanoid in Webots. What was taken over and what was not:

| webots-mcp (NAO) | ORBIT (TurtleBot3) | why |
|---|---|---|
| FastMCP server started by the AI client over stdio | own stdio server, standard library only | no extra packages on Windows; ORBIT stays standard library + numpy. The protocol is the same; tested with the official SDK client |
| separate Webots controller, talks to the server through `commands.json` / `status.json` files | the server process IS the Webots extern controller and reuses the existing `WebotsSim` adapter (`--sim webots`) | no file polling or timing races; the same adapter the benchmark uses, so World2D and Webots behave the same |
| NAO tools: head, arms, LEDs, `.motion` files, camera image | TurtleBot3 tools from the PEL registry: move_forward, turn, clean, describe_view, get_pose, report, stop_robot, goto_cell, cover_region (+ localize) | a wheeled robot; the registry is ORBIT's single list of robot tools |
| values clamped silently (`max(-1, min(1, yaw))`) | out-of-range values are REJECTED with a message | a silently changed command is not what the person asked for |
| no safety layer | every robot call goes through the PEL registry and safety guard | see below |
| `get_robot_capabilities` | `get_capabilities` (generated from the registry) | |

## The one path to the robot

```
AI client -> tools/call -> OrbitMcpSession.robot_call
           -> offered tool?  -> call budget (default 500 per scene)
           -> registry.validate_call (name, typed arguments, preconditions)    rejected = not sent
           -> SafetyGuard.check (room bounds, believed obstacles, no-go cells, at most 12 cells per move)   vetoed = not sent
           -> world.execute (World2D / FakeSim / Webots / ROS 2 adapter)
           -> belief update (seen objects, blocked cells, cleaned cells) -> JSON result to the AI client
```

* `goto_cell` and `cover_region` are expanded into primitive calls; **each primitive is checked again** before it is sent, so a macro is
  never a way around the guard.
* The registry and the guard cannot be switched off from the server or from the AI client.
* The benchmark agent tools `ask`, `abstain`, `remember`, `done`, `replan` are not offered: the AI client is the agent and talks to the
  person directly.
* The guard knows only what the robot reported (pose, camera, blocked moves) and the no-go cells the person declared. An object the
  robot has never seen is not in the guard's map: the simulator then stops the move (`failed: blocked after k m`) and the cell is added.
  Call `describe_view` before driving.
* `stop_robot` latches: every later motion is rejected until `load_scene`.
* Tests: `tests/test_mcp_server.py`. The key one, `test_no_bypass_random_calls`, sends 600 random, mostly malformed calls in World2D and
  FakeSim (grid and continuous) and asserts that every call that reached the robot was admissible for the registry and the guard at
  that moment, and that the robot never collided.

## Tools

Robot tools (from `orbit_ros/pel/registry.py`):

| tool | arguments | notes |
|---|---|---|
| `move_forward` | `distance` integer 0..12 | cells (1 m) along the heading |
| `turn` | `direction` left/right, `degrees` 90/180/270 | left = counter-clockwise |
| `clean` | none | cleans the current cell |
| `describe_view` | none | label, image_x, apparent_size, plus bearing_deg, distance_m, estimated world_xy |
| `get_pose` | none | x, y, yaw_deg, room_w, room_h |
| `report` | `answer` | answer to a question in the task |
| `stop_robot` | none | latching stop |
| `goto_cell` | `x`, `y` | shortest known-free path; off with `--no-nav-tools` |
| `cover_region` | `side` left/right/front/back/all, `avoid` [[x, y], ...] | off with `--no-nav-tools` |
| `localize` | none | only with `--localizer` |

Session tools (never move the robot): `list_scenes`, `load_scene` (demo, empty, task:&lt;family&gt;), `get_status`, `get_map`
(believed map as text), `set_no_go_cells`, `get_capabilities`, `check_task` (evaluator score of a task scene; uses ground truth).

Every result is JSON with `status` = `ok`, `rejected` (invalid, not sent), `vetoed` (unsafe, not sent), `failed` (sent, the robot could
not do it) or `error` (simulator problem). Anything but `ok` is returned as an MCP tool error (`isError: true`) so the AI client notices.

## Command line

```
python orbit_mcp_server.py [--sim world2d|fake|webots|ros2] [--motion grid|continuous] [--perception camera|truth]
                           [--scene demo|empty|task:<family>] [--no-nav-tools] [--localizer] [--max-calls 500] [--log-dir DIR]
python orbit_mcp_server.py --selftest [same options]     # starts the server like an AI client and runs 12 scripted calls
```

stdout carries only the protocol (newline-delimited JSON-RPC 2.0; protocol versions 2024-11-05 to 2025-11-25). At start the server
points file descriptor 1 at stderr, so `print` and Webots' own messages cannot corrupt it. Every call is logged to
`results_mcp/session_<UTC>_<pid>.jsonl` (call, registry issues, guard verdict, result).

## Webots

Only the NAO house is supported: run `.\run.ps1 -Mode server` (opens the house) and use `--robot nao --sim webots`
(`orbit-nao-webots` in `.vscode/mcp.json`); the server connects to Webots directly on `tcp://127.0.0.1:1235`. The TurtleBot Webots
world was removed, so `--sim webots` without `--robot nao` has no world to open.

Webots finds itself like the rest of ORBIT (`WEBOTS_HOME` in `.env`, then `C:\Program Files\Webots` and
`%LOCALAPPDATA%\Programs\Webots`). Without Webots the first robot call returns a tool error "Webots not found ..."; the server keeps
running and never falls back to World2D silently.

## NAO humanoid in the house (v0.16)

`python orbit_mcp_server.py --robot nao [--sim webots|fake] [--scene living_room|kitchen|bedroom|bathroom]` offers a NAO humanoid
walking in a furnished one-floor house instead of the TurtleBot3. This is the robot of the supervisor's reference project
(`premananda108/webots-mcp`), built on ORBIT's own checks. NAO MCP client configurations and chat default to live Webots on the
direct extern-controller endpoint `tcp://127.0.0.1:1235`; `fake` is reserved for isolated unit tests and must be selected explicitly.

* House: `orbit_ros/house.py` is the only definition. `tools/make_house_world.py` writes `webots/worlds/orbit_house.wbt` from it, and
  the same data is the robot's map for the guard and the path planner, so the picture and the map cannot drift apart.
* Cleaning: the Webots scene has a visible floor attachment with vacuum and mop geometry. `clean` chooses the mode for the current
  pass; `clean_room`, `clean_house`, `clean_corners`, and `clean_direction` accept an optional `mode` (`vacuum` or `mop`). The selected
  mode persists until changed or a new scene is loaded. Coverage is estimated from simulated brush paths, not a dirt sensor.
* Path to the robot: tools/call -> call budget -> `validate_humanoid_call` (tool, argument types and ranges, stopped, fallen) ->
  `HumanoidGuard.check` for every single motion (walls, furniture, bumped obstacles, no-go zones, ROBOT_R + MARGIN = 0.3 m from the
  reported pose) -> Webots `.motion` file. `go_to`, `go_to_room` and `go_to_object` plan a path (A*, 0.1 m grid) and send one checked
  motion at a time; a bump adds an obstacle and the path is planned again.
* Tools: walk_forward (1..6 cycles), walk_backward (1..3), turn (left/right, 40/60/180), side_step, go_to (x, y), go_to_room, go_to_object, turn_to_object (face an item in place without walking),
  tour_house (all rooms, optionally return to start), join_tv_watching (take a safe spot beside the sofa and face the television), wave/greet (hand-wave motion), get_pose, describe_view (items in the 61-degree view, hidden behind walls), look (camera picture, Webots only), stand_up,
  stop_robot; session tools list_scenes, load_scene, house_info, get_status, get_map, set_no_go_zone, get_capabilities.
* Cleaning tools include `clean_room` (navigate to and sweep one room), `clean_house`, `clean_corners`, `clean_direction`, `clean_status`,
  `pause_cleaning`, `resume_cleaning` and `return_to_start`. “Deep-clean the bathroom”, “tour the house and come back”, “greet me”
  Greet performs a hand-wave and does not produce speech.
* Webots: `.\run.ps1 -Mode server` (opens the house), then `orbit-nao-webots` in `.vscode/mcp.json` (the server connects directly on `tcp://127.0.0.1:1235`). The `.motion` files
  are read from `WEBOTS_HOME\projects\robots\softbank\nao\motions` (or `webots/motions/`) and played by driving the joint motors. Distances per walking cycle in Webots differ
  from FakeNao; the session reads the pose back after every motion. If one of Webots' furniture objects does not exist in your Webots
  version, only that item is missing from the picture (it is still on the robot's map).

### v0.16.1: real Webots objects, and a test client with your own API key

* The house uses Webots R2025a's own objects (checked against the R2025a PROTO files: each one exists and its size was read from the
  PROTO): Wall and Window (2.4 m walls, 5 windows), CeilingLight in every room, Parquetry / tile / carpet floors, Sofa, Armchair,
  Television, FloorLight, BunchOfSunFlowers, Fridge, Sink, HotPlate, CookingPot, Chair, Plate, Apple, Orange, Clock, Bed, Desk,
  OfficeChair, Laptop, Bathtube, Toilet, BathroomSink, WashingMachine, RubberDuck, PottedTree. There is no ceiling, so the default view
  from above sees into every room. `RectangleArena` is not used: the house has its own floor and walls.
* `orbit_chat.py` is a minimal MCP client: it starts this server over stdio, gives its tools to a model (Gemini, Groq, Cerebras,
  OpenRouter or OpenAI, through their OpenAI-compatible chat endpoint, key from `.env`) and runs every tool call through the server.
  Tested here against a local fake model server (`tests/test_orbit_chat.py`); NOT run against a live provider by us.
