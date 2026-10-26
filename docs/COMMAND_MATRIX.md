# ORBIT ROS NAO — Command Compatibility Matrix (v0.16.5)

Authoritative inventory, generated from the live registry (`HUMANOID_TOOLS`) and session tools (`H_SESSION_TOOLS`) and confirmed through
the live MCP `tools/list`: **49 tools = 33 robot tools + 16 session tools** (v0.16.5, Phase 16).

Session tools added since v0.16.4: `connection_status`, `task_memory`, the MetaTool tools `validate_plan`, `execute_plan`,
`plan_and_execute` (Phase 12; see `docs/METATOOL_SYSTEM.md`), and the display-only `camera` (Phase 16; see `docs/VIEWPOINT_SYSTEM.md`).
The live `tools/list` was compared against the registry and matches (49, confirmed by the Phase-16 live self-test).

- **Transport path** for every tool: MCP client → stdio JSON-RPC → `orbit_mcp_server.py` → `HumanoidSession` → (robot tools) registry +
  `HumanoidGuard` → `WebotsNao` on `tcp://127.0.0.1:1235`. No port 1240.
- **Validated** column: `unit` = covered by the pytest suite (511 passed, 2 skipped); `live` = also exercised in the live Webots self-test
  and/or the live movement/reliability runs on 2026-10-24 (see `TASKS.md` and `docs/RELIABILITY_REPORT.md`).
- Robot tools pass argument validation + preconditions + the safety guard. Session tools are read-only and never move the robot.

## Robot tools (registry: `orbit_ros/humanoid.py`)

| Tool | Category | Arguments (`?` = optional) | Preconditions | Idempotent | Validated |
|---|---|---|---|---|---|
| walk_forward | motion | steps:int 1..6 | not_stopped & upright | no | unit, live |
| walk_backward | motion | steps:int 1..3 | not_stopped & upright | no | unit |
| turn | motion | direction:enum(left/right), degrees:enum(40/60/180) | not_stopped & upright | no | unit, live |
| side_step | motion | direction:enum(left/right) | not_stopped & upright | no | unit |
| go_to | macro | x:number, y:number (metres) | not_stopped & upright | no | unit, live (rejected-outside) |
| go_to_room | macro | room:enum | not_stopped & upright | no | unit, live (chat) |
| go_to_object | macro | name:enum | not_stopped & upright | no | unit |
| turn_to_object | macro | name:enum | not_stopped & upright | no | unit |
| look_around | macro | (none) | not_stopped & upright | yes | unit |
| wave | effector | (none) | not_stopped & upright | yes | unit, live |
| greet | effector | (none) | not_stopped & upright | yes | unit, live |
| join_tv_watching | macro | (none) | not_stopped & upright | no | unit, live |
| get_pose | sensing | (none) | - | yes | unit, live |
| describe_view | sensing | (none) | not_stopped | yes | unit, live |
| look | sensing | (none) | not_stopped | yes | unit |
| clean | motion | mode:enum(vacuum/mop) | not_stopped & upright | no | unit |
| clean_room | macro | room:enum, pattern:enum?, mode:enum? | not_stopped & upright | no | unit |
| clean_house | macro | pattern:enum?, mode:enum? | not_stopped & upright | no | unit |
| clean_corners | macro | room:enum?, mode:enum? | not_stopped & upright | no | unit |
| clean_direction | motion | direction:enum, steps:int?, mode:enum? | not_stopped & upright | no | unit |
| scan_obstacles | sensing | (none) | not_stopped | yes | unit |
| motion_accuracy | sensing | (none) | - | yes | unit, live |
| verify_cleaning | sensing | (none) | - | yes | unit |
| verify_task | sensing | (none) | - | yes | unit |
| clean_status | sensing | (none) | - | yes | unit |
| pause_cleaning | safety | (none) | - | yes | unit |
| resume_cleaning | safety | (none) | - | yes | unit |
| return_to_start | macro | (none) | not_stopped & upright | no | unit |
| tour_house | macro | finish:enum?(stay/start) | not_stopped & upright | no | unit |
| plan_tasks | sensing | steps:list, order:enum?, finish:enum?, pattern:enum? | not_stopped | yes | unit |
| run_plan | macro | steps:list, order:enum?, finish:enum?, pattern:enum? | not_stopped & upright | no | unit |
| stand_up | safety | (none) | not_stopped | no | unit |
| stop_robot | safety | (none) | - | yes | unit, live (latch) |

## Session tools (read-only; `orbit_ros/mcp_server.py` `H_SESSION_TOOLS`)

| Tool | Purpose | Validated |
|---|---|---|
| list_scenes | List rooms `load_scene` can place the robot in | unit |
| load_scene | Stand the robot at a room entry; the only restart after stop_robot (destructive hint) | unit, live |
| house_info | House plan: rooms + furniture with positions/sizes | unit, live |
| get_status | Pose, fallen/stop state, bumps, no-go zones, call counts, last veto | unit, live |
| get_map | Text map of the house as the robot knows it | unit |
| set_no_go_zone | Declare a circle the robot must never enter | unit, live |
| get_capabilities | Robot + tools with arg limits + rooms | unit |
| **connection_status** (new v0.16.5) | Transport/controller health: backend, direct 1235 connection, ready/stopped, pose, call budget | unit, live |
| debug_view | House map with route/target/obstacles + task status | unit |
| **camera** (new Phase 16) | Adaptive display camera: overview/follow/closeup/target/human/demo/reset + zoom (dolly). Display only — never moves the robot, calls the LLM or changes memory; safe no-op in `--sim fake` | unit, live |
| recall_memory | Advisory memory brief (sensors + guard always win) | unit |
| search_github | Public GitHub repo search (read-only, internet) | unit |

## Chat commands (`orbit_chat.py`, not model tools)

These are typed directly into `orbit_chat.py` and handled **without** the LLM, so they are never mis-parsed as a robot task:

| Command | Effect |
|---|---|
| `/view` | overview framing (angled bird's-eye of the house) |
| `/view overview\|follow\|closeup\|target\|human\|demo\|reset` | set the named camera framing via the `camera` MCP tool |
| `/view target X Y` | frame the NAO together with the point (X, Y) |
| `/view zoom in\|out\|<number>` | dolly the current framing closer/further |
| `/view <mode> zoom in\|out\|<number>` | a framing mode plus a dolly in one command |

`/view` drives the display-only `camera` tool only: no robot motion, no LLM call, no memory change. (`/camera …` is accepted as an alias.)

## Result status contract

Every tool returns JSON with `status` one of:
`ok` · `rejected` (invalid args / precondition — not sent) · `vetoed` (unsafe — not sent) · `failed` (sent, robot could not do it) ·
`error` (simulator/transport problem). Anything but `ok` is flagged `isError: true` to the MCP client.

## Known limitations

- Only one extern controller can own the NAO at a time: run the live MCP server or the standalone benchmark, not both at once.
- Reconnection requires a **new process** (Webots allows one Supervisor per process); a lost connection surfaces as `{status:error}` and the
  next MCP session reconnects (~0.5 s). See `docs/RELIABILITY_REPORT.md`.
- Coverage/cleaning is simulated from brush paths, not a dirt sensor.
- `look` (camera image) is Webots-only; `FakeNao` returns no picture.
- Perception (`describe_view`) is map-based (61° cone, 5 m, walls occlude), not image recognition.

## Behavior under connection loss (v0.16.5)

Every robot tool returns a clean `{status:error}` (never raises) if the Webots connection is lost; the error message is actionable
("restart the Webots house ... start a new MCP session"). The stop-latch and argument validation still apply during a loss, and read-only
session tools (`house_info`, `list_scenes`, ...) keep working. Verified by `tests/test_humanoid.py::TestConnectionLoss` (fake) and a live
fault-injection run (see `docs/RELIABILITY_REPORT.md`).



## Phase 17 — MetaTool behaviour notes (no new MCP tools)

Phase 17 added no new MCP tools; it strengthened the existing `plan_and_execute` / `execute_plan` path. Behaviour
changes a client may observe (all config-switched, defaults preserve prior behaviour):

- A plan that both refuses and requests motion is now rejected before any motion (`REFUSAL_MISMATCH`), reinforcing the
  "zero motion for a refused plan" guarantee.
- With the deterministic safety gate enabled (`Config.use_safety_gate`), `plan_and_execute` may return status
  `deferred` (a clarification question, zero motion) in addition to the existing `refused`; both are treated as
  non-error by the MCP layer.
- Optional tool-manifest integrity: set env `ORBIT_TOOL_MANIFEST_CHECK=1` to make the server verify the registered
  tool catalogue against `configs/tool_manifest.lock` at first robot action and block robot tools /
  `execute_plan` / `plan_and_execute` on a mismatch (fail-closed). Off by default; MCP stdio transport unchanged.

Deliberate manifest update procedure: after intentionally adding/removing a tool, run
`python -c "from orbit_ros.humanoid import HUMANOID_TOOLS as H; from orbit_ros.metatool import tool_manifest as tm; tm.write_lock(H)"`
and commit the new `configs/tool_manifest.lock`. Full detail: `docs/PHASE17_RESEARCH_UPGRADES_REPORT.md`.
