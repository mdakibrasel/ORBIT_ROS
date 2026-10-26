# ORBIT ROS NAO — System Architecture & Execution Flow (v0.16.5)

## Components

| Component | File | Role |
|---|---|---|
| MCP client | Claude Desktop / VS Code Agent / Gemini CLI / `orbit_chat.py` | Spawns the MCP server over **stdio** and keeps it alive for the whole session. |
| MCP server | `orbit_mcp_server.py` | Entry point. Parses args, sets up the stdio JSON-RPC transport, builds the session, serves tools. |
| JSON-RPC / tools layer | `orbit_ros/mcp_server.py` (`HumanoidMcpServer`) | MCP `initialize`/`tools/list`/`tools/call`. Maps tool names to session methods; builds tool schemas from the registry. |
| Session (core logic) | `orbit_ros/humanoid.py` (`HumanoidSession`) | The single path to the robot: call budget -> argument validation -> preconditions -> safety guard -> motion. Owns pose, cleaning, memory, planning. |
| Tool registry | `orbit_ros/pel/registry.py` + `HUMANOID_TOOLS` | Authoritative list of robot tools with typed `Arg`/`ToolSpec` definitions; re-checks every argument. |
| Safety guard | `HumanoidGuard` (in `humanoid.py`) | Vetoes any motion that would hit a wall/furniture/bumped obstacle or enter a no-go zone, predicted from the **reported** pose. Cannot be switched off. |
| Robot backend | `WebotsNao` / `FakeNao` (in `humanoid.py`) | `WebotsNao` is a Webots R2025a extern **Supervisor** controller; `FakeNao` is a kinematic stand-in for unit tests. |
| House model | `orbit_ros/house.py` | Single source of truth for rooms, furniture, the robot's map and the A* planner. `tools/make_house_world.py` writes the `.wbt` from it. |
| Webots world | `webots/worlds/orbit_house.wbt` | The furnished one-floor house (kitchen, bathroom, bedroom, living room). The NAO is an `<extern>` controller named `orbit_nao`. |
| Memory | `orbit_ros/nao_memory.py` | Advisory SQLite store (`recall_memory`); live sensors + guard always win. v2 (Phase 10) adds episodic task memory + semantic strategies + `task_memory` retrieval — still evidence-only. See `docs/MEMORY_MODULE.md`. |

## Transport (v0.16.5 — port 1235 only)

```
+-------------+   stdio (JSON-RPC 2.0)   +----------------------+   tcp://127.0.0.1:1235   +------------------+
| MCP client  | <----------------------> |  orbit_mcp_server.py | <----------------------> |  Webots R2025a   |
| (LLM agent) |   newline-delimited      |  = HumanoidSession   |   extern controller      |  orbit_house.wbt |
|             |                          |    = WebotsNao       |   'orbit_nao'            |  (NAO + house)   |
+-------------+                          +----------------------+                          +------------------+
```

- The MCP server **is** the single Webots extern controller. It connects in-process via `WEBOTS_CONTROLLER_URL = tcp://127.0.0.1:1235/orbit_nao`.
- The MCP client keeps that one server process alive for the whole session, so the controller stays connected (Webots drops an extern
  controller only when its process ends). `--keep-alive` keeps Webots connected after a one-shot self-test ends.
- **No TCP port 1240 and no relay process exist anywhere.** The former `orbit_bridge.py` relay (port 1240) is retired; `orbit_ros/remote.py`
  survives only as the serialization library for a unit test and uses `PORT = 0` (disabled).
- Only **one** extern controller can own the NAO at a time (Webots limitation). Run either the live MCP server or the standalone
  `tools/nao_benchmark.py` — not both simultaneously.

## The one path from a tool call to the robot

```
tools/call (name, args)
  -> HumanoidMcpServer.call_tool
     - session tool (read-only: get_status, connection_status, house_info, ...) -> HumanoidSession method, returned directly
     - robot tool -> HumanoidSession.robot_call:
         1. call budget (max_calls per scene; runaway protection)
         2. validate_humanoid_call  (tool exists, typed args in range)        -> "rejected" (NOT sent)
         3. preconditions           (not stopped; upright unless safety tool) -> "rejected"
         4. HumanoidGuard.check     (bounds, furniture, bumps, no-go zones)   -> "vetoed"   (NOT sent)
         5. backend motion          (WebotsNao plays the .motion / Supervisor move)
         6. pose read back from the simulator; belief + memory update
  -> JSON {status: ok|rejected|vetoed|failed|error, ...} back to the agent (isError set for anything but ok)
```

- Macros (`go_to`, `go_to_room`, `go_to_object`, `tour_house`, `run_plan`, `clean_*`) expand into primitive motions and **each primitive is
  checked again** by the guard, so a macro is never a way around safety.
- `stop_robot` latches: every later motion is rejected until `load_scene`.
- Walking is open-loop: distances per Webots walking cycle vary, so the session reads the true pose back after every motion instead of
  assuming how far the robot went.

## Startup

1. `.\start_webots.ps1` (or `.\run.ps1 -Mode server`) opens `orbit_house.wbt` in realtime mode on controller port 1235; the NAO waits
   for its extern controller.
2. The MCP client launches `orbit_mcp_server.py --robot nao --sim webots` (over stdio); the server connects directly on 1235 and the NAO
   starts responding. `--keep-alive` keeps Webots connected after a self-test.

## Failure handling

- A transient `WebotsNao.step() == -1` (world paused/reloaded) raises `RuntimeError("Webots simulation ended")`; `robot_call` surfaces it as
  a `{status: error}` result rather than crashing the server, so the agent can retry. (Reliability-hardened in v0.16.5: the pose read used
  for validation and the `_trace` audit path are both resilient to a dead backend, so a lost connection never raises out of `robot_call`;
  the stop-latch and argument validation still fire. See `docs/RELIABILITY_REPORT.md`.)
- `connection_status` reports whether the controller is actually connected (real `sup.getSelf()`), not merely whether a port is open.
- `look`/camera and perception are computed from the house map (not image recognition), so a missing Webots furniture PROTO only removes
  that item from the picture; the map and guard are unaffected.

## Controller lifecycle (confirmed Phase 8)
- The extern controller **exits at the end of each MCP session**; it is not resident. Webots keeps the robot/world and, on controller exit,
  returns port 1235 to LISTENING and re-accepts the next controller.
- **Reconnection = a new process.** Webots allows only one `Robot`/`Supervisor` per controller process
  (`Error: only one Robot instance can be created per controller process.`), so a session never re-instantiates `WebotsNao` in-process;
  a fresh MCP session (fresh process) reconnects. Measured reconnect latency ~0.5–0.6 s (3/3 in the Phase 8 benchmark).

## Memory layer (Phase 10, v0.16.6)

`orbit_ros/nao_memory.py` (`NaoMemory`, SQLite) is a **leaf module** (imports nothing from the robot/planner layers; the session passes it
the house vocabulary). Full design: `docs/MEMORY_MODULE.md`.

- **Working memory**: session fields (`last_task`, `_dbg`, `replans`, live `_pose()`) — current process only.
- **Episodic memory** (`episodes` table): one row per task — instruction, normalized goal, start/final pose, planned vs executed, outcome,
  success, replans, feedback, provenance.
- **Semantic memory** (`strategies` table): distilled `(goal, tool, args)` with success/failure counts, confidence, `verified` flag.
- Schema is versioned (`meta.schema_version=2`); v1 dbs migrate in place; WAL + busy-timeout make writes recoverable.

Pipeline with memory (extends "the one path"):
```
instruction -> live pose -> task_memory(instruction) retrieval -> brief prepended to the REAL LLM request
            -> LLM picks MCP tools -> argument validation -> HumanoidGuard -> real motion -> observe true pose
            -> record_episode(...) + learn_strategy(...) write-back -> next task can retrieve it
```
Invariants (tested + live-verified in Phase 10): current observations and the guard always override memory; memory cannot bypass safety,
the stop-latch, or argument validation; retrieval failure returns empty (never fabricated); a broken db leaves the robot operational.
Config: `--no-memory`, `--no-memory-retrieval`, `--no-memory-write`, `--memory-db`, `--memory-max-records`; the active config is printed at
startup (`[orbit-mcp] memory: ...`). Two read-only MCP tools expose memory to the LLM: `recall_memory` and `task_memory` (45 tools total).

## Adaptive viewpoint (Phase 12, v0.16.5)

`orbit_ros/viewpoint.py` adapts the Webots camera for visualization — **display only**, wired into the live harness, never affecting
physics, navigation, planning or outcomes. Modes: overview (top-down, default start/reset), follow (built-in Viewpoint `follow`/
`followType`/`followSmoothness`), target (frames NAO + target via a copied `look_at_rotation`), demo (follow + HUD). It uses the one
existing `WebotsNao` Supervisor (no second controller); a viewpoint change is rendered by advancing the existing controller a few frames
(`Viewpoint.render`), which does not command robot motion. Full design + live evidence: `docs/VIEWPOINT_SYSTEM.md`,
`docs/PHASE12_INTEGRATION_REPORT.md`. The NAO runtime does NOT import the TurtleBot/World2D legacy `webots_adapter` (the look-at math is copied).



## Phase 17 — MetaTool research upgrades (config-switched, safety-preserving)

New modules under `orbit_ros/metatool/` extend the plan→validate→execute→verify→repair loop. Every one has a
configuration switch, defaults to the prior behaviour, and does **not** bypass `HumanoidSession.robot_call` or the
`HumanoidGuard`. Full status, evidence paths and limitations: `docs/PHASE17_RESEARCH_UPGRADES_REPORT.md`.

| Module | Role | Switch | Status |
|---|---|---|---|
| `validator.py` (`REFUSAL_MISMATCH`) | refuse any plan that both refuses and requests motion (zero-motion invariant) | `validate(check_refusal=True)` | CODE-CHECK |
| `validator.py` (`INCOMPLETE`) | flag goal targets not addressed; drive bounded refinement | `Config.check_complete` | CODE-CHECK |
| `tool_retrieval.py` | show planner a tool subset; validator keeps full catalogue; bounded full-catalogue retry | `Config.use_tool_retrieval`, `retrieval_k` | CODE-CHECK |
| `safety_gate.py` | deterministic AUTHORIZE/DEFER/REJECT + Task Safety Contract (LLM cannot override abort) | `Config.use_safety_gate` | CODE-CHECK |
| `tool_manifest.py` | fail-closed SHA-256 tool-manifest integrity (`configs/tool_manifest.lock`) | env `ORBIT_TOOL_MANIFEST_CHECK=1` | CODE-CHECK |
| `prompts._as_data` + `SYSTEM_RULES` | fence untrusted names/memory as DATA (injection hardening) | always on | CODE-CHECK |
| `root_cause.py` | fixed failure taxonomy; attribute earliest contributing step | wired into `_repair` | CODE-CHECK |
| `voting.py` | consistency vote over normalized goals | `Config.vote_k>1` | CODE-CHECK |
| `judge.py` | advisory plan review (never overrides hard checks) | off by default | CODE-CHECK |
| `memory_view.py` | consolidated scene subgraph + short-term view (read-only) | read-only helper | CODE-CHECK |
| `monitors.py` | in-step runtime monitor **evaluator** (not wired into the live motion loop) | evaluator only | CODE-CHECK; live wiring BLOCKED |
| `clarify.py` | calibrated clarification (falls back to deterministic triggers with no dataset) | — | CODE-CHECK; calibration data BLOCKED |

Live acceptance for all of the above is BLOCKED this session (Webots not running); see the report's limitations.
