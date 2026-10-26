# ORBIT-ROS

**Safety-guarded NAO robotics control for Webots, MCP, and VS Code**

ORBIT-ROS connects a NAO humanoid to a furnished Webots environment through the
[Model Context Protocol (MCP)](https://modelcontextprotocol.io/). It provides typed robot tools,
guarded navigation and cleaning, persistent Webots control, memory-assisted planning, and a
Windows PowerShell workflow designed for reliable use from VS Code.

The project also includes a deterministic fake simulator for fast regression testing. Webots and
fake-simulator execution use the same validation, planning, and safety interfaces wherever possible.

> **Release status:** The project is prepared for an initial public release, but `v1.0.0` is not
> claimed yet. Full Webots close/restart reattachment and interactive Ctrl+C shutdown capture remain
> open verification gates. See [RELEASE_EVIDENCE_20261026.md](docs/RELEASE_EVIDENCE_20261026.md).

## Contents

- [Highlights](#highlights)
- [MCP tools](#mcp-tools)
- [MetaTool](#metatool)
- [Architecture](#architecture)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Quick start](#quick-start)
- [Health checks](#health-checks)
- [Connection lifecycle and recovery](#connection-lifecycle-and-recovery)
- [Testing and evidence](#testing-and-evidence)
- [Repository layout](#repository-layout)
- [Troubleshooting](#troubleshooting)
- [Security and limitations](#security-and-limitations)
- [Contributing](#contributing)

## Highlights

- Direct Webots external-controller connection at `tcp://127.0.0.1:1235`.
- Webots robot name `orbit_nao`.
- MCP tools for navigation, perception, cleaning, planning, memory, diagnostics, and safety.
- Persistent `--keep-alive` controller behavior.
- Bounded startup retries and safe reconnection after temporary Webots failure.
- Hard argument validation, room/furniture/no-go safety checks, stop latch, and call budget.
- Fake NAO backend for fast unit and integration tests.
- Windows PowerShell and `.venv` workflow.

## MCP tools

The live MCP registry exposes **49 tools**: **33 robot tools** and **16 session/MetaTool tools**.
The complete argument, category, precondition, idempotency, and validation matrix is maintained in
[docs/COMMAND_MATRIX.md](docs/COMMAND_MATRIX.md). The robot tools cover:

- Motion and navigation: walking, turning, side steps, room/object navigation, tours, and return-to-start.
- Cleaning: vacuum/mop actions, room/house/corner cleaning, directional cleaning, pause/resume, and verification.
- Sensing and state: pose, camera view, map-based scene description, obstacle scans, task status, and motion accuracy.
- Effectors and safety: wave, greet, stand up, stop latch, no-go zones, and guarded preconditions.

Session tools provide scene loading, house information, status, maps, capabilities, connection health, debug views,
adaptive display camera control, memory retrieval, and the MetaTool operations described below. Every robot action
passes typed argument validation, preconditions, the humanoid safety guard, and the call budget before it can reach
Webots. Invalid or unsafe calls return a structured status instead of being sent to the robot.

## MetaTool

The core MetaTool provides a plan-constrained path for natural-language tasks while preserving the same robot safety
path. Its three MCP operations are:

- `validate_plan`: validate tools, arguments, dependencies, and preconditions without moving the robot.
- `execute_plan`: execute an already-structured plan deterministically, with per-step verification and bounded repair.
- `plan_and_execute`: turn an instruction into a schema-constrained plan, validate and self-refine it before motion,
  then execute it through the guard and verification path.

Plans can explicitly refuse an unsafe or unsupported request. Refusals, invalid plans, stop-latch state, no-go zones,
fallen-robot preconditions, and tool-manifest mismatches fail closed; a refused plan cannot move the robot. Memory is
advisory only: current observations and the safety guard always take precedence.

Example MCP payloads:

```json
{"name":"validate_plan","arguments":{"plan":{"feasible":true,"steps":[]}}}
{"name":"execute_plan","arguments":{"plan":{"feasible":true,"steps":[]}}}
{"name":"plan_and_execute","arguments":{"instruction":"go to the kitchen"}}
```

The core MetaTool has **10/10 historical live acceptance scenarios passed** and **69 focused code tests passed**.
Advanced follow-up features are tracked separately in
[docs/METATOOL_ADVANCED_PLAN.md](docs/METATOOL_ADVANCED_PLAN.md) and are not all implemented or live-verified.

## Architecture

```text
VS Code MCP
    |
    v
tools/start_orbit_mcp.ps1
    |-- start or reuse Webots
    |-- wait for 127.0.0.1:1235
    v
orbit_mcp_server.py --robot nao --sim webots --keep-alive
    |-- MCP stdio protocol
    |-- registry + safety guard + memory
    v
Webots external controller: orbit_nao @ 127.0.0.1:1235
```

The MCP server is the single Webots external controller. There is no relay process. Only one process may own
`orbit_nao` at a time.

## Requirements

- Windows 10 or later.
- Python 3.10 or later; Python 3.13 is the safest choice for Webots compatibility.
- Webots R2025a, with `WEBOTS_HOME` set in the environment or `.env` when it is not installed in a standard location.
- PowerShell.
- VS Code with the Python and PowerShell extensions recommended by `.vscode/extensions.json`.
- An API key only for LLM chat examples. Robot control and tests do not require a key.

## Installation

From a fresh clone:

```powershell
cd C:\path\to\ORBIT-ROS
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup.ps1
```

The setup script creates `.venv`, installs [requirements.txt](requirements.txt), prepares `.env` from `.env.example` when needed,
and runs the project checks. Never commit `.env` or `.venv`.

If Python 3.14 cannot install a dependency, create the environment with Python 3.13:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

## Configuration

Copy `.env.example` to `.env` and set only the provider variables you use:

```powershell
Copy-Item .env.example .env
notepad .env
```

Keep API keys local. The repository's `.gitignore` excludes `.env`, audit logs, caches, virtual environments, and generated live
results.

## Quick start

The recommended launch path is the checked-in
[.vscode/mcp.json](.vscode/mcp.json), which invokes
[tools/start_orbit_mcp.ps1](tools/start_orbit_mcp.ps1). The wrapper provides one deterministic startup order:

1. Reuse an existing Webots instance, or start `start_webots.ps1`.
2. Poll port `1235` until it accepts a TCP connection.
3. Start the `.venv` MCP server with `--keep-alive`.
4. Keep the MCP process alive while VS Code is connected.

After opening the repository, start Agent mode in VS Code. No separate `run.ps1 -Mode server`
window is required. Do not also run `run.ps1 -Mode hold`; that would create a second controller owner.

Manual equivalent:

```powershell
.\tools\start_orbit_mcp.ps1
```

The wrapper has a bounded default timeout of 60 seconds. It prints `STARTING`, `READY`, `DISCONNECTED`, and `ERROR` diagnostics to
stderr, leaving stdout available for MCP JSON-RPC.

## Manual Webots and MCP commands

Start only Webots:

```powershell
.\start_webots.ps1
```

Start a persistent MCP controller after Webots is ready:

```powershell
.\.venv\Scripts\python.exe .\orbit_mcp_server.py --robot nao --sim webots --keep-alive
```

Hold the controller in the current terminal:

```powershell
.\run.ps1 -Mode hold
```

Use exactly one of the MCP server, `-Mode hold`, or a live benchmark as the `orbit_nao` controller.

## Health checks

Fast, non-connecting port probe:

```powershell
.\run.ps1 -Mode health
```

This exits `0` when port `1235` is listening and `2` when it is not. It does not take the Webots controller slot.

Before starting the persistent MCP server, verify the actual NAO Supervisor connection:

```powershell
.\run.ps1 -Mode health -VerifyRobot
```

Expected output includes:

```text
[health] READY: Webots and NAO controller responded
pose=(4.800, 3.100, 180.0 deg, fallen=False)
```

The MCP `connection_status` tool reports the same endpoint, backend, `connected`, `ready`, pose, fallen state, call budget, and
connection state.

## Connection lifecycle and recovery

- `STARTING`: the wrapper is locating or launching Webots.
- `WAITING`: Webots is starting or its controller endpoint is not ready.
- `CONNECTED`: `orbit_nao` is attached to the live Supervisor.
- `DISCONNECTED`: a Webots step or robot call failed.
- `RECONNECTING`: the server waits for port `1235` and creates one replacement Supervisor.
- `ERROR`: the bounded reconnect timeout expired or configuration is invalid.
- `STOPPING`: the user closes Webots, presses Ctrl+C, or stops the MCP process.

Reconnect timing is configurable:

```powershell
$env:ORBIT_WEBOTS_RECONNECT_TIMEOUT = "60"
$env:ORBIT_WEBOTS_RECONNECT_INTERVAL = "1"
```

After Webots restarts, the server restores the current scene entry pose. It never blindly retries a motion that may have partially
executed. A reconnect does not bypass argument validation, the safety guard, the stop latch, or the call budget.

To stop a session, stop the MCP process first, then close Webots. If a process remains, identify it before stopping it:

```powershell
Get-Process -Name webots,webots-bin -ErrorAction SilentlyContinue
Get-NetTCPConnection -State Listen -LocalPort 1235 -ErrorAction SilentlyContinue
Stop-Process -Id <identified-pid>
```

Do not use name-based process termination in scripts that may affect unrelated Webots sessions.

## Testing and evidence

Fast fake-simulator self-tests:

```powershell
.\.venv\Scripts\python.exe .\orbit_mcp_server.py --selftest --sim fake
.\.venv\Scripts\python.exe .\orbit_mcp_server.py --selftest --robot nao --sim fake
```

Focused MCP, humanoid, and Webots-discovery checks:

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_mcp_server tests.test_humanoid tests.test_webots_discovery
```

Complete non-live suite:

```powershell
.\.venv\Scripts\python.exe -m pytest -m "not live"
```

Live Webots checks require a running house and one available controller slot:

```powershell
.\.venv\Scripts\python.exe .\orbit_mcp_server.py --selftest --robot nao --sim webots --keep-alive
.\.venv\Scripts\python.exe .\tools\nao_benchmark.py --sim webots
```

The live self-test must end with `SELF-TEST PASSED`. Live tests are marked `live` and are not part of the fake/non-live suite.

### Verified test summary

The project has two different kinds of evidence. **Live results below used the real Webots R2025a world, the real `orbit_nao`
extern controller on TCP port `1235`, and, where stated, real Groq API calls.** They are not non-live tests. Software regression
results are listed separately. Counts are not added together because focused suites overlap and each live evidence pack contains
multiple files for one run.

#### Direct live Webots/NAO results

| Live test or benchmark | Recorded result | What was verified |
|---|---:|---|
| Phase 8 reliability and reconnection | **18/18 commands passed; 3/3 tasks passed; 3/3 fresh-process reconnections passed** | Real pose, turns, walks, expected controller close/reopen, safety vetoes, zero safety violations, zero unexpected disconnects; reconnection mean about `0.55 s`. |
| Phase 9 live strategy comparison | **36 live trial rows** | Open-loop: **6/10 judged passed**, 2 skipped; deterministic planner: **12/12 passed**; Groq LLM strategy: **12/12 passed**. Across all 36 trial conditions: **0 true safety violations**, **0 true collisions**, real pose-based judging. |
| Phase 10 persistent memory and real LLM evaluation | **Experiment A: 4/4; Experiment B: failure learned then retest succeeded; D: restart persistence passed; E: stale memory safely overridden** | Real Webots plus real Groq calls verified retrieval, process-restart persistence, learning from a genuine failure, observation-over-memory behavior, and guard enforcement. |
| Core MetaTool live acceptance | **10/10 scenarios passed** | Unknown tools rejected before motion, dependency references, guarded navigation, bounded recovery, loop guard, hazardous refusal, stop latch, no repeated completed steps, memory context, and real `orbit_chat.py --mode plan`. |
| Live adaptive viewpoint | **7 live checks verified** | Overview/follow/target/demo/reset behavior, distinct screenshots, real camera metadata, no robot movement during camera changes, and idle-render fix. |
| Live human-controller integration | **PASS** | Candidate Webots world preserved the house and NAO; human route moved `1.443 m` to within `0.3 m` of its goal; NAO drift was `0.0 m` and remained upright. |
| Live MCP/Webots self-test, 2026-10-26 | **`SELF-TEST PASSED`** | **49 tools listed**, **15 MCP commands** executed, argument rejection, no-go veto, outside-house rejection, stop latch, keep-alive persistence, and clean controller release. |
| Live wrapper and NAO motion check, 2026-10-26 | **PASS** | Wrapper started Webots, connected `orbit_nao`, executed `load_scene`, `get_pose`, and `walk_forward`; real pose changed from `(4.800, 3.100, 180.0°)` to `(4.703, 3.094, 179.8°)`, with the robot upright. |

These live results are historical, timestamped evidence under `results_live/` and `results_nao_benchmark/`; they were not
reclassified as non-live tests. The Phase 9 and Phase 10 reports explicitly separate their live trials from their software tests.

#### Software and deterministic regression results

| Software test area | Recorded result | What was verified |
|---|---:|---|
| Complete `.venv` regression discovery run | **628 tests, 1 skipped, 0 failed — `OK`** | Unit/integration coverage for safety, MCP, humanoid, memory, planning, recovery, bridge, PEL, viewpoint, camera, Webots discovery, MetaTool, and compatibility behavior. This is the software suite, not a Webots live run. |
| MetaTool focused code suite | **69 passed, 0 failed** | Plan schema, dependencies, references, validation, refusal consistency, retrieval, safety gates, loop prevention, budgets, trace metrics, prompt hardening, manifest integrity, and provider fallback bookkeeping. |
| MCP and humanoid lifecycle focused suite | **87 tests, 1 skipped, 0 failed — `OK`** | Protocol/configuration, registration, health, stale listener invalidation, reconnect logic, safety, and NAO session behavior. |
| Webots discovery/startup focused suite | **40 tests, 1 optional skip — `OK`** | Executable discovery, `WEBOTS_HOME`, duplicate-process protection, startup scripts, health probe, and VS Code wrapper configuration. |
| Historical fake-simulator benchmark runs | **9 runs; 87/97 recorded tasks passed** | Deterministic NAO house benchmark runs using the fake simulator. These are not live Webots results and are retained as historical benchmark evidence. |
| Historical Webots benchmark runs | **3 runs; 4/6 recorded tasks passed** | Direct Webots benchmark records from 2026-10-21. These small runs are separate from the larger Phase 9 live evaluation and are not combined with it. |
| Evidence inventory and duplicate validation | **737 indexed files, 0 exact duplicates** | Markdown, TXT, CSV, JSON/JSONL, XML/HTML, logs, screenshots, and other evidence indexed with SHA-256 references; duplicate copies removed only when a canonical copy was retained. |

#### Live limitations

The following are deliberately not marked as passed:

- Full in-process Webots close/restart and controller reattachment remains **incomplete**; a forced listener replacement can still block inside the synchronous Webots controller API.
- Interactive Ctrl+C shutdown capture was **not run**; PID-specific cleanup was verified.
- Gemini provider testing was **not run successfully** because Gemini was unreachable; the historical LLM live trials used real Groq calls.
- Advanced MetaTool features listed in [docs/METATOOL_ADVANCED_PLAN.md](docs/METATOOL_ADVANCED_PLAN.md) are not all implemented or live-verified.

See [docs/RELEASE_EVIDENCE_20261026.md](docs/RELEASE_EVIDENCE_20261026.md),
[docs/PHASE14_METATOOL_REPORT.md](docs/PHASE14_METATOOL_REPORT.md), and
[docs/TEST_REPORT_INDEX.md](docs/TEST_REPORT_INDEX.md) for commands, dates, source evidence, and limitations.

## Test evidence index

The repository keeps original reports and run artifacts unchanged. To discover and normalize evidence from Markdown, TXT, CSV,
JSON/JSONL, XML/HTML, logs, screenshots, and other files under the evidence directories, run:

```powershell
.\.venv\Scripts\python.exe .\tools\normalize_test_reports.py
```

This writes the searchable machine-readable register to
[`docs/TEST_REPORT_INDEX.json`](docs/TEST_REPORT_INDEX.json) and a readable summary to
[`docs/TEST_REPORT_INDEX.md`](docs/TEST_REPORT_INDEX.md). Each record retains the original filename and path, source format,
category, result, available command/framework/count metadata, SHA-256 checksum, duplicate group, and limitations. Ambiguous
results remain `UNVERIFIED`; they are not converted to passes. The generator also reports the discovered-versus-indexed count,
duplicates, unclassified records, and explicit exclusions.

An exact-duplicate cleanup removed 130 non-canonical evidence copies on 2026-10-26; all were untracked files with SHA-256-identical
canonical copies retained. The deletion list and post-cleanup counts are recorded in
[`docs/DUPLICATE_CLEANUP_20261026.md`](docs/DUPLICATE_CLEANUP_20261026.md).

## Repository layout

- `orbit_mcp_server.py`: MCP stdio entry point.
- `orbit_ros/`: robot, safety, memory, MCP, Webots, and simulator implementation.
- `tools/`: startup, health, world generation, benchmark, and validation utilities.
- `tests/`: unit and integration tests; live tests are explicitly marked.
- `webots/`: the required house world, controller assets, motions, and PROTO files.
- `configs/`: example client and experiment configuration.
- `docs/`: architecture, protocol, memory, maintenance, and evidence documentation.

## Troubleshooting

**Port 1235 is not listening.** Run `.\start_webots.ps1`, wait for the house to finish loading, then run `.\run.ps1 -Mode health`.

**The robot says it is already controlled.** Stop the other MCP server, `run.ps1 -Mode hold`, or benchmark. Only one external controller
can own `orbit_nao`.

**Webots is not found.** Set `WEBOTS_HOME` to the Webots folder containing `lib\controller`, in `.env` or the process environment.

**The MCP client receives invalid JSON.** Use the PowerShell wrapper or the `.venv` Python entry point. Startup diagnostics belong on
stderr; stdout must contain only MCP JSON-RPC.

**The server reports `ERROR` after a Webots restart.** Check the Webots console and port 1235, then start a new MCP session if the
bounded reconnect timeout has expired. Inspect `connection_status` before issuing motion commands.

## Security and limitations

- Do not commit `.env`, API keys, `.venv`, audit logs, or generated live results.
- Every robot action passes the registry and safety guard; `stop_robot` remains latched until `load_scene`.
- Webots is a simulator, not a physical safety certification.
- LLM providers and live API behavior are not tested without the user's credentials.
- A `v1.0.0` release requires a fresh complete non-live test run and the live verification evidence listed in
  [docs/RELEASE_EVIDENCE_20261026.md](docs/RELEASE_EVIDENCE_20261026.md).

## Contributing

Keep changes focused, add regression tests for lifecycle or safety behavior, run the focused suite and applicable full suite, and
update the relevant documentation with actual commands and outcomes. Do not commit generated results or secrets.
