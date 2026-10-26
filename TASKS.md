# ORBIT ROS NAO — Inspect / Fix / Implement / Test Log

Real runtime tests only. No mocks, no placeholder results, no unverified success claims.

Environment (verified 2026-10-24):
- Webots R2025a: `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe` (found via `tools/find_webots.ps1`; `WEBOTS_HOME=C:\Program Files\Webots`).
- Python venv: `.venv\Scripts\python.exe`.
- Git repo (root one level above this folder). Shell is PowerShell.

---

## Phase 14 — Core plan-constrained MetaTool: live acceptance (2026-10-25) — COMPLETE (all 10 scenarios PASS)

Full report: `docs/PHASE14_METATOOL_REPORT.md`; advanced follow-up: `docs/METATOOL_ADVANCED_PLAN.md`. Evidence:
`results_live/phase14_metatool_20261025/` (+ referenced `results_live/m10_20261025/`). No core module rebuilt (audit confirmed the
MetaTool was already implemented + partially verified in prior turns); Phase 14 ran the previously-uncovered live acceptance scenarios.

- **All 10 section-5 live scenarios PASS** (real NAO + real Groq, judged from true pose, 0 fallbacks/resets, no safety bypass):
  1 invalid-tool reject, 2 dependent-order plan, 3 navigation true-pose, 4 blocked→honest abort, 5 loop-guard block, 6 hazardous refusal
  (0 motion), 7 stop-latch not bypassable, 8 no step repeated after replan, 9 memory context without safety bypass, 10 chat `--mode plan`.
- Regression: code checks `-m "not live"` **560 passed, 2 skipped**; live MCP self-test **PASSED (261.5 s, 48 tools)**.
- Docs: `PHASE14_METATOOL_REPORT.md` (new), `METATOOL_ADVANCED_PLAN.md` (new, dependency-ordered), `METATOOL_SYSTEM.md` + spec `tasks.md`
  updated (M.10 → done). Backups: `_work_backup/20261025_025110/`. Protected files + historical results untouched.

---

## Phase 13 — Animated human actor + dedicated Human Controller (2026-10-25) — CORE DONE LIVE (interaction/speech = follow-up)

Full report: `docs/PHASE13_HUMAN_REPORT.md`; controller design: `docs/HUMAN_CONTROLLER.md`. Live evidence:
`results_live/phase13_20261025/human_walk/`. Protected files (`orbit_house.wbt`, `house.py`, `webots/protos/`) **untouched**.

- **Asset check (honest):** installed R2025a has `Pedestrian.proto` (articulated, jointed human) and `skin_animated_humans/CharacterSkin.proto`
  + BVH; **no Robert/Sandra/Sophia standalone skins** → used `Pedestrian.proto` (not the unavailable named skins).
- **Non-destructive world generation:** `tools/make_human_world.py` takes the approved generator's house text (from `house.py`, unchanged)
  and inserts a Pedestrian, writing the **candidate** `webots/worlds/orbit_house_human.wbt` — `orbit_house.wbt` never overwritten
  (verified: 148/148 house lines preserved, NAO preserved).
- **Dedicated `orbit_ros/human_controller.py`:** independent module, `human_*` namespace, drives `DEF ORBIT_HUMAN` via the EXISTING NAO
  Supervisor (no 2nd Supervisor/process, port 1235 only). idle/set_pose/reset/pause/resume/turn_to/walk_to/follow_route/update/report;
  configurable speed/tolerance/timeout; statuses ACCEPTED/RUNNING/COMPLETED/TIMED_OUT/FAILED. Animation-driven (gait angle fields),
  explicitly not physics locomotion; never calls NAO motors.
- **Live check PASS:** human walked a scripted route **1.443 m** to within 0.3 m of the goal (COMPLETED), while the **NAO drifted 0.0 m and
  stayed upright** (independent controllers; NAO uncorrupted); 0 fallbacks/resets; target-mode screenshots captured.
- Code checks: `tests/test_human_controller.py` (10). Full `-m "not live"`: **560 passed, 2 skipped**. Backups: `_work_backup/20261025_023417/`.
- **NOT IMPLEMENTED / NOT RUN (honest):** MetaTool `human_*` tool contracts, person-following, look-at/social gestures, Speaker lifecycle +
  joint-interpolation audits (sec 6–7), perception/visualization/voice/dashboard, and native human–NAO collision stress tests. Recorded in
  the acceptance matrix, not claimed.

---

## Phase 12 — MetaTool + adaptive Webots viewpoint integration (2026-10-25) — PARTIAL (viewpoint DONE live; advanced MetaTool NOT IMPLEMENTED)

Full report: `docs/PHASE12_INTEGRATION_REPORT.md` (incl. the start-of-phase status matrix + acceptance matrix). New evidence:
`results_live/p12vp_20261025b/` (viewpoint), referencing `results_live/m10_20261025/` (MetaTool, prior turn). Protected files untouched.

- **Adaptive viewpoint (`orbit_ros/viewpoint.py`) — VERIFIED LIVE.** overview/follow/target/demo/reset; built-in Webots `follow`/
  `followType`="Tracking Shot"/`followSmoothness` (confirmed `builtin=True` live); `look_at_rotation` copied (not imported) so the NAO
  runtime stays free of the TurtleBot/World2D legacy `webots_adapter`. Display only — never moves the robot / resets physics / calls the LLM.
- **Evidence pipeline integration — VERIFIED LIVE.** `tools/live/webots_session.py`: `camera(mode)`, per-screenshot `screenshots.json`
  metadata recording the REAL active mode, start/end forced to overview. Live check `p12vp_20261025b`: reached kitchen (true pose),
  5 distinct mode-tagged screenshots, 0 fallbacks/resets.
- **Bug found + fixed live:** viewpoint change not rendered while the controller is idle between calls (byte-identical screenshots) →
  `Viewpoint.render()` advances the EXISTING controller a few frames (no second Supervisor, no robot motion). Before/after evidence saved.
- Code checks: `tests/test_viewpoint.py` (4). Full `-m "not live"`: **550 passed, 2 skipped**. Docs: `VIEWPOINT_SYSTEM.md`,
  `METATOOL_SYSTEM.md`, `PHASE12_INTEGRATION_REPORT.md`; `COMMAND_MATRIX.md` updated to 48 tools; `ARCHITECTURE.md` to follow.
- **NOT IMPLEMENTED (honest):** the extended MCP interface (`plan_status`, `cancel_plan`, `resume_plan`, `interrupt`, `approve`, `deny`,
  `answer_clarification`, `list_skills`, `explain_last_run`, `get_metrics`) and the 13 advanced MetaTool modules (dryrun, candidates,
  plan_cache, clarify, constraints, scheduler, interrupts, skills, routing, tool_stats, explain, approval, certificate). Each needs its own
  module + live acceptance; recorded in the acceptance matrix, not claimed complete.
- Backups: `_work_backup/20261025_025424/`.

---

## Phase 10 — Persistent memory module + LLM learning (2026-10-24) — COMPLETE

Full write-up: `docs/PHASE10_MEMORY_REPORT.md`; module design: `docs/MEMORY_MODULE.md`. New data only (own dir + ids):
`results_nao_benchmark/phase10_memory_20261024T224840Z/`. **Phase 8/9 results were NOT recomputed/overwritten** — hashed and
verified unchanged: `HISTORICAL INTEGRITY: OK (7 files)` via `tools/phase10_registry.py`.

- **Extended** (not replaced) `orbit_ros/nao_memory.py` to v2: episodic `episodes` table (per-task records) + semantic `strategies` table
  (distilled reusable strategies with confidence/verified) + `meta` schema version + in-place v1→v2 migration + WAL-safe writes + dedup +
  configurable size/retention + explicit reset + deterministic task-similarity retrieval (`normalize_goal`, `recall_episodes`,
  `recall_strategies`, `task_brief`). Kept it a leaf module (passes PEL layering test).
- **Session integration** (`orbit_ros/humanoid.py`): `task_memory(instruction)` (retrieval) and `record_episode(...)` (write-back +
  `learn_strategy`). **MCP tool `task_memory`** added (now 45 NAO tools). Config via `orbit_mcp_server.py`: `--no-memory`,
  `--no-memory-retrieval`, `--no-memory-write`, `--memory-db`, `--memory-max-records`; printed at startup (`[orbit-mcp] memory: ...`).
- **Live LLM integration verified**: enabled trials retrieve memory and prepend the brief to the REAL Groq request; disabled trials do not.
- **Live experiments (real Webots 1235 + real Groq, one process/trial):**
  - A (off vs on, goto_kitchen/goto_bedroom): 4/4 PASS; retrieval wired in (goto_bedroom on retrieved 1); success neutral, small token cost (n=1/cell → preliminary).
  - B (learn from a genuine failure): seeded a real `go_to_room bedroom → "no safe path"` failure; memory-enabled retest **retrieved the
    failure** and, with the zone removed, **succeeded** (true pose = bedroom) — observations overrode stale memory.
  - D (restart persistence): a fresh process retrieved 4 persisted memories (strategy provenance → Exp A) and reached the kitchen.
  - E (stale memory): a stale strategy's unsafe target was **rejected by the guard**; robot never entered the zone (memory can't bypass safety).
  - F (deterministic metrics): retrieval precision@1 = **1.0**, retrieval 0.37 ms / write 0.16 ms mean, dedup 5→0, corrupted db caught, disabled honored.
- **Bugs found live & fixed** (Phase 10 code only; protected files untouched): memory→leaf layering (`normalize_goal` vocab as args);
  Cloudflare 1010/403 from a missing `User-Agent` in the exp-B runner; a conflicting-episode db confound → rerun B in a fresh db.
  Affected experiment reran live after each fix.
- **Software tests (separate from live):** `pytest` → **519 passed, 2 skipped** (1 slow test deselected; passes in isolation), incl. 7 new
  memory tests. Pre-existing safety/memory invariants still hold (guard never reads memory; broken memory doesn't stop the robot).
- New files: `orbit_ros/nao_memory.py` (extended), `tools/phase10_registry.py`, `tools/phase10_experiment.py`, `tools/phase10_exp_bdef.py`,
  `tools/phase10_exp_f.py`, `docs/PHASE10_MEMORY_REPORT.md`, `docs/MEMORY_MODULE.md`. Backups: `_backups_20261025_003321\`.
- Remaining/blockers: Gemini unreachable (Groq used); n=1/cell (neutrality shown, not improvement — harder multi-step suite is future work);
  the two `.bak` files still flagged for approval.

---

## Phase 9 — Live LLM planning & closed-loop evaluation (2026-10-24) — COMPLETE

Full write-up: `docs/PHASE9_RESEARCH_REPORT.md`. Authoritative data: `results_nao_benchmark/phase9_consolidated_20261024T190219Z/`
(+ 101 raw per-trial traces under `results_nao_benchmark/phase9_live_*/`). All live; Groq `openai/gpt-oss-120b` genuinely called;
Webots on tcp://127.0.0.1:1235 (never 1240); one Supervisor process per trial.

- Audited LLM/planner/perception/memory/guard: labelled honestly (A* and replanning are deterministic; perception is map-based, not vision;
  only `orbit_chat`/the Phase 9 runner are LLM-driven). No fixed-sequence baseline existed → implemented strategy A.
- Pre-flight: real Groq call verified (`tools/phase9_llm_check.py`); live controller verified (`tools/phase9_conn_check.py`). Gemini was
  configured but **timed out** from this machine → all LLM trials used Groq (documented blocker).
- Ran **36 live trials** across **12 deterministic scenarios × 3 strategies** (A open-loop, B planner, C LLM), judged from TRUE pose.
  **Results: A 6/10 (60%, 2 SKIP), B 12/12 (100%), C 12/12 (100%); 0 safety violations / 36 trials; C used 31 real LLM calls, 2.11 s mean
  latency, 157,916 provider-reported tokens.** Closed-loop (B/C) beat open-loop on navigation by +100 pts (0/4 → 4/4), matched conditions.
- Bugs found live & fixed (runner only; protected files untouched & byte-identical to backups): (1) no 429 retry → added backoff;
  (2) runner bypassed MCP TypeError handling → wrapped like production; (3) reasoning-model token cap → raised + nudge; (4) success-judge
  bugs → fixed + strategy-aware criteria for invalid/unsafe (documented). Regression tests added:
  `tests/test_humanoid.py::TestMalformedToolArgs` (2 tests, pass).
- New deliverables: `tools/phase9_experiment.py` (live runner), `tools/phase9_consolidate.py`, `tools/phase9_llm_check.py`,
  `tools/phase9_conn_check.py`, `docs/PHASE9_RESEARCH_REPORT.md`.
- Software tests (separate from live evidence): **512 passed, 2 skipped** (one heavy pre-existing fake-sim test,
  `test_go_to_every_room_and_object_never_bypasses_guard`, deselected for time — passes alone in ~270 s; `humanoid.py` unchanged this phase).
- Backups: `_backups_20261024_163632\`, `_backups_20261024_172922\`.
- Remaining/blockers: Gemini unreachable from this machine (Groq used). n=1 per cell (qualitative ordering clear; no CIs claimed).
  Webots crashed once mid-session (diagnosed: process gone/1235 free → restarted). The two `.bak` files still flagged for approval.

---

## v0.16.5 — Full-system engineering pass (2026-10-24)

### 0. Architecture audit (done)
Mapped the real execution path end to end:
`LLM/agent -> MCP tools/call (stdio JSON-RPC) -> HumanoidMcpServer.call_tool -> (robot tool) OrbitMcpSession/HumanoidSession.robot_call
-> call budget -> validate_humanoid_call (typed args) -> preconditions (not stopped, upright) -> HumanoidGuard.check (walls, furniture,
no-go zones, from the REPORTED pose) -> WebotsNao/.motion -> Webots -> pose read back -> JSON result -> agent`.
See `docs/ARCHITECTURE.md` (new) for the full diagram and `docs/COMMAND_MATRIX.md` (new) for the per-tool compatibility matrix.

### 1. MANDATORY transport change: port 1240 removed, 1235 only (done, live-verified)
**Before:** `orbit_mcp_server (RemoteSession) --TCP:1240--> orbit_bridge (WebotsNao) --TCP:1235--> Webots`. The bridge existed only to
share one Webots extern controller across short-lived client processes.
**After (v0.16.5):** `MCP client --stdio--> orbit_mcp_server (HumanoidSession/WebotsNao) --TCP:1235--> Webots`. The MCP server is itself
the single extern controller and stays alive for the whole client session (standard MCP stdio lifetime). **No port 1240 anywhere.**

Changes:
- `orbit_mcp_server.make_session`: NAO+Webots now builds a direct in-process `HumanoidSession(sim="webots")` on `tcp://127.0.0.1:1235`.
- `--bridge-only` and `--direct` are now deprecated no-op aliases (kept so old MCP configs keep working); both connect directly on 1235.
- `orbit_bridge.py`: **retired** — `main()` refuses to start a server (cannot bind 1240); `serve_client`/`execute` kept for the protocol unit test.
- `orbit_ros/remote.py`: `PORT` changed `1240 -> 0` (disabled). Kept as a library for the serialization unit test only.
- `run.ps1`: `-Mode server` now just opens Webots; it no longer starts any relay. The `Test-Bridge`/1240 logic is gone.
- Configs updated: `.vscode/mcp.json`, `.vscode/launch.json`, `.vscode/tasks.json` (task renamed "Start Webots (NAO house)"),
  `configs/claude_desktop_config.example.json`, `configs/gemini_settings.example.json`, `orbit_chat.py` `DEFAULT_SERVER_ARGS`.
- Docs updated: `README.md` §4, `docs/MCP_SERVER.md`, `CHANGELOG.md`, `tools/nao_benchmark.py` header.
- New regression test `tests/test_mcp_server.py::TestClientConfigs::test_no_config_or_script_references_port_1240` enforces that no
  config/script references 1240; `test_make_session_connects_to_webots_directly_without_bridge_or_1240` locks the direct transport.

Trade-off (documented): only one extern controller can own the NAO at a time (Webots limitation — already true). Run either the live MCP
server or the standalone benchmark, not both at once. `--keep-alive` keeps Webots connected after a self-test ends.

### 2. Pre-existing bugs fixed (unrelated to transport)
- **`.gitignore` was deleted from disk** (broke `tests/test_env_config.py::test_gitignore_protects_env`, which guards `.env`). Restored from `HEAD`.
- **Hard-coded tool-count assertion** in `tests/test_orbit_chat.py` (`== 42`) was stale. Made it derive the expected count from the
  registry (`len(HUMANOID_TOOLS) + len(H_SESSION_TOOLS)`), per the "update assertions dynamically, don't weaken tests" rule.

### 3. New command added (justified): `connection_status`
Read-only session diagnostic that reports the live transport/controller health: backend, whether the Webots extern controller is actually
connected on `tcp://127.0.0.1:1235` (a real `sup.getSelf()`, not just a listening socket), ready/stopped state, pose when live, and the
remaining call budget. Never moves the robot, never raises. Directly serves the task's connection-diagnostics theme.
Wired into `HumanoidMcpServer` (`H_SESSION_TOOLS` + `call_tool`); 4 regression tests in `tests/test_humanoid.py::TestConnectionStatus`.
Authoritative tool count is now **44** (33 humanoid + 11 session), confirmed through the live MCP `tools/list`.

### 4. Cleanup
See `CLEANUP_REPORT.md`. 22 root scratch files + 6 project `__pycache__` dirs + 3 live-verify captures were moved to a timestamped
quarantine folder **outside** the project. `.venv`, `.git`, `.vscode`, `.env`, source, tests, docs, configs and the house world were preserved.
The two `.bak` files remain flagged for approval (backups — not moved).

---

## v0.16.5 — Phase 8: reliability, reconnection, command regression, benchmarking (2026-10-24)

See `docs/RELIABILITY_REPORT.md` for the full scenario-by-scenario write-up and metrics.

### Confirmed defect fixed: connection loss raised instead of returning an error
- **Symptom (reproduced live):** on an unexpected Webots disconnect, `robot_call("walk_forward")` / `robot_call("get_pose")` raised
  `RuntimeError` out of `robot_call` instead of returning `{status:error}`.
- **Root cause (captured traceback):** `robot_call` read the pose (`self._pose()` → `self.nao.pose()`) for validation **before** its
  `try/except`, and the error-reporting path `_finish → _trace → debug_state → _pose` read the dead backend **again**.
- **Fix (`orbit_ros/humanoid.py` only):** defensive pose read in `robot_call` with safety-first ordering (stop-latch + argument
  validation still fire on a dead backend), returning a clean/logged/actionable `{status:error}`; and `_trace` made resilient so the
  diagnostic path never re-raises. No guard/schema/latch behavior changed; the connected path is unchanged.
- **Regression tests:** `tests/test_humanoid.py::TestConnectionLoss` (5 tests), incl. "stop-latch still rejects during a loss".

### Controller lifecycle (confirmed by live observation)
- The extern controller is meant to **exit per MCP session**; Webots keeps the robot and re-accepts the next controller (port 1235
  returns to LISTENING on exit). Reconnection = a **new process** (Webots allows only one Supervisor per process). Each MCP session is a
  fresh process, so this is the production model.

### Live reliability benchmark (3 trials, one process per trial)
`results_nao_benchmark/reliability_20261024T145555Z/reliability.json`:
- Command success **18/18 (100%)**; task completion **3/3 (100%)**.
- Turn error mean **0.9°** / max **1.1°** (tol 15°, n=9); walk displacement mean **0.142 m** (n=6).
- Reconnections **3/3 (100%)**, mean latency **0.55 s**; **0 unexpected disconnects**; **0 safety violations** (3 correct vetoes).

### Command regression
- Registry (44) **== live MCP `tools/list` (44)**: 0 missing, 0 extra.
- Fake-sim argument/veto/stop-latch checks: **9/9** (labelled fake, not presented as live).
- Full unit suite after the fix: **511 passed, 2 skipped, 0 failed** (+5 reliability tests over the 506 baseline).

### Files changed (Phase 8)
- `orbit_ros/humanoid.py` (defect fix), `tests/test_humanoid.py` (TestConnectionLoss), `docs/RELIABILITY_REPORT.md` (new),
  plus doc updates. Backups: `_backups_20261024_163632\`.

---

## Verification evidence (all executed on this PC, 2026-10-24)

### Unit suite — GREEN
- Command: `.venv\Scripts\python.exe -m pytest -p no:cacheprovider`
- Result: **506 passed, 2 skipped, 0 failed** (413 s). Baseline before this pass was 500/2 after fixing the two pre-existing failures;
  +2 transport tests +4 connection_status tests = 506.
- The 2 skips are environment/data dependent and unrelated: `test_integrity.py:188` (optional `results_v08_quick` artifact absent) and
  `test_mcp_server.py:210` (skips a direct-connect test because a Webots install is present).
- Imports: `import orbit_ros.humanoid, orbit_ros.mcp_server, orbit_mcp_server, orbit_bridge, orbit_ros.remote, orbit_chat` -> `IMPORTS_OK`.

### House-world integrity — GREEN
- `tests/test_humanoid.py::TestHouse::test_world_file_matches_house` and `test_rooms_and_items`: **2 passed**. The shipped
  `webots/worlds/orbit_house.wbt` matches `orbit_ros/house.py` and contains no `nan`. The world file was **not modified** in this pass.
- `tests/test_integrity.py`: 17 passed / 1 skipped. `tests/test_webots_discovery.py` + `test_webots_view.py` + `test_integrity.py`: 26 passed / 1 skipped.

### Fake NAO self-test — PASS
- `.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim fake` -> `SELF-TEST PASSED (1.1 s ...)`, exit 0, 44 tools.

### LIVE Webots self-test (direct 1235, no bridge) — PASS
- Webots launched with the house world on port 1235 (`webots-bin` PID 19028 LISTENING on 1235).
- `.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim webots --keep-alive`
- Server log: `[orbit-mcp] connecting to Webots directly on 127.0.0.1:1235 (extern controller 'orbit_nao')`.
- Result: **`SELF-TEST PASSED (246.6 s, robot nao, simulator webots)`**, 44 tools listed (incl. `connection_status`). Full path exercised live:
  load_scene, house_info, describe_view (5 items), get_pose, greet, join_tv_watching (moved beside sofa, faced TV), argument rejection
  (`walk_forward steps=9 -> rejected 1..6`), guard veto (`set_no_go_zone` then `walk_forward -> vetoed`), out-of-house rejection
  (`go_to 20,3 -> rejected`), emergency stop + stop-latch (`wave -> rejected: robot is stopped`).
- Capture: `_live_selftest_1235.txt` (moved to quarantine as evidence).

### LIVE Webots movement verification (real before/after pose) — PASS
- Direct `HumanoidSession(sim="webots")`; true pose read from the Webots supervisor.
- `connection_status` live: `{"transport":"in-process (direct)","uses_port_1240":false,"controller_port":1235,"connected":true,"backend":"webots"}`.
- start pose x=4.800 y=3.100 yaw=180.0
- `turn left 60` -> ok, measured_dyaw **+58.9°** (target +60, err 1.1°).
- `walk_forward 2` -> ok, **moved 0.189 m** from (4.805,3.100) to (4.707,2.938).
- `motion_accuracy`: turning pass (1.1° err), walking pass (0.095 m mean, 0.1° drift), **0 kinematic fallbacks** -> real gait, `pass: true`.
- Verdict: `LIVE_VERIFY_VERDICT: PASS (turn_ok=True, walk_ok=True)`. Capture `_live_verify_1235.txt` (in quarantine).

### Port-usage proof
- During and after the live runs: `Get-NetTCPConnection -LocalPort 1240` returned **0** connections; port **1235** is owned by `webots-bin`.
- Static sweep: no `.py`/`.ps1`/`.json`/`.bat`/`.toml` under the project (excluding `_backups_*`/`.venv`/`.git`) uses 1240 as a live
  value — every remaining literal is a retirement comment, a test asserting its absence, or the disabled `PORT = 0` constant.

---

## Test commands (reproduce on this PC)
- Unit suite: `.\.venv\Scripts\python.exe -m pytest -p no:cacheprovider`
- Fake NAO self-test: `.\.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim fake`
- Open the house: `.\start_webots.ps1`  (or `.\run.ps1 -Mode server`)
- Live MCP self-test (direct 1235): `.\.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim webots --keep-alive`
- Live benchmark: `.\.venv\Scripts\python.exe tools\nao_benchmark.py --sim webots`  (run alone; only one controller at a time)

## Remaining / blockers
- No open blocker. The 1235-only transport is implemented and live-verified.- `.bak` files (`orbit_ros/humanoid.py.bak`, `orbit_ros/mcp_server.py.bak`) remain in place, flagged for your approval (backups).
- `orbit_ros/house.py` and `webots/worlds/orbit_house.wbt` show as modified **relative to git HEAD**, but that is PRE-EXISTING work from
  before this session (both appeared as `M`/`??` in the first `git status` at session start; the `.wbt` on-disk timestamp is 2026-10-23 23:25,
  ~16 h before my first edit at 2026-10-24 15:11, and the file is not in my `_backups_` set because I never opened it). I did NOT modify the
  house world or its assets (`webots/protos/CleaningBrush.proto`, timestamp 2026-10-23). The house-world integrity test
  `test_world_file_matches_house` passes, so the shipped world remains consistent with `orbit_ros/house.py`.
- Webots was left running (the house world I opened for the live test); close it when done, or it keeps waiting for a controller.
- Backups of every file I edited are under `_backups_20261024_151132\`.

---

## Phase 16 — Visual Clarity (adaptive camera + realistic human)

Full report: `docs/PHASE16_VISUAL_CLARITY_REPORT.md`. Evidence: `results_live/phase16_visual_clarity_20261025T034432Z/`.

- Extended `orbit_ros/viewpoint.py`: 7 modes (`overview` angled bird's-eye / `follow` / `closeup` / `target` / `human` / `demo` /
  `reset` saved straight-down) + `zoom` (dolly, clamped, not FOV) + `smoothing_alpha`. Display-only contract preserved.
- Added `HumanoidSession.camera(mode, zoom?, target?)` and the MCP **`camera`** tool (read-only) → tool count **48 → 49**. Added `/view`
  chat commands in `orbit_chat.py` (handled locally, never routed to the LLM).
- New realistic human candidate `webots/worlds/orbit_house_human_realistic.wbt` via additive `tools/make_realistic_human_world.py`
  (Pedestrian with sculpted-mesh face + tuned PBR skin/clothing; `DEF ORBIT_HUMAN` unchanged, protected files untouched).
- Code checks: `tests/test_camera.py` (22) + `tests/test_realistic_human_world.py` (7) + updated `tests/test_viewpoint.py`; full
  `pytest -m "not live"` = **590 passed, 2 skipped, 1 deselected**.
- Live: `tools/live/phase16_camera_trial.py` — all 7 modes `ok`, zoom in/out ok, **all 14 screenshots distinct**, **robot moved 0.0006 m** over
  all camera changes, 0 fallbacks / 0 resets → verdict PASS. Live MCP self-test PASSED (167 s, 49 tools incl. `camera`).
  Clean evidence: `results_live/phase16_visual_clarity_20261025T035901Z/`. (Also fixed a harness bug where `end.png` was not rendered before
  capture.)
- **PARTIAL:** the `_realistic` candidate world itself was not rendered live (the running Webots had the Phase-13 human world loaded). Open
  `orbit_house_human_realistic.wbt` in the Webots GUI and re-run the trial to finish this; it is a user GUI step (one controller per 1235).
- Integrity after phase: protected files unchanged (timestamps verified); port 1240 = 0 connections; all historical results intact.
- Backups: `_work_backup/phase16_20261025_032659/`.



---

## Phase 17 — Evidence-driven MetaTool upgrades (2026-10-25)

Status labels: VERIFIED LIVE / CODE-CHECK ONLY / BLOCKED. Full report:
`docs/PHASE17_RESEARCH_UPGRADES_REPORT.md`. Evidence: `results_live/phase17_20261025T140447Z/`.
Backups: `_work_backup/phase17_20261025_150447/`. No protected file (`orbit_house.wbt`, `house.py`,
`webots/protos/`, live motion path) was changed.

- [x] Audit prerequisites + runtime config (port 1235, Groq/Gemini keys, Webots availability). Note: referenced
      `KIRO_PROMPT_ORBIT.md` is absent from the repo; audited against actual modules.
- [x] Feature 2 — refusal/action gate (`REFUSAL_MISMATCH`) — CODE-CHECK ONLY
- [x] Feature 1 — two-pass/completeness (`INCOMPLETE`, planning_mode) — CODE-CHECK ONLY
- [x] Feature 8 — tool retrieval (`tool_retrieval.py`) — CODE-CHECK ONLY
- [x] Feature 3 — deterministic safety gate + contract (`safety_gate.py`) — CODE-CHECK ONLY
- [x] Feature 11 — tool-manifest integrity + injection hardening (`tool_manifest.py`, prompts) — CODE-CHECK ONLY (fail-closed verified)
- [x] Feature 7 — root-cause repair (`root_cause.py`) — CODE-CHECK ONLY
- [x] Feature 5 — consistency voting (`voting.py`) — CODE-CHECK ONLY
- [x] Feature 9 — optional judge (`judge.py`, off by default) — CODE-CHECK ONLY
- [x] Feature 10 — consolidated memory view (`memory_view.py`, read-only) — CODE-CHECK ONLY
- [x] Feature 6 — in-step monitor evaluator (`monitors.py`) — CODE-CHECK ONLY; live motion-loop wiring BLOCKED
- [x] Feature 4 — calibrated clarification framework (`clarify.py`) — CODE-CHECK ONLY; calibration data BLOCKED (needs ≥50 real user-labelled instructions)
- [x] Regression + safety tests (`tests/test_phase17_metatool.py` = 44 passed; metatool suite 69 passed)
- [x] Full non-live suite: only pre-existing `test_world_file_matches_house` fails (world Viewpoint mismatch, §6 of the audit; not a Phase 17 regression)
- [ ] LIVE acceptance of every feature — **BLOCKED** (Webots not running; one extern controller; hours-long). Commands in `results_live/phase17_20261025T140447Z/LIVE_STATUS.md`.
- [ ] Measured ablation/evaluation tables (Wilson intervals, McNemar, wrong-valid-schema, etc.) — **BLOCKED** (need live provider + NAO).
