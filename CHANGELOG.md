# Changelog

## 0.16.5
- Persistent NAO–Webots connection (keep-alive fix): `--keep-alive` now reliably holds the single `orbit_nao` extern controller after
  the AI client disconnects or a self-test finishes, instead of the process exiting and dropping the controller. The keep-alive loop
  builds the Webots backend explicitly (it no longer swallows a connection error via `hasattr`), steps the world continuously, exits
  cleanly on Ctrl+C or when Webots ends (`sup.step()` == -1), and pauses 1 s then retries on a transient error rather than busy-looping
  or crashing. Keep-alive only steps the world; it issues no motion and never bypasses the guard, the stop-latch or the call budget.
- `--keep-alive` is now part of the correct NAO+Webots client config: `--print-config` emits it, and `.vscode/mcp.json` and
  `configs/claude_desktop_config.example.json` include it, so VS Code / Claude keep the controller connected across client restarts.
- `run.ps1`: added `-Mode hold`, which opens Webots (if needed) and runs the `--keep-alive` server in that window to hold the controller
  until Ctrl+C, independent of any AI client. `-Mode server` now explains that the AI-client server uses `--keep-alive` and points to
  `-Mode hold` for a standalone persistent controller.
- Verified by code checks: `python -m pytest -q -m "not live"` -> **624 passed, 1 skipped** (2026-10-26, .venv Python 3.14).
  `--print-config` confirmed to emit `--keep-alive`. The live Webots behaviour of keep-alive is covered by the `@pytest.mark.live`
  tests / NAO benchmark and must be confirmed on a machine with Webots open (not run in this code-check environment).
- Transport change: the MCP server now connects to Webots **directly** as the single `orbit_nao` extern controller on `tcp://127.0.0.1:1235`.
  The old TCP relay (`orbit_bridge.py`, port 1240) is **retired**; no project component listens on or connects to port 1240 anymore.
  The MCP client keeps the one server process alive over stdio for the whole session (standard MCP lifetime), so the controller stays
  connected; use `--keep-alive` to keep Webots connected after a self-test ends.
- `--bridge-only` is now a deprecated no-op alias (still accepted so existing MCP configs keep working) that connects directly on 1235.
  `--direct` is likewise a deprecated no-op (direct is the only behavior). Generated client configs (`--print-config`), `.vscode/mcp.json`,
  `configs/*.example.json`, `run.ps1` and the VS Code tasks no longer reference the bridge or port 1240.
- `run.ps1 -Mode server` now just opens the Webots house; it no longer starts a relay process.
- Fixed two pre-existing test failures unrelated to the transport: restored the deleted `.gitignore` (which protects `.env`), and made
  the chat tool-count assertion derive the expected count from the registry instead of a hard-coded number.

## 0.16.4
- Added a visible NAO floor attachment that switches between vacuum and mop heads; cleaning commands accept a mode, and the selected mode is reported in status.
- Added a TV remote to the living-room coffee table.
- Corrected cleaning documentation to describe the floor attachment rather than the retired hand-wipe action.
- SQLite memory for the NAO (`recall_memory`), advisory only; sensors and the safety guard always win.
- TCP bridge (`orbit_bridge.py`, port 1240) so Webots stays connected between runs; `run.ps1` launcher.
- The Webots MCP config now requires `--bridge-only`; it uses the persistent bridge as the sole NAO controller and refuses to start a competing direct controller.
- `debug_view` (target, route, position, obstacles, task status) with a Webots text overlay.
- `plan_tasks` / `run_plan`: several rooms or objects in one job, shortest order, each step with a completion check.
- Added guarded `tour_house` (visit every room, optionally return), `clean_room` (navigate and report one-room brush coverage), and `greet` (supported hand-wave, no speech).
- Added the Webots house resident and `greet_resident` MCP command, which navigates to a safe nearby point and waves. The resident now sits on the sofa facing the television; `join_tv_watching` safely approaches the sofa and faces the screen.
- Removed the seated resident from the house and retired the resident-specific greeting command.
- Added `turn_to_object`, a local guarded command that faces a named house item without navigating to it.
- Added live benchmark scenarios for house touring, single-room cleaning and greeting; the NAO MCP self-test now exercises the greeting and safety veto.
- Live fixes: fewer false sonar vetoes, no oscillation on long walks, bridge client timeout raised to 1 h.
- The TurtleBot Webots world, controller (`orbit_c1`), `run_extern.py`, `webots_auto_repair.py`, `docs/WEBOTS_C1.md`, their VS Code entries and `-Benchmark/-Extern` flags were removed: the only Webots world is the NAO house. World2D/FakeSim research code stays.
- Removed the stale `FILE_MANIFEST.txt`.

## 0.16.3
- NAO walking: replaced Webots' `Motion` player (it reported "over" after 0.26 s) with a `.motion` file player that drives the joints directly.
- Cleaning with the hand (`WipeForehead.motion`, two passes) in `clean` and `clean_house`; automatic stand-up after a fall (up to 3 per walk).
- `go_to_object` stops next to the object itself (not the merged furniture cluster), so the sofa and desk are reached.
- The per-walk motion limit now grows with the route length (a real NAO drifts and needs about 10 motions per metre).
- `start_webots.ps1` opens `orbit_house.wbt` by default, refuses to start a second Webots, and keeps the TurtleBot worlds behind `-Benchmark`.
- New `tools/nao_benchmark.py` (22 tasks; SR, SPL, goal-conditioned success, collisions, falls) and `tools/make_oracle_baseline.py`.
- Missing Webots R2025a assets (Bed, RubberDuck, skybox textures) are completed from the official sources.
- Removed stale backups, diagnostics and old result folders; `orbit_c1.wbt` rebuilt after Webots saved `nan` values into it.
- Tests accept worlds re-saved by Webots (layout and camera pose differ) and expect 26 NAO tools.
