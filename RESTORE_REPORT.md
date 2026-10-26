# NAO Environment Restore Report

**Date:** 2026-10-25
**Action:** Stopped the experimental human-model work and restored the last known-good NAO Webots environment.

---

## 1. Exact world restored / confirmed active

**Active world: `webots/worlds/orbit_house.wbt`** — the original furnished NAO house, **no human**.

- SHA-256 (first 16): `2d6dfcb66232780f`
- Last modified: **Oct 23 23:25** — *before* any human-experiment work (all human work is Oct 24+), so the file was **never touched by the experiment**. No file-content restore was required; the working world was already intact on disk.
- Contents verified: `Nao {}` / `orbit_nao` present (1), human markers absent (`ORBIT_HUMAN`, `CharacterSkin`, `Pedestrian`, `orbit_human_skin` all 0).
- Integrity check: `python tools/make_house_world.py --check` → **"orbit_house.wbt is up to date"** (matches the `orbit_ros/house.py` source of truth).

The production entry points already defaulted to this world and needed no change:
- `start_webots.ps1` (default world = `webots\worlds\orbit_house.wbt`)
- `run.ps1` → calls `start_webots.ps1`
- `.vscode/mcp.json` → `orbit_mcp_server.py --robot nao --sim webots` (no world path)

---

## 2. Experimental human model removed from the active world

The failed experiment used two separate candidate worlds (never the production world):
- `orbit_house_human_realistic.wbt` — `DEF ORBIT_HUMAN Robot { CharacterSkin "Robert" }`, controller `orbit_human_skin` (the failed sitting experiment).
- `orbit_house_human.wbt` — Phase-13 `Pedestrian` variant.

Both were moved out of the active tree (see §3). Their local controller `orbit_human_skin` was moved with them, so it can no longer be launched. The active `webots/worlds/` now contains **only** `orbit_house.wbt`, and `webots/controllers/` contains **only** the original furniture controllers (`television_switch_on`, `laptop_switch_on`).

---

## 3. Files archived (moved, not deleted)

All experiment-exclusive artifacts were **moved** (reversible) into:

**`_archive_human_experiment_20261025_124510/`** (preserves relative paths; `ARCHIVE_MANIFEST.txt` inside lists every item).

Moved (34 items):

| Category | Items |
|---|---|
| Experimental worlds | `webots/worlds/orbit_house_human_realistic.wbt` (+ `.wbproj`), `orbit_house_human.wbt`, `_verify_sit.wbt` (+ `.wbproj`), `orbit_human_cmd.txt` |
| Experiment controllers | `webots/controllers/orbit_human_skin/` (incl. `sit.bvh`), `webots/controllers/orbit_sit_verify/` |
| Experiment generators / refs | `tools/make_sit_bvh.py`, `tools/make_skin_human_world.py`, `tools/make_realistic_human_world.py`, `tools/make_human_world.py`, `tools/walk_ref.bvh`, `tools/stepup_ref.bvh`, `tools/jump_ref.bvh` |
| Loose debug/helper scripts | `orbit_human_skin.py` (root copy), `_live_verify2.ps1`, `_run_sit_verify.ps1`, `_bk.ps1`, `_bk_now.ps1`, `_bk_tmp.ps1`, `_inspect_procs.ps1`, `_proc_check.ps1`, `_probe_live2.ps1`, `_skin_probe.py`, `_live_human_tool.py`, `_tmp_probe_human.py`, `_tmp_layout.py`, `_tmp_probe/`, `_verify_out/`, `_live_nao_selftest.log`, `_live_webots_console.log`, `_verify_webots.log`, `_verify_webots.err.log` |

Nothing was deleted. Nothing was skipped.

---

## 4. Preserved (original / shared / history — intentionally NOT touched)

- `orbit_house.wbt` and its `.wbproj`.
- The NAO robot model, MCP integration, and all core modules: `orbit_mcp_server.py`, `orbit_ros/` (including `humanoid.py`, `mcp_server.py`, `house.py`, `nao_memory.py`), `orbit_chat.py`, `orbit_bridge.py`, `run.ps1`, `start_webots.ps1`, `.vscode/*`, `setup.ps1`.
- `orbit_ros/human_controller.py` — **kept** because it is imported by the active test suite (`tests/test_human_controller.py`, `tests/test_realistic_human_world.py`); it is a standalone module and is not loaded by the restored world.
- `tools/make_house_world.py` — **kept** (production generator for `orbit_house.wbt`).
- `tools/live/p13_human_live.py` — kept as history (not part of `unittest discover -s tests`).
- All results and history: `results_mcp/`, `results_live/`, `results_nao_benchmark/`, `results_spatial_oracle/`, `docs/*.md`, `TASKS.md`.
- All prior backups: `_work_backup/*`, `_backups_*/*`.

---

## 5. Reference / configuration fixes

No active executable or config reference pointed at the experimental human world, so no production references needed changing:
- `.vscode/mcp.json`, `run.ps1`, `start_webots.ps1`, `orbit_ros/*`, `orbit_mcp_server.py`, and the `tests/` suite reference only `orbit_house.wbt` (or no world path).
- Remaining `orbit_house_human*` strings exist **only** in: the archive folder itself, `_work_backup/*` backups, and historical documentation (`docs/PHASE13_HUMAN_REPORT.md`, `docs/HUMAN_CONTROLLER.md`, `docs/PHASE16_VISUAL_CLARITY_REPORT.md`, `TASKS.md`, `results_live/*`). These are records of past work, not live path configuration, and were preserved.

---

## 6. Tests performed

| Test | Command | Result |
|---|---|---|
| World integrity | `python tools/make_house_world.py --check` | **PASS** — "orbit_house.wbt is up to date" |
| Active-tree contents | directory listing of `webots/worlds`, `webots/controllers` | **PASS** — only `orbit_house.wbt` + original furniture controllers remain |
| Live Webots load | launched `webots.exe --mode=realtime --port=1235 orbit_house.wbt` | **PASS** — port 1235 listening; Webots console shows **no** human/skin/pedestrian references and **no** errors/warnings |
| Live NAO + MCP | `orbit_mcp_server.py --selftest --robot nao --sim webots` | **PASS** — `SELF-TEST PASSED (238.7 s, robot nao, simulator webots)`, exit 0; MCP connected to the live `orbit_nao` extern controller and exercised 49 tools (navigation, cleaning, safety-guard veto, stop-latch, scene load) |

---

## 7. Remaining errors

**None.** The restored NAO world loads cleanly in Webots R2025a with no human-related error, and the live NAO + MCP self-test passes. The experimental human model is fully removed from the active tree and archived (recoverable) under `_archive_human_experiment_20261025_124510/`.

To run the restored environment:
```
.\start_webots.ps1            # opens webots\worlds\orbit_house.wbt
.\run.ps1 -Mode server        # Webots + MCP bridge for the NAO
```
