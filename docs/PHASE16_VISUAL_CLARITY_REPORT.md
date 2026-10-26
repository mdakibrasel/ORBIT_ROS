# Phase 16 — Visual Clarity: Adaptive Camera + Realistic Human

Goal: make the live Webots demo clearer by (1) extending the adaptive camera to seven framing modes plus zoom, (2) exposing the camera as
an MCP `camera` tool and a `/view` chat command, and (3) providing a genuinely realistic, skinned human actor — all verified live, with no
fakes, no teleport, and without touching protected files or historical results.

Evidence root (camera): `results_live/phase16_visual_clarity_20261025T035901Z/`. Evidence root (realistic human, canonical):
`results_live/phase16_realistic_human_20261025T041933Z/`.

## What changed (files)

| File | Change |
|---|---|
| `orbit_ros/viewpoint.py` | **Extended + fixed.** Added `closeup`, `human`, `zoom` (dolly, not FOV); angled `overview`; `smoothing_alpha`. **Framing fixes:** `_interior_camera_pos` + `_frontal_camera_pos` keep the camera inside the house and in front of the human (face-visible); `_aim`/`_look` make `zoom` dolly toward the current subject (not always the robot). Display-only contract and single-Supervisor render model preserved; `OVERVIEW` kept as an alias of the saved view. |
| `orbit_ros/humanoid.py` | **Extended.** Added `HumanoidSession.camera(mode, zoom?, target?)`: builds/caches one `Viewpoint` on the live backend, dispatches a mode, optionally dollies, renders on the existing controller, returns a structured dict. Never calls the LLM, never moves the robot, never writes memory. Safe no-op in `--sim fake`. |
| `orbit_ros/mcp_server.py` | **Extended.** Registered the `camera` tool in `H_SESSION_TOOLS` (read-only annotation) and dispatched it in `HumanoidMcpServer.call_tool`. Tool count 48 → **49**. |
| `orbit_chat.py` | **Extended.** Added `parse_view_command`, `_parse_zoom_amount`, `view_turn`, and a `/view` (alias `/camera`) intercept in `handle` so view commands call the `camera` tool directly and are never routed through the LLM. |
| `tools/live/webots_session.py` | **Extended + fixed.** `LiveTrial.camera` now supports `closeup`, `human`, `zoom`; and `__exit__` now renders the overview before capturing `end.png` (previously the field write was not stepped, so `end.png` could show the prior frame). |
| `tools/make_realistic_human_world.py` | **New.** Additive generator → `webots/worlds/orbit_house_human_realistic.wbt` (tuned skin/clothing PBR; `DEF ORBIT_HUMAN`). |
| `tools/live/phase16_camera_trial.py` | **New.** Live validation runner for the camera + human. |
| `start_webots.ps1` | **Extended.** Optional `-World <path>` parameter (defaults unchanged to `orbit_house.wbt`). |
| `webots/worlds/orbit_house_human_realistic.wbt` | **New candidate** (generated; additive, 151 house lines preserved + 11). |
| `tests/test_camera.py`, `tests/test_realistic_human_world.py` | **New** code checks. `tests/test_viewpoint.py` updated for the angled-overview/saved-view split. |
| `docs/VIEWPOINT_SYSTEM.md`, `docs/HUMAN_CONTROLLER.md`, `docs/COMMAND_MATRIX.md` | **Updated.** |

Backups of every edited file: `_work_backup/phase16_20261025_032659/`.

## Camera: modes and zoom

Seven modes — `overview` (angled bird's-eye), `follow` (built-in tracking), `closeup` (tight on the NAO), `target` (NAO + a point),
`human` (frame `DEF ORBIT_HUMAN`), `demo` (follow for a walkthrough), `reset` (restore the saved straight-down view). `zoom` dollies the
camera along its view axis (`in`/`out`/numeric factor), clamped to `[1.5 m, 40 m]` and height ≥ 2.8 m; it does **not** change FOV. Details
in `docs/VIEWPOINT_SYSTEM.md`.

Display-only contract (enforced and tested): a `camera` call never calls the LLM, never issues a robot motion, never resets physics, never
changes the robot pose, and never writes memory. It writes only the Viewpoint node through the existing Supervisor; `render()` steps the one
controller with no motor command.

## MCP tool and chat command

- MCP: `camera(mode, zoom?, target?)` — read-only annotation, structured return `{status, mode, zoom, available, ...}`. In `--sim fake` or
  without a Webots viewpoint it returns `status:ok, available:false` and does nothing.
- Chat: `/view [mode] [zoom …]` and `/view zoom …` and `/view target X Y` — parsed locally and sent straight to the `camera` tool, never to
  the model (so a view command is never mistaken for a robot task). `/camera` is an accepted alias.

## Realistic human

The actor is the Webots `Pedestrian` — the most realistic human installed with this R2025a build: a sculpted `IndexedFaceSet` head with real
facial geometry (brow/nose/chin/ears), separate hand/foot/neck/torso meshes, 13 hinge joints, and `PBRAppearance` skin and clothing. This
is **not** a texture on a sphere. No standalone photo-real named character skin is installed; an external licensed model was out of scope for
a live demo. Phase 16 therefore ships a **new** candidate world `webots/worlds/orbit_house_human_realistic.wbt` with tuned natural skin and
clothing colours, generated additively by `tools/make_realistic_human_world.py`, keeping `DEF ORBIT_HUMAN`/`name "orbit_human"` so
`HumanController` drives it unchanged. Asset provenance + license: `…/assets_and_licenses/pedestrian_provenance.txt`.

## Verification

### Code checks (no robot, no LLM, no Webots) — PASS
- `tests/test_camera.py` — 22 checks: viewpoint geometry (angled overview, look-at, zoom dolly/clamp, smoothing), the `camera()` session
  side-effect contract (no robot motion, no memory write, safe no-op, all modes, target requires a point, unknown mode rejected), the MCP
  tool registration/dispatch (read-only annotation), and the `/view` parser (all forms + errors).
- `tests/test_realistic_human_world.py` — 7 checks: generator is additive (every house line preserved), realistic human fields present,
  house/NAO preserved, never targets a protected file, DEF matches `HumanController`, on-disk candidate current.
- `tests/test_viewpoint.py` — 5 checks (updated): saved view is straight-down; adaptive `overview()` is angled above the floor.
- Full suite: `pytest -m "not live"` (slow fake-sim guard test deselected per standing practice) — **594 passed, 2 skipped, 1 deselected** (226 s). Report: `…/test_reports/phase16_full_code_checks.txt`.
  Report: `…/test_reports/phase16_code_checks.txt`.

### Live checks (Webots R2025a, port 1235, real controller) — PASS
Runner: `tools/live/phase16_camera_trial.py`. Camera-only evidence: `…T035901Z/camera/`. **Realistic-human world evidence (canonical):**
`results_live/phase16_realistic_human_20261025T041933Z/`.
- All **seven** camera modes returned `ok:true`; `zoom in` and `zoom out` ok.
- Native `sup.exportImage` screenshots per mode; **all 14 captured images are distinct** (different MD5), seven mode images included.
- **Robot moved 0.0006 m (camera run) / 0.0001 m (realistic run)** across all camera changes; 0 fallbacks, 0 supervisor resets — proving
  the camera is display-only. Verdict: **PASS**.
- **Realistic human rendered live and visually confirmed.** The new `orbit_house_human_realistic.wbt` was loaded in Webots
  (`./start_webots.ps1 -World …`), the controller connected on 1235, and the `ORBIT_HUMAN` Pedestrian was confirmed at its candidate start
  (2.2, 1.2, 1.27) — distinct from the Phase-13 position. The captured `camera_human.png` / `human_model.png` / `human_model_closeup.png`
  were **inspected as images** and show a skinned, articulated figure (sculpted head, red shirt, dark trousers, skin-tone arms/hands),
  correctly framed and filling the shot.
- Live MCP self-test against the realistic world: **PASSED (229 s)**, `tools/list` = **49 tools including `camera`**, guard
  rejected/vetoed/stop-latch all confirmed. Report: `…/test_reports/phase16_mcp_selftest_realistic.txt`.

### Bugs found by visual inspection and fixed (error-driven)
Trusting `ok:true` was not enough — inspecting the actual PNGs revealed two real framing defects, each fixed and re-verified live:
1. **Human/closeup framed from outside a wall.** `human`/`closeup` placed the camera at `subject + dist·(cos b, sin b)` with a fixed
   bearing; for a subject near a corner (human at (2.2,1.2)) this put the camera outside the house, so the shot showed a wall/floor. **Fix:**
   `_interior_camera_pos` places the camera on the house-interior side of the subject and clamps it inside the footprint; `human` additionally
   uses `_frontal_camera_pos` (reads the human's yaw) to face the figure's front. Regression tests: corner-robot and corner-human stay inside
   the house; the human camera is in front of a +x-facing human.
2. **Zoom snapped back to the robot.** `zoom`/`_current_look` always dollied toward the robot, so `human` + zoom re-aimed at the NAO (the
   close-up showed the robot from above). **Fix:** every mode records its look-at via `_aim`, and `zoom` dollies toward that remembered point.
   Regression test: zoom after `human()` stays on the human, not the robot.
Also fixed a harness bug where `end.png` was not rendered before capture (duplicate frame).

## Honest limitations / not-yet-verified

- **RESOLVED — the `_realistic` candidate world was rendered live** (was PARTIAL). `orbit_house_human_realistic.wbt` was loaded in the Webots
  GUI via `./start_webots.ps1 -World webots\worlds\orbit_house_human_realistic.wbt`, the controller connected on 1235, and the realistic
  human (Pedestrian at (2.2,1.2)) was captured and visually confirmed in `results_live/phase16_realistic_human_20261025T041933Z/`.
- **Face detail is geometry-only, not textured.** The Pedestrian head is a sculpted `IndexedFaceSet` (brow/nose/chin/ears are real
  geometry), but the model has **no facial texture** (no painted eyes/eyebrows/lips), so from a distance the eyes/mouth read faintly. The
  three-quarter `camera_human.png` is the clearest face view. This is a limitation of the only realistic human installed with this Webots
  build; a photo-real textured head would require an external licensed asset, which was out of scope for a live professor demo.
- **`closeup` looks down fairly steeply** because the camera height is clamped to ≥ 2.8 m (to clear furniture) while the NAO is ~0.57 m tall
  and the camera stands ~1.8 m away; the robot is well framed but seen from above. A lower clamp would flatter the robot at the cost of
  occlusion risk; the current tradeoff favours never clipping into furniture.

## Integrity (verified after the phase)
- Protected files unchanged by timestamp/size: `orbit_house.wbt` (10/23 23:25:11, 16931 B), `orbit_ros/house.py` (10/23 23:24:32),
  `webots/worlds/orbit_house_human.wbt` (Phase-13 candidate, 10/25 02:39:39). `.bak` files preserved.
- Port **1240**: 0 connections (direct 1235 only).
- All historical `results_live/` and `results_nao_benchmark/` directories intact; no measured result overwritten or re-labelled.
