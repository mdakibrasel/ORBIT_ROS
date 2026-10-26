# ORBIT Adaptive Webots Viewpoint System (Phase 12)

`orbit_ros/viewpoint.py` adapts the Webots R2025a camera for clear real-time visualization of the NAO in the 3D house. **Display only:**
a viewpoint change never calls the LLM, never issues a robot motion command, never resets physics, and never changes the robot pose. It
writes the Viewpoint node's fields through the **existing** `WebotsNao` Supervisor (no second Supervisor, no competing controller loop).

## Modes

| Mode | What it does | Implementation |
|---|---|---|
| `overview` | **(Phase 16)** angled bird's-eye of the whole house (SW and above, looking at the centre) so the ~0.57 m NAO and furniture read as 3-D objects, not a flat floor-plan | computed from the real 10×8 m footprint; camera clamped to ≥ `MIN_CAM_H` (2.8 m) |
| `reset` | restores the **saved** `DEF ORBIT_VIEW` straight-down view at (5,4,30) exactly as stored in the world file | the restorable default; distinct from the adaptive `overview` since Phase 16 |
| `follow` | the camera follows the walking NAO | Webots **built-in** Viewpoint `follow`=`orbit_nao`, `followType`="Tracking Shot", `followSmoothness`; a static chase-cam fallback only if those fields are absent |
| `closeup` | **(Phase 16)** tight three-quarter shot of the NAO (camera ~1.6 m away, looking at mid-torso) so the robot fills the frame | static, computed from the live robot position; camera clamped to ≥ `MIN_CAM_H` |
| `target` | frames the NAO and a target point together | reuses a self-contained `look_at_rotation` (copied, not imported, so the NAO runtime never pulls in the TurtleBot/World2D legacy `webots_adapter`); uses verified positions |
| `human` | **(Phase 16)** frames the animated human actor (`DEF ORBIT_HUMAN`) the same way; `ok:false` if no human node is in the loaded world | reads the human node position through the existing Supervisor |
| `demo` | built-in follow + the HUD labels | `follow` + `WebotsNao.show()`/`setLabel` driven by the trial/executor |

## Zoom (Phase 16)

`Viewpoint.zoom(amount)` **dollies** the camera along its view axis toward / away from the current look-at point. `amount` is `"in"`,
`"out"`, or a positive number (a multiplier on the current distance — `0.5` = twice as close, `2.0` = twice as far). It deliberately
does **not** change the field of view, so perspective and geometry stay honest. The distance is clamped to `[ZOOM_DIST_MIN, ZOOM_DIST_MAX]`
= `[1.5 m, 40 m]` and the camera height to ≥ `MIN_CAM_H` (2.8 m), so a zoom can never bury the camera in the floor or pass through the
look-at point. Zooming disables built-in follow first (a dolly only makes sense on a static framing). `smoothing_alpha(dt, tau)` =
`1 − exp(−dt/τ)` is provided for optional exponential glides.


## HUD (research/demo)

Reuses the existing `WebotsNao.show()` / `Supervisor.setLabel` (top-left, 6 lines) rather than a duplicate label system. The live harness
`LiveTrial.hud(step=, verify=)` writes: instruction, current step, last verification. Planned/running/verified states are distinguishable
by the executor's per-step status in the trace and the HUD text.

## Controller-lifecycle handling (honest)

The MCP extern controller does not call `step()` continuously between tool calls (expected). A Viewpoint field write therefore does not
**render** until the next simulation step. Measured directly (`results_live/p12vp_20261025/`): a mode change made while the sim was idle
produced a **byte-identical** screenshot (hash unchanged) until the next step, whereas the follow view during actual navigation (many
steps) changed correctly.

Fix within the rules: `Viewpoint.render(frames)` advances the **existing** controller a few steps so the new view renders before a
screenshot. This uses the one Supervisor we already have — no second controller, no second loop — and stepping with no motor command does
not translate the robot. After the fix (`results_live/p12vp_20261025b/`): overview / follow / target / follow-arrived / demo screenshots
are all **distinct** (different SHA-256), each correctly mode-tagged, and the robot's true-pose timeline shows no translation during the
idle camera switches (a small standing-yaw settle under gravity is visible and recorded, not a commanded motion).

Limitation: a camera change cannot be *seen* while the simulation is fully paused; it renders on the next step. Ordinary following during
movement is smooth because the robot's own motion advances the simulation.

## Evidence integration

`LiveTrial` (`tools/live/webots_session.py`):
- `start.png` and `end.png` are always captured in **overview** (the harness forces overview before each).
- `camera(mode, target_xy=, **kw)` switches mode, renders, and records the event.
- `screenshot(name, mode=, phase=)` records per-image metadata to `screenshots.json`: `{file, camera_mode, phase(before|during|after), t}`.
  The mode is read from the active camera; an image is never mislabeled (verified: `start/end=overview`, `02=follow`, `03=target`, `05=demo`).

## Live acceptance (Phase 12)

`results_live/p12vp_20261025b/viewpoint_nav/` — PASS: during a real `go_to_room kitchen`, the camera switched overview→follow→target→follow→demo,
captured 7 mode-tagged screenshots (start/end overview), the robot reached the kitchen by true pose (4.13, 5.65), **0 fallbacks / 0 supervisor
resets**, and the five mode screenshots have distinct hashes. Built-in follow confirmed used (`builtin=True follow=orbit_nao followType="Tracking Shot"`).

## Controls

Call `LiveTrial.camera("overview"|"follow"|"closeup"|"target"|"human"|"demo"|"reset", target_xy=…, zoom=…)`. Since Phase 16 the same
`Viewpoint` object is also wrapped by the MCP **`camera(mode, zoom?, target?)`** tool and the **`/view`** chat command (see
`docs/COMMAND_MATRIX.md`); both are intentionally NOT robot tools and perform no motion, no LLM call and no memory change.

## Phase 16 extension and live acceptance

`results_live/phase16_visual_clarity_20261025T034432Z/camera/` — **PASS**. Against the live Webots NAO house, the harness exercised all
seven modes and a zoom in/out through the one display-only `Viewpoint`, captured a native `sup.exportImage` screenshot per mode, and
sampled the true pose throughout:
- all seven modes returned `ok:true`; built-in follow confirmed used;
- the seven mode screenshots are **distinct** (different MD5); 13 of 14 captured images are distinct;
- **robot moved 0.0006 m** across all camera changes (true-pose delta), 0 fallbacks, 0 supervisor resets — proving display-only;
- the live MCP self-test lists **49 tools including `camera`** and passed (167 s).

Code checks (no Webots): `tests/test_camera.py` (22) and `tests/test_viewpoint.py` (5) cover the geometry (angled overview, look-at,
zoom dolly + clamp, smoothing), the `camera()` session side-effect contract (no motion, no memory write, safe no-op without a viewpoint),
the MCP tool registration/dispatch, and the `/view` parser.

