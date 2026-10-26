# ORBIT Human Controller (Phase 13)

`orbit_ros/human_controller.py` drives the animated human actor — a Webots `Pedestrian` (`DEF ORBIT_HUMAN`) — **independently** of the NAO.

## Independence & lifecycle
- Own module, own state, own command namespace (`human_*`), own target. It never calls the NAO's motors and never impersonates NAO motion.
- It uses a Supervisor handle **passed in by the caller** (the existing `WebotsNao.sup`). It does **not** create a second Supervisor or a
  second controller process — nothing competes for port 1235. Stepping is the caller's existing loop (`WebotsNao.step`), so the human and
  NAO advance on one clock. (Chosen as the safest lifecycle compatible with the single-extern-controller architecture; the controllers are
  logically independent even though coordinated in the one allowed process.)

## Coordinate frame
House ENU (x east, y north, z up), identical to `house.py`/the NAO. The Pedestrian origin sits at z≈1.27 (feet on the floor); rotation is
about +z. Verified against the installed `Pedestrian.proto` default.

## Commands
| Command | Effect | Status it returns |
|---|---|---|
| `idle()` | stand, clear target/route | COMPLETED |
| `set_pose(x,y,yaw_deg)` | place the human (scene setup only, not mid-task) | COMPLETED |
| `reset()` | return to the validated initial pose captured at construction | COMPLETED |
| `pause()` / `resume()` | pause/continue the current route | ACCEPTED |
| `turn_to(x,y)` | face a point (animated) | COMPLETED/FAILED |
| `walk_to(x,y)` | walk (animated) to a point | COMPLETED/TIMED_OUT/FAILED |
| `follow_route([(x,y)…])` | walk a scripted route | COMPLETED/TIMED_OUT/FAILED |
| `update()` | advance one frame (non-blocking use) | — |
| `report()` | `{available, state, status, pose, target, route_remaining, paused, error}` | — |

Config: `speed` (m/s along the path), `waypoint_tol` (m), `timeout_s`. Movement is **animation-driven** (torso translation + cycled
leg/arm angle fields for a visible gait), explicitly not physics locomotion.

## Collision / separation
The Pedestrian is generated with `enableBoundingObject TRUE` (native collision proxy). Human–NAO separation is computed at the application
level from the two Supervisor-read ground-truth poses and is labelled as such; native physics contact was not stress-tested in Phase 13.

## Safety
A failed human action sets `status=FAILED`/`state=error` and never touches the NAO. The human controller has no access to `robot_call` or
the NAO motors (enforced/tested). NAO motion continues to go only through `HumanoidSession.robot_call` → guard → motion.

## Live evidence
`results_live/phase13_20261025/human_walk/` — the human walked a scripted route (1.443 m, COMPLETED) while the NAO drifted 0.0 m and stayed
upright (independent controllers), judged from the true Supervisor poses.

## Realistic human candidate (Phase 16)

The human actor is the Webots `Pedestrian`, which is a genuinely skinned, articulated figure — a **sculpted head mesh with real facial
geometry** (`PedestrianHead.proto` is an `IndexedFaceSet` of ~200 vertices forming brow, nose, chin and ears), separate hand/foot/neck/torso
meshes, 13 hinge joints and `PBRAppearance` skin and clothing. This is **not** a texture painted on a sphere. It is the most realistic human
installed with this Webots R2025a build; no separate photo-real named skin (e.g. a standalone `Robert`/`Sandra` character) is installed, and
pulling an external licensed model into a live professor demo was out of scope.

Phase 16 adds a NEW candidate world **`webots/worlds/orbit_house_human_realistic.wbt`**, generated additively by
`tools/make_realistic_human_world.py` (it reuses the approved house generator, preserves every house line, and only inserts the Pedestrian
with tuned natural `skinColor`/`shirtColor`/`pantsColor`/`shoesColor` PBR values). It keeps `DEF ORBIT_HUMAN` and `name "orbit_human"`, so
`HumanController` drives it **unchanged**. It does not touch any protected file nor the Phase-13 candidate `orbit_house_human.wbt`. Asset
provenance and the Cyberbotics proto license are recorded under
`results_live/phase16_visual_clarity_20261025T034432Z/assets_and_licenses/`.

Live status: the camera's `human` framing mode was verified live against the human world currently loaded in Webots (human node present and
framed, NAO did not move — see the Phase-16 report). Rendering the `_realistic` candidate specifically requires loading that world in the
Webots GUI (one extern controller per process); that GUI reload is a user step and is marked PARTIAL in `docs/PHASE16_VISUAL_CLARITY_REPORT.md`.
