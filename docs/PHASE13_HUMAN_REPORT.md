# Phase 13 — Animated Human Actor + Dedicated Human Controller: Implementation Report

Date: 2026-10-25. Additive, non-destructive work. Protected files (`webots/worlds/orbit_house.wbt`, `orbit_ros/house.py`,
`webots/protos/`) were **not modified**. New live evidence: `results_live/phase13_20261025/`. Historical Phase 8–12 results untouched.
Live = real Webots R2025a NAO on tcp://127.0.0.1:1235 (never 1240); code checks touch no robot/LLM and are reported separately.

## 1. Assets actually verified (installed R2025a)
- `projects/humans/pedestrian/protos/Pedestrian.proto` — an **articulated** human (torso/neck/arms/legs/hands/feet via HingeJoints), with
  configurable colours, `name`, `controller`, `controllerArgs`, `bodySlot`, `enableBoundingObject`, and hidden joint-angle fields
  (`leftLegAngle`, `rightArmAngle`, …). **Chosen** as the animated human actor.
- `projects/humans/skin_animated_humans/` — only `CharacterSkin.proto` + BVH motions (walk/run/jump/step). **No** `Robert.proto` /
  `Sandra.proto` / `Sophia.proto` standalone skins are installed here, so those named skins were NOT referenced (honest asset check).

## 2. Approach (resolves the generator-vs-protected-file tension)
`tools/make_house_world.py` generates the world entirely from the protected `house.py`. To add the human through the approved generator
**without editing a protected file**, a new additive generator `tools/make_human_world.py` takes the house text from the approved
generator (unchanged) and inserts the Pedestrian EXTERNPROTO + node, writing a **new candidate** `webots/worlds/orbit_house_human.wbt`.
`orbit_house.wbt` is never overwritten (verified: timestamp unchanged; all 148 original house lines present in the candidate, 0 missing).

## 3. Dedicated Human Controller — `orbit_ros/human_controller.py`
Logically independent of the NAO: own module, own state, own command namespace (`human_*` at the tool layer), own target. It drives the
`DEF ORBIT_HUMAN Pedestrian` node through a Supervisor handle **passed in by the caller** (the existing `WebotsNao.sup`) — it creates **no
second Supervisor and no second controller process**, so nothing competes for port 1235, honouring the single-extern-controller lifecycle.
Stepping is the caller's existing loop (`WebotsNao.step`), so human and NAO share one clock.

Commands: `idle`, `set_pose`, `reset` (to the validated initial pose), `pause`/`resume`, `turn_to(x,y)`, `walk_to(x,y)`,
`follow_route([wps])`, `update()` (non-blocking frame), `report()`. Configurable `speed`, `waypoint_tol`, `timeout_s`. Structured statuses:
`ACCEPTED`/`RUNNING`/`COMPLETED`/`TIMED_OUT`/`FAILED`. Movement is **animation-driven** (torso translation along waypoints + cycled
leg/arm angle fields for a visible gait) — explicitly **not** physics locomotion; no claim of physically realistic human walking.
It never calls the NAO motors (`test_command_namespace_is_human_only` asserts no `robot_call`/`play`/`nao` attrs).

## 4. NAO integrity
The NAO is untouched by this phase: its model, extern controller on 1235, motors, guard, stop-latch and call budget are unchanged. The
live check confirmed the NAO drifted **0.0 m** and stayed upright while the human walked — the human controller cannot corrupt NAO state.

## 5. Collision / separation (honest labelling)
The Pedestrian is generated with `enableBoundingObject TRUE` (a native collision proxy). Human–NAO separation in the live check is an
**application-level distance** computed from the two Supervisor-read poses (ground-truth), labelled as such — not a claim of native
physics contact. Native contact behaviour was not stress-tested this phase.

## 6. Live checks executed
| # | Check | Result | Evidence |
|---|---|---|---|
| Stage B | Candidate world generated + validated, house+NAO preserved | PASS | `make_human_world.py --validate` OK; 148/148 house lines preserved; `orbit_house.wbt` unchanged |
| Stage C | Human node present & articulated in the live world | PASS | `results_live/phase13_20261025/human_walk/trace.jsonl` (`human_node present=True`) |
| Stage C | HumanController walks the actor along a scripted route | PASS | moved **1.443 m**, status COMPLETED, ended (3.18, 2.26) within 0.3 m of goal (3.2, 2.4) |
| Stage C | Controllers independent — NAO untouched while human moves | PASS | NAO drift **0.0 m**, upright; 0 fallbacks/resets |
| — | Mode-tagged screenshots (target mode) of human start/end | PASS | `01_human_start.png`, `02_human_end.png`, `screenshots.json` |

## 7. Code checks (separate from live)
`tests/test_human_controller.py` (10): generator preserves house + adds Pedestrian; walk/route/turn/pause/resume/reset; timeout reported;
no-node fails safely; the `human_*` controller exposes none of the NAO motor API. Full `-m "not live"` suite: **560 passed, 2 skipped**.

## 8. Acceptance status (section 19)
| Criterion | Status | Note |
|---|---|---|
| Genuinely articulated animated human from verified assets | PASS (live) | Pedestrian.proto, jointed, gait-animated |
| Dedicated Human Controller with deterministic scenarios | PASS (live) | walk/route/turn/reset verified; idle/pause/resume + waypoints |
| Generated world preserves house + NAO | PASS | 148/148 lines; protected files untouched |
| Human/NAO commands routed to independent controllers; one failure doesn't corrupt the other | PASS (live) | NAO drift 0.0 m during human walk; separate namespaces |
| NAO remains physics-driven, safety intact | PASS | NAO unchanged; guard/stop-latch/budget untouched |
| MetaTool `human_*` tool contracts in the planner | NOT IMPLEMENTED | controller + statuses exist; MCP/MetaTool `human_*` tools are a follow-up |
| Speaker lifecycle audit / interpolation (sec 6–7) | NOT RUN | out of this session's scope (controller + human was the focus) |
| Person-following / look-at / social interaction (sec 8) | NOT IMPLEMENTED | needs the human in the live NAO map + a follow tool; follow-up |
| Native physics collision between human & NAO | NOT VALIDATED | bounding object enabled; contact not stress-tested; app-level distance only |

## 9. Limitations / NOT RUN (honest)
- The human is **animation-driven**, not physics-locomoted (stated, not hidden).
- Human–NAO **native collision** not stress-tested; only Supervisor-ground-truth distance was used, labelled application-level.
- MetaTool `human_*` tools, person-following, look-at/social gestures, Speaker/interpolation audits, perception/visualization/voice/dashboard,
  and the advanced-experiment sections are **NOT IMPLEMENTED / NOT RUN** this session and are recorded here rather than claimed.
- Gemini remains unreachable from this machine; no LLM was needed for this phase (pure controller + world work).

## 10. Files
- New: `orbit_ros/human_controller.py`, `tools/make_human_world.py`, `webots/worlds/orbit_house_human.wbt` (candidate, additive),
  `tools/live/p13_human_live.py`, `tests/test_human_controller.py`, this report.
- Backups: `_work_backup/20261025_023417/`. Protected files unmodified.
