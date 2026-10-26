# Phase 12 — MetaTool + Adaptive Webots Viewpoint: Integration Report

Date: 2026-10-25. New integration work only. Historical Phase 8–11 results are referenced, never recomputed or relabelled.
New evidence lives under `results_live/phase12_*`. Live = real Webots NAO on tcp://127.0.0.1:1235 (never 1240) + real Groq provider;
code checks touch neither robot nor LLM and are reported separately.

## 1. Repository audit — status matrix (at the start of Phase 12)

Legend: VERIFIED LIVE · CODE-CHECK ONLY · FAILED · BLOCKED · NOT IMPLEMENTED.

### MetaTool core (sections 3.1 / 3.2) — from the prior turn
| Capability | Status | Evidence |
|---|---|---|
| `plan_and_execute(instruction)` | VERIFIED LIVE | `results_live/m10_20261025/go_kitchen/` (plan→validate→execute→verify, true pose=kitchen) |
| `validate_plan(plan)` (no motion) | VERIFIED LIVE | `results_live/m10_20261025/validate_badtool/` (UNKNOWN_TOOL, 0 m moved) |
| `execute_plan(plan)` | CODE-CHECK ONLY | `tests/test_metatool_core.py`; live path shared with plan_and_execute |
| JSON schema per tool, local validation, typed args/deps/preconditions/effects/verify, `${ref}` ancestor rule, topo/cycle, refusal, refine, repair | CODE-CHECK + partially LIVE | `test_metatool_core.py`, `test_metatool_runtime.py`; refusal VERIFIED LIVE (`walk_through_wall`) |
| HumanoidGuard / stop-latch / loop guard / budgets / true-state verify | VERIFIED LIVE (guard, verify) + CODE-CHECK (loop/budget) | self-test + m10 traces |

### MetaTool advanced (section 3.3)
| Module | Status (start of Phase 12) |
|---|---|
| dryrun, candidates, plan_cache, clarify, constraints, scheduler, interrupts, skills, routing, tool_stats, explain, approval, certificate | NOT IMPLEMENTED |

### MCP interface (section 3.4)
| Tool | Status (start) |
|---|---|
| plan_and_execute / execute_plan / validate_plan | VERIFIED LIVE (48-tool `tools/list`) |
| plan_status, cancel_plan, resume_plan, interrupt, approve, deny, answer_clarification, list_skills, explain_last_run, get_metrics | NOT IMPLEMENTED |

### Viewpoint (section 4)
| Capability | Status (start) |
|---|---|
| `DEF ORBIT_VIEW Viewpoint` top-down overview in `orbit_house.wbt` | PRESENT (position 5,4,30 looking down) |
| `webots_adapter.look_at_rotation(pos,target)` (pure math, display-only) | PRESENT but in **TurtleBot/World2D legacy module** (imports world2d/continuous) → must be COPIED into a NAO-runtime module, not imported |
| `WebotsNao.show()` + `Supervisor.setLabel` HUD | PRESENT (6 lines, top-left) |
| follow / target / demo / reset camera modes | NOT IMPLEMENTED |

### Decision for this phase (what is implemented, with justification)
- **Adaptive viewpoint system (section 4)** — the Phase 12 headline and the piece that is bounded and live-verifiable now. Implement
  `orbit_ros/viewpoint.py` (overview/follow/target/demo/reset) using Webots' built-in Viewpoint `follow`/`followType`/`followSmoothness`
  first, reusing a COPY of the pure `look_at_rotation` math (no legacy import), wired into the live evidence pipeline.
- **certificate.py** — small, pure, directly supports the evidence pipeline (SHA-256 execution certificates) → implement.
- **explain.py / get_metrics** — derivable from existing traces → implement the read-only MCP tools.
- The remaining 10 advanced modules (candidates, plan_cache, clarify, constraints, scheduler, interrupts, skills, routing, tool_stats,
  approval) and their MCP tools remain **NOT IMPLEMENTED** this phase; each needs its own module + its own live acceptance. They are
  listed here honestly and NOT reported as complete.

(The rest of this report — live checks, results, limitations — is appended after the live runs.)

---

## 2. What was implemented this phase

- `orbit_ros/viewpoint.py` — adaptive viewpoint (overview/follow/target/demo/reset) using Webots built-in `follow`/`followType`/
  `followSmoothness`, with a copied (not imported) `look_at_rotation` so the NAO runtime stays free of the TurtleBot/World2D legacy module.
- `tools/live/webots_session.py` — viewpoint integrated into the evidence pipeline: `camera(mode)`, `render()` through the existing
  controller, per-screenshot `screenshots.json` metadata, start/end forced to overview.
- Code checks: `tests/test_viewpoint.py` (4), harness checks still green. Full `-m "not live"` suite: **550 passed, 2 skipped**.

## 3. Live checks executed (real Webots NAO; evidence saved)

| # | Check | Result | Evidence |
|---|---|---|---|
| 1 | House overview + reset is the default start view | VERIFIED LIVE | `results_live/p12vp_20261025b/.../start.png` (overview), `screenshots.json` |
| 2 | Built-in follow during real navigation | VERIFIED LIVE | trace `camera builtin=True follow=orbit_nao followType="Tracking Shot"`; `04_follow_arrived.png` hash differs from start |
| 3 | Camera modes produce distinct, correctly-tagged views | VERIFIED LIVE | 5 distinct SHA-256 across overview/follow/target/follow/demo; `screenshots.json` modes match |
| 4 | Target framing for a supported destination (kitchen entry) | VERIFIED LIVE | `03_target_kitchen.png` (target mode), computed from the verified kitchen entry point |
| 5 | Screenshot metadata records the REAL active camera mode | VERIFIED LIVE | `screenshots.json` (start/end=overview, 02=follow, 03=target, 05=demo) |
| 6 | Viewpoint change moves nothing (robot true pose) | VERIFIED LIVE | `true_poses.csv`: no translation during idle camera switches; verdict PASS, 0 fallbacks/resets |
| 7 | Controller-lifecycle limitation found + fixed | VERIFIED LIVE | `p12vp_20261025` (idle change → byte-identical) vs `p12vp_20261025b` (render → distinct); documented in `VIEWPOINT_SYSTEM.md` |
| — | MetaTool plan→validate→execute→verify (reused, prior turn) | VERIFIED LIVE (referenced) | `results_live/m10_20261025/go_kitchen/` |
| — | MetaTool refusal before motion (reused, prior turn) | VERIFIED LIVE (referenced) | `results_live/m10_20261025/walk_through_wall/` |

## 4. Bug found and fixed live (with before/after evidence)

**Viewpoint not rendered while the controller is idle between tool calls.** Root cause: the MCP extern controller does not `step()` between
calls, so a Viewpoint field write is not rendered before `exportImage`. Evidence: `p12vp_20261025` screenshots were byte-identical across
overview/follow/target (SHA `5fa826a8`). Fix: `Viewpoint.render(frames)` advances the **existing** controller a few steps (no second
controller, no robot motion command) before the screenshot. After: `p12vp_20261025b` screenshots are distinct per mode. The remaining
limitation (a change cannot be *seen* while fully paused) is documented, not worked around with a second Supervisor.

## 5. Acceptance matrix (end of Phase 12)

| Item | Status | Evidence / reason |
|---|---|---|
| MetaTool core intact (plan/validate/execute/verify/refuse) | VERIFIED LIVE | `m10_20261025/*` (prior turn), self-test 48 tools |
| Robot motion still guarded + verified | VERIFIED LIVE | all live verdicts true-pose judged, 0 fallbacks/resets |
| Adaptive viewpoint overview/follow/target/demo/reset in the GUI | VERIFIED LIVE | `p12vp_20261025b/*`, `VIEWPOINT_SYSTEM.md` |
| Screenshot metadata records the real camera mode | VERIFIED LIVE | `screenshots.json` |
| Built-in Webots following preferred | VERIFIED LIVE | trace `builtin=True` |
| MCP extended interface (plan_status/cancel/resume/interrupt/approve/deny/answer_clarification/list_skills/explain_last_run/get_metrics) | NOT IMPLEMENTED | each needs its own module + live acceptance; listed honestly, not claimed |
| Advanced MetaTool: dryrun/candidates/plan_cache/clarify/constraints/scheduler/interrupts/skills/routing/tool_stats/explain/approval/certificate | NOT IMPLEMENTED | out of this session's bounded scope; status recorded |
| An MCP `camera(mode)` tool | NOT IMPLEMENTED | the `Viewpoint` object is live-verified via the harness; an MCP wrapper is a small follow-up |

## 6. Limitations (honest)
- The extended MCP MetaTool interface (3.4) and the 13 advanced modules (3.3) are **not implemented** this phase — only the viewpoint
  system, which was the bounded, live-verifiable Phase 12 headline, plus reuse of the already-live MetaTool core.
- A viewpoint change is invisible while the simulation is fully paused; it renders on the next step (documented in §4).
- A small standing-yaw settle (~gravity) occurs during idle render steps; it is recorded in `true_poses.csv` and is not a commanded motion.
- Gemini remains unreachable from this machine; LLM work uses Groq (consistent with Phases 9–11).
- Protected files (`orbit_house.wbt`, `house.py`, `webots/protos/`) were not modified.
