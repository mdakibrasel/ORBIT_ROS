# ORBIT meta-tool — Requirements

Derived from sections 2–11 (incl. 4b) of the program brief. Format: user stories + EARS acceptance criteria.
Governing rule (steering `orbit-rules.md`): **LIVE ONLY** — the robot is always the live Webots NAO, LLM calls always hit a real
provider, and any test touching a robot or LLM is a live test with an evidence pack. Pure code checks (JSON/schema/validator/SQLite/docs)
may run without Webots and are never presented as robot results.

## R2 — Live test harness (built first; everything depends on it)
**Story:** As an evaluator, I want a harness that runs one fresh Webots process per trial and captures objective evidence, so every later
claim is backed by true-pose measurements.
- WHEN a trial starts and port 1235 has no listener, THE harness SHALL start Webots via `start_webots.ps1` and wait until the world is ready.
- THE harness SHALL run exactly one Supervisor-bound Python process per trial and SHALL refuse to run if another extern controller is connected.
- FOR each trial THE harness SHALL write an evidence pack under `results_live/<run>/<trial>/` containing `start.png`, `end.png`,
  `trace.jsonl`, `plan.json`, `metrics.json`, `true_poses.csv` (sampled every 0.5 s), `webots_console.txt`, and `verdict.json`.
- FOR every robot tool and session tool in the registry THE harness SHALL have ≥1 live case with an objective pass condition computed
  from the true simulator state, and SHALL write `tool_checks.json`/`tool_checks.md`.
- Disturbances (fall via `Node.addForce`, obstacle as a real Solid, moved object as a world variant) SHALL be physical and logged, never a teleport during a task.
- `run.ps1 -Mode live-check` SHALL run the harness + tool checks; `-Mode live-bench` SHALL run the task suite.
- A baseline `live-check` on the CURRENT tools SHALL be saved as `results_live/<ts>_baseline/` before other changes.

## R3 — Honesty fixes in the Webots backend (Phase 1)
- THE kinematic fallback in `WebotsNao.play()` SHALL be OFF by default; enabled only by `--allow-kinematic-fallback` (debug); when off a
  non-moving motion SHALL return `status:"failed"` with the measured value.
- `_t_stand_up` SHALL play the real `StandUpFromFront.motion` and verify upright; the supervisor reset SHALL exist only behind
  `--allow-supervisor-reset` (debug, counted, trial invalid).
- Fall recovery SHALL use the real `stand_up`, ≤2 times per task, every fall/recovery logged.
- Any trial with a fallback or reset SHALL be marked `invalid` by benchmarks and `live_task_check`.

## R4 — Meta-tool package (Phase 2) `orbit_ros/metatool/`
- THE meta-tool SHALL be a new package leaving `pel/` and `run_plan` working.
- A plan SHALL be produced by a schema-constrained LLM call, statically validated and self-refined (≤3 rounds) BEFORE any motion.
- IF a plan is `feasible:false` THE agent SHALL `say` the reason and stop with 0 motions.
- Steps SHALL execute in topological order with NO LLM call between steps; each step runs precondition check → loop guard →
  `session.robot_call` (unchanged validation+guard+motion) → verifier.
- ON step failure THE agent SHALL `skip`/`abort`/`repair` per the step's `on_fail`; repair plans the remaining work only, re-validated with `done_ids`.
- THE only non-plan action after a fall SHALL be the real `stand_up` reflex.
- MCP tools `validate_plan`, `execute_plan`, `plan_and_execute` SHALL be added; `run_plan` SHALL stay.

## R4b — Advanced features (Phase 2b)
- Map dry-run (`dryrun.py`): a validated plan SHALL be simulated on the house map (A* + guard logic) from the live pose with NO motion;
  an unreachable step SHALL reject the plan before motion (code `UNREACHABLE`).
- Candidates (`candidates.py`): K plans (default 3) SHALL be validated+dry-run+scored; the lowest-score feasible plan runs.
- Plan cache (`plan_cache.py`): a plan verified live SHALL be cached by `goal_key`; a cache hit SHALL be offered as a candidate with no
  LLM call, re-validated and dry-run against current facts; dropped when `failures≥2 and failures>successes`.
- Clarify (`clarify.py`): THE executor SHALL `ask_user` (no motion) on target ambiguity, unknown target, candidate disagreement, or request+negation.

## R5 — Fix existing tools (Phase 3)
- Add `turn_by` (arbitrary angle, closed loop) and `walk(distance_m)` (closed loop); keep old names as hidden aliases.
- `--calibrate` SHALL measure real gait/turn effects into `configs/nao_calibration.json`.
- `--pose-noise SIGMA_M,SIGMA_DEG` SHALL be off by default; when on, guard/planner use noisy pose, true pose only for evaluation, reports state it.
- `search_github` SHALL be removed from the robot MCP server; cleaning tools grouped into `clean(scope,...)`; status merged into `get_status(sections?)`; old names hidden aliases.

## R6 — New tools (Phase 4)
- Each new tool in the section-6 table SHALL have typed args, requires/effects/verify, and a LIVE pass condition with an evidence pack.
- World changes SHALL go through `tools/make_house_world.py` (+ `--check`), never by hand-editing `.wbt`.

## R7 — Memory v3 (Phase 5)
- In-place migration to `schema_version=3`; structured goals from `goal_struct`; fixed `normalize_goal` fallback with the listed unit tests
  (incl. negation, question vs task, Bengali).
- Tables: `aliases` (EN+BN seeds), `places`, `object_state` (staleness on >0.5 m disagreement); confidence decay; optional embedding hook (off).
- Invariants preserved: advice only, guard never reads it, broken db never stops the robot, observations override memory.

## R8 — Live evaluation (Phase 6) `results_live/phase11_<ts>/`
- Strategies A/B/C/D + D ablations; large+small models; the section-8 task set; ≥10 trials per (task,strategy,model), one fresh process each,
  judged from true pose; report with Wilson intervals generated only from data by `tools/live/make_report.py`.

## R9 — Repository hygiene (Phase 7)
- Measured results SHALL be moved unchanged into `evidence/` with `MANIFEST.json` (path, sha256, date) and `README.md`; paths updated; tests still pass.
- Fakes SHALL be removed from the running system (FakeNao, `--sim fake`, mocks, oracle, World2D → `legacy/`); a check SHALL fail if
  `FakeNao`/`sim="fake"`/`--sim fake`/`Mock`/`Scripted`/`oracle` appears outside `legacy/`, `docs/history/`, `evidence/`.
- One version source (`orbit_ros.__version__`, keep 0.16.5); `tests/test_consistency.py`; generated tool count/table and test count via `tools/gen_docs.py`.
- `tools/make_release.py` SHALL build a whitelist zip excluding `.env`, `.venv`, backups, logs; it SHALL fail if a forbidden path is present.

## R10/R11 — README + presentation mode
- `README.md` rewritten (English) with the architecture diagram and generated numbers only.
- `run.ps1 -Mode demo` / `-Mode demo-script` SHALL run live with the real provider and record evidence packs per instruction.

## Cross-cutting (always)
- Port 1235 only; never 1240; MCP stdio preserved; `.env` never printed/logged/committed (read only via `env_config.load_env()`).
- Back up a file into `_work_backup/<ts>/` before editing.
- Never delete/modify existing measured results.

## Known environmental constraints (must be stated, not worked around)
- One Supervisor per process (Webots); a fresh process per trial.
- Live navigation trials take ~200–550 s each; the full Phase-6 matrix is thousands of trials (days of wall-clock) — execution is bounded by available time and MUST be reported as partial where not completed.
- Groq reachable; Gemini times out from this machine (documented in Phase 9/10). "Small + large model" requires both to be reachable.
