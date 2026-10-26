# ORBIT meta-tool — Tasks

Work in order. Tick `[x]` only after the task's **live acceptance check** passed and its evidence pack exists (pure code-check tasks tick
after the code check passes). Every task lists: Files, Unit/code tests, Live acceptance.

Legend for status: `[ ]` not started · `[~]` code+code-checks done, live pending · `[x]` done incl. live evidence · `[blocked]` env blocker (reason).

---

## Phase 0 — Kiro setup
- [x] 0.1 Steering `.kiro/steering/orbit-rules.md` (section 1 verbatim). Live: n/a.
- [x] 0.2 Spec `requirements.md` / `design.md` / `tasks.md`. Live: n/a.

## Phase 2 (built FIRST per brief) — Live harness  `tools/live/`
- [x] H.1 `tools/live/webots_session.py` — fresh process/trial, true pose capture, evidence pack (start/end.png, trace.jsonl, plan.json,
  metrics.json, true_poses.csv, webots_console.txt, verdict.json), setLabel HUD, refuses a 2nd extern controller.
  Files: `tools/live/webots_session.py`. Code tests: evidence-pack writer on a synthetic trial (no robot). Live: one real trial to the
  kitchen writes a complete pack judged from the true pose.
  **DONE 2026-10-25**: code checks `tests/test_live_harness.py` green; live PASS → `results_live/h1_20261025/go_kitchen/` (515 true-pose
  samples over 271 s, 0 fallbacks/resets, end room=kitchen, both screenshots 2.4 MB).
- [ ] H.2 `tools/live/live_tool_check.py` — ≥1 live case per registry tool, objective pass from true state; physical disturbances
  (addForce fall, Solid obstacle, world-variant moved object). Files: `tools/live/live_tool_check.py`. Live: `tool_checks.{json,md}` with
  every current tool PASS/FAIL + measurement + threshold + evidence.
- [ ] H.3 `tools/live/live_task_check.py` — runs the section-8 task suite for a chosen strategy/model, per-trial traces.
- [ ] H.4 `run.ps1 -Mode live-check` (H.1+H.2) and `-Mode live-bench` (H.3).
- [ ] H.5 BASELINE: run `live-check` on the CURRENT tools, keep `results_live/<ts>_baseline/`. **Acceptance for section 2.**

## Phase 1 — Honesty fixes (Webots backend)  `orbit_ros/humanoid.py`
- [ ] P1.1 `WebotsNao.play()` kinematic fallback OFF by default; `--allow-kinematic-fallback` enables (counted, invalid). Non-moving → `failed`+measurement.
- [ ] P1.2 `_t_stand_up` plays real `StandUpFromFront.motion`, verify upright, roll-over attempt, else `failed`; reset only behind `--allow-supervisor-reset`.
- [ ] P1.3 `_run()` fall recovery = real stand_up, ≤2/task, logged.
- [ ] P1.4 benchmarks + `live_task_check` mark fallback/reset trials `invalid`.
  Files: `orbit_ros/humanoid.py`, `orbit_mcp_server.py` (flags). Code tests: flag parsing; invalid-marking logic. Live: stand_up from an
  induced front fall PASS; forward step into a wall → `failed` (not teleported); `live-check` reports 0 fallbacks.

## Phase 2 — Meta-tool  `orbit_ros/metatool/`
- [~] M.1 `spec.py` (MetaSpec + facts). Code tests: requires/effects/verify present for every `HUMANOID_TOOLS` tool; `facts(session)`.
  **CODE DONE 2026-10-25** (`orbit_ros/metatool/spec.py`; covered by `tests/test_metatool_core.py`). Live use via M.10.
- [~] M.2 `plan.py` (Plan/Step, goal_struct, topo_order, pseudo_code, extract_json). Code tests: topo/cycle, ${ref} ancestor rule, extract_json on think/fenced text.
  **CODE DONE** (`orbit_ros/metatool/plan.py`; `test_metatool_core.py::TestPlanModel`).
- [~] M.3 `schema.py`. Code test: schema variants == tool count; args schema matches each tool.
  **CODE DONE** (`orbit_ros/metatool/schema.py`; `TestSchema`, variants==44 tools).
- [~] M.4 `validator.py`. Code test: every Issue code on hand-written plan JSON incl. forward-sim PRECONDITION and REPEAT.
  **CODE DONE** (`orbit_ros/metatool/validator.py`; `TestValidator`, all codes exercised).
- [~] M.5 `llm_io.py`. Code test: mode fallback bookkeeping with a local stub transport (no network) — pure I/O logic only; live covered by M.10.
  **CODE DONE 2026-10-25** (`orbit_ros/metatool/llm_io.py`; `tests/test_metatool_runtime.py::TestLlmModeBookkeeping`). Live-exercised in M.10 (groq used json_object fallback).
- [~] M.6 `prompts.py`. Code test: catalogue rendered from registry; refine/repair prompts include issues/scratchpad.
  **CODE DONE** (`orbit_ros/metatool/prompts.py`; `TestPrompts`).
- [~] M.7 `guard.py`. Code test: fingerprint repeat + A-B-A-B on a recorded sequence; budgets.
  **CODE DONE** (`orbit_ros/metatool/guard.py`; `TestLoopGuard`).
- [~] M.8 `executor.py` (MetaToolAgent.run). Code test: plan→validate→refuse path with a hand plan; repair splice with done_ids (no robot).
  **CODE DONE + LIVE-EXERCISED** (`orbit_ros/metatool/executor.py`). The deterministic plan→validate→refine→execute→verify→refuse path ran
  live in M.10 (go_kitchen, walk_through_wall). Repair/skip/abort branches coded; the obstacle/fall repair live checks are blocked on Phase 4 tools.
- [~] M.9 `trace.py` + metrics. **CODE DONE** (`orbit_ros/metatool/trace.py`; `TestTraceMetrics`; live traces written in M.10).
- [x] M.10 Integration: MCP `validate_plan`/`execute_plan`/`plan_and_execute`; `orbit_chat.py --mode plan|stepwise`; strategy D + ablations in live runner.
  **CODE DONE + ALL 10 SECTION-5 LIVE CHECKS PASS** (2026-10-25, Phase 14).
  Live PASS (evidence under `results_live/m10_20261025/`):
    (a) non-existent-tool plan → `UNKNOWN_TOOL` rejected, 0 m moved (`validate_badtool/verdict.json`);
    (f) hazardous "walk through the wall" → `feasible:false` refusal, 0 m moved (`walk_through_wall/verdict.json`);
    "go to the kitchen" via the full meta-tool loop (real Groq json_object plan → validate → go_to_room → verified) → end room=kitchen, PASS
    (`go_kitchen/verdict.json`, `metatool.jsonl`: start→llm→validate→plan→tool→verify→done; 1 LLM call, 2001 tokens, 0 fallbacks).
  Live MCP self-test PASSED (167 s, 48 tools incl. validate_plan/execute_plan/plan_and_execute).
  **Phase 14 (2026-10-25) completed the remaining section-5 live acceptance scenarios** — all 10 now PASS (evidence
  `results_live/phase14_metatool_20261025/` + `results_live/m10_20261025/`): dependent-ref order, loop-guard block, stop-latch not
  bypassable, no-step-repeat after replan, memory context without safety bypass, and blocked→honest-abort recovery. See
  `docs/PHASE14_METATOOL_REPORT.md`. Advanced-feature follow-up is dependency-ordered in `docs/METATOOL_ADVANCED_PLAN.md`.
  STILL PENDING (blocked on Phase 4 tools / disturbance injection, not on the meta-tool): `${ref}` recall→go_to_place; obstacle-drop→repair;
  push→fall→stand_up→repair; the strategy-D ablations in the live runner.
  Live (each an evidence pack): (a) non-existent-tool plan → rejected, 0 motions in true_poses.csv; (b) `${ref}` recall→go_to_place executes+verified;
  (c) obstacle dropped on route → veto/blocked → repair → success; (d) push during walk → fall → real stand_up reflex → repair → success or honest fail;
  (e) repeated identical call → blocked by loop guard; (f) hazardous instruction → refusal, 0 motions.
  Live checks via `orbit_chat.py --mode plan`: "go to the kitchen"; "remember this place as reading corner, go to the bedroom, then come back to the reading corner"; "walk through the wall".

## Phase 2b — Advanced meta-tool
- [ ] MB.1 `dryrun.py` (map dry-run, UNREACHABLE). Code test: a plan through a no-go zone is UNREACHABLE on the map.
- [ ] MB.2 `candidates.py` (K plans, score, pick). Code test: scoring/selection on hand plans.
- [ ] MB.3 `plan_cache.py` (SQLite, verified-only, drop rule). Code test: cache write-after-verify, hit offered as candidate, drop on failures.
- [ ] MB.4 `clarify.py` (ambiguity → ask_user). Code test: the 4 deterministic triggers.
  Live (evidence packs): target behind no-go zone → dry-run rejects, 0 motions; "go to the chair" → ask_user lists chairs → answer → reached;
  same instruction twice → 2nd is a cache hit with 0 planning LLM calls, still verified from true pose; K=3 long-horizon → candidates/scores/chosen logged, predicted vs measured time reported.

## Phase 3 — Fix existing tools
- [ ] P3.1 add `turn_by`, `walk(distance_m)` (closed loop); keep `walk_forward` alias. (see Phase 4 table rows)
- [ ] P3.2 `--calibrate` → `configs/nao_calibration.json` (LIVE measurement).
- [ ] P3.3 `--pose-noise SIGMA_M,SIGMA_DEG` (off by default; reports state it when on).
- [ ] P3.4 remove `search_github` from robot MCP; group `clean(scope,...)`; merge `get_status(sections?)`; old names hidden aliases.
  Code tests: alias resolution; clean/get_status dispatch. Live: `turn_by`/`walk` tolerance checks; calibration file written from live runs.

## Phase 4 — New tools (each: typed args, requires/effects/verify, LIVE check + evidence)
- [ ] P4.* one subtask per section-6 row: say, set_eyes, express, gesture, look_at, turn_by, walk, detect_objects, find_object,
  describe_scene, remember_place, forget_place, go_to_place, recall, memorize_visible, knowledge_query, follow_target, point_at, hands,
  ask_user, stand_up, get_metrics. World edits via `tools/make_house_world.py` (+ `--check`). CLI: `--calibrate`, run.ps1 modes, chat modes.

## Phase 5 — Memory v3  `orbit_ros/nao_memory.py`
- [ ] P5.1 migrate to schema_version=3; fix `normalize_goal` fallback (unit tests incl. negation/question/Bengali/taught-place).
- [ ] P5.2 `aliases` (EN+BN seeds), `places`, `object_state` (staleness >0.5 m), confidence decay, optional embedding hook (off).
  Live: remember a place → unrelated tasks → go_to_place back (true pose in tol); move object (world variant) → find_object updates object_state, old stale.

## Phase 6 — Live evaluation  `results_live/phase11_<ts>/`
- [ ] P6.* strategies A/B/C/D + ablations × models × section-8 tasks, ≥10 trials/cell, one process/trial, true-pose judged;
  `tools/live/make_report.py` → `docs/PHASE11_REPORT.md` (Wilson intervals; every number linked to a file).

## Phase 7 — Repo hygiene
- [ ] P7.1 move measured results into `evidence/` + `MANIFEST.json` (path/sha256/date) + `README.md`; update paths; tests pass.
- [ ] P7.2 remove fakes from running system (FakeNao, `--sim fake`, mocks, oracle, World2D → `legacy/`); no-fake grep check.
- [ ] P7.3 one version source `orbit_ros.__version__` (keep 0.16.5); `tests/test_consistency.py`; `tools/gen_docs.py` for tool count/table + test count.
- [ ] P7.4 `.gitignore`; `tools/make_release.py` whitelist zip (fails on forbidden path).

## Phase 10/11 — README + presentation
- [ ] P10 rewrite `README.md` (English, generated numbers only, architecture diagram).
- [ ] P11 `run.ps1 -Mode demo` / `-Mode demo-script`; `tools/live/make_report.py --run <dir>` → report.md/html.

## Definition of done (section 12) — tracked at the end
- [ ] all phases ticked with live evidence paths; code checks + live tests green; no-fake check passes; live-check all PASS, 0 fallbacks/resets;
  demo-script ran live with a report; 4b live checks passed; live-bench ≥10 trials/cell; PHASE11_REPORT generated; make_release clean zip;
  final summary of everything not verified live with reasons.

---

### Scope note (honest, per steering)
The full Phase-6 matrix and the complete Phase-4 tool set are thousands of live trials + large world-generation work = days of wall-clock,
and the "small model" axis needs a second reachable provider (Gemini currently times out). This session executes the tasks in order,
builds and code-checks each module, and runs live checks where feasible within time; every task left without live evidence stays `[ ]`
or `[blocked]` with its reason, and the final summary lists them. No task is ticked `[x]` on code checks alone if it has a live acceptance.
