# Phase 17 — Evidence-Driven MetaTool Upgrades (report)

Date: 2026-10-25. Evidence dir: `results_live/phase17_20261025T140447Z/`.
Model/provider available: Groq `openai/gpt-oss-120b` (default) and Gemini `gemini-3.8-flash` (both keys present).
Canonical robot-control port: **1235** (`orbit_ros/humanoid.py`), verified.

This report states only what the project's own evidence supports. Every feature carries a status label:
**VERIFIED LIVE** / **CODE-CHECK ONLY** / **FAILED** / **BLOCKED** / **NOT IMPLEMENTED**, with evidence paths.

## 0. Audit of prerequisites (before any change)

The MetaTool was already implemented, tested and previously live-accepted (see `docs/METATOOL_AUDIT_REPORT.md`,
`docs/PHASE14_METATOOL_REPORT.md`). Confirmed present and working as prerequisites (so they were extended, not
rebuilt): plan DAG + schema (`plan.py`, `schema.py`), static validator (`validator.py`), deterministic executor with
per-step verification (`executor.py`, `spec.py`), repair loop + loop guard + call budget (`executor._repair`,
`guard.py`), K-round self-refine and bounded repair, plan cache/clarification hard-triggers at the chat layer.

Note: the prompt referenced `KIRO_PROMPT_ORBIT.md` sections 4/4b/4c. **That file does not exist in this repository.**
The audit was therefore performed against the actual modules and the documented behaviour; prompt sections were mapped
to the real code. This is recorded rather than worked around.

Protected files confirmed untouched this phase: `webots/worlds/orbit_house.wbt`, `orbit_ros/house.py`,
`webots/protos/`, and the live motion/safety path in `orbit_ros/humanoid.py`.

Backups of every edited file: `_work_backup/phase17_20261025_150447/`.

## 1. Per-feature status

### Feature 2 — Refusal/action consistency gate — CODE-CHECK ONLY (live BLOCKED)
- **Purpose / switch:** reject any plan that both refuses and requests motion, so a rejected plan yields zero motion.
  Switch: `validator.validate(..., check_refusal=True)` (default on; executor never turns it off).
- **Implementation:** new deterministic validator issue `REFUSAL_MISMATCH` in `orbit_ros/metatool/validator.py`.
  Fires when (a) `feasible==false` with actionable steps, (b) `feasible==true` with a non-empty `refusal_reason` and
  actionable steps, or (c) explicit refusal language in `goal`/`done_when`/step `why` while actionable steps exist.
  "Actionable" = any tool not in a read-only allow-list (`READ_ONLY_TOOLS`); unknown tools are actionable by default
  (fail-safe). Absence of refusal language never grants permission to act.
- **Evidence:** `tests/test_phase17_metatool.py::TestRefusalMismatch` (6 cases) + `TestZeroMotionInvariant`
  (exhaustive over every actionable tool) — all pass (`results_live/.../phase17_tests.txt`, 44 passed).
- **Live:** the "zero motion for a rejected plan" target must still be measured on the live NAO; BLOCKED (Webots down).
  Existing harness: `tools/live/m10_metatool_live.py walk_through_wall` (expects status=refused, <0.1 m moved).

### Feature 1 — Two-pass planning + completeness (INCOMPLETE) — CODE-CHECK ONLY (live BLOCKED)
- **Purpose / switch:** flag goal targets/constraints not represented in the plan and drive a bounded refinement
  round. Switches: `Config.check_complete` (default False, conservative) and `Config.planning_mode`
  (`single_constrained` default | `two_pass` | `single_free`).
- **Implementation:** `validator._completeness` adds `INCOMPLETE` for any `goal_struct` target that is a KNOWN name
  yet appears in no step arg and no constraint (conservative, to avoid false positives). Wired into
  `executor._plan_once`'s existing refine loop via `check_complete`, so refinement stays bounded by `max_refine` and
  the existing loop guards. `planning_mode` is exposed on `Config`; `single_constrained` remains the verified default.
- **Evidence:** `tests/test_phase17_metatool.py::TestCompleteness` (missing-target flagged only when enabled;
  all-addressed passes; unknown target not flagged).
- **Limitation:** `wrong_valid_schema_rate` (schema-valid + validator-approved plans that later fail live goal
  verification) requires live runs with the real provider to measure; BLOCKED this session. Reporting fields defined
  (numerator, denominator, model, planning mode, verification criterion) but not yet populated with live data.

### Feature 3 — Deterministic safety gate + Task Safety Contract — CODE-CHECK ONLY (live BLOCKED)
- **Purpose / switch:** final AUTHORIZE/DEFER/REJECT decided by code rules, not the LLM. Switch: `Config.use_safety_gate`
  (default off to preserve existing behaviour).
- **Implementation:** new `orbit_ros/metatool/safety_gate.py`. `evaluate(...)` -> `Decision`: harmful intent -> REJECT;
  unsupported capability -> REJECT; a person implicated without reliable person-distance sensing -> DEFER; unresolved
  target -> DEFER; otherwise AUTHORIZE with a compiled `Contract` (invariants, step guards, abort conditions).
  Person-clearance 1.0 m preferred / 0.6 m critical are included in the contract **only** when `person_sensing=True`.
  This project's live NAO has no person-distance sensing wired into the metatool, so the executor passes
  `person_sensing=False` and the gate DEFERs when a person is implicated (never assumes the area is clear). Wired into
  `executor.run`: DEFER -> say the question, status `deferred`, zero motion; REJECT -> refuse, zero motion. The
  HumanoidGuard remains the hard safety authority; the LLM can never override an abort.
- **Evidence:** `tests/test_phase17_metatool.py::TestSafetyGate` (harmful REJECT, unsupported REJECT, person DEFER,
  unresolved DEFER, safe AUTHORIZE with contract, person-distance invariant only with sensing).
- **Live:** simulated person-path intrusion and live authorize/defer/reject need Webots + (for person tests) a sensing
  setup that does not currently exist; BLOCKED. No person detection is claimed.

### Feature 8 — Tool retrieval — CODE-CHECK ONLY (live BLOCKED)
- **Purpose / switch:** show the planner a relevant subset of tools; the validator keeps the full catalogue. Switches:
  `Config.use_tool_retrieval` (default off), `Config.retrieval_k` (default 12).
- **Implementation:** new `orbit_ros/metatool/tool_retrieval.py`: deterministic keyword+category scoring;
  `essential_tools` returns only tools that ACTUALLY EXIST — here `stop_robot` and `stand_up`. The prompt also named
  `ask_user`, `get_status`, `say`; these are **not** robot tools in this project, so they are skipped, not invented.
  `retrieve` always includes existing essentials, then top-scored tools, capped at k. The executor plans against the
  subset but validates against the full registry; on a `retrieval_miss` it retries once with the full catalogue
  (recorded as a miss).
- **Evidence:** `tests/test_phase17_metatool.py::TestToolRetrieval` (essential-only existence, essentials always
  included + k respected, subset ⊆ full catalogue, miss detected).
- **Limitation:** tool-selection accuracy, retrieval-miss rate, prompt-token and latency comparisons vs full-catalogue
  planning need live LLM runs; BLOCKED.

### Feature 11 — MCP tool-manifest integrity + prompt-injection hardening — CODE-CHECK ONLY (fail-closed verified)
- **Purpose / switch:** detect any change to the registered tool surface and treat untrusted text as data. Switch:
  env `ORBIT_TOOL_MANIFEST_CHECK=1` (default off, preserving current startup); when on, a manifest mismatch blocks
  robot tools and the metatool executing paths (fail-closed).
- **Implementation:** new `orbit_ros/metatool/tool_manifest.py` builds a deterministic canonical representation of all
  tools (names, categories, arg schemas, prompts) and SHA-256 hashes it; `configs/tool_manifest.lock` holds the
  expected hash (33 tools, `865ac510…a4f5`). `verify()` is fail-closed (unknown/absent lock -> blocked). A deliberate
  update is a documented, on-purpose `write_lock(...)`. Injection hardening in `prompts.py`: untrusted memory text is
  fenced in a `<<<DATA … DATA>>>` block with escape markers neutralised, and `SYSTEM_RULES` states that names/memory/
  DATA are untrusted data, never instructions. The executor is fail-closed: it only invokes a registered tool from an
  approved plan.
- **Evidence:** `tests/test_phase17_metatool.py::TestManifestIntegrity` (lock matches live; a shrunk catalogue does not
  verify; hash deterministic) and `::TestInjectionHardening` (untrusted memory fenced + escape neutralised; an
  instruction-like object name fails ARG_ENUM and stays a label; system rules declare untrusted data).
  `results_live/.../manifest_verify.json` = ok:true, 33 tools.
- **Preserved:** MCP stdio transport and session architecture unchanged; the check is additive and off by default.

### Feature 7 — Root-cause repair — CODE-CHECK ONLY (live BLOCKED)
- **Purpose:** classify a failure into a fixed taxonomy and restart repair from the EARLIEST contributing step, never
  repeating a completed physical action.
- **Implementation:** new `orbit_ros/metatool/root_cause.py` (taxonomy planning/action/perception/memory/execution/
  system). `attribute(steps)` finds the earliest failed/errored/vetoed step; wired into `executor._repair` (logs a
  `root_cause` trace event). The existing splice logic already freezes completed steps and gives new ids to new steps.
- **Evidence:** `tests/test_phase17_metatool.py::TestRootCause` (earliest step attributed; connection->system;
  unknown when no evidence).
- **Limitation:** the stale-memory scenario and a live root-cause distribution need an approved live reproduction;
  BLOCKED. Labels are only asserted when trace evidence supports them (`supported` flag).

### Feature 5 — Consistency voting — CODE-CHECK ONLY (live BLOCKED)
- **Purpose / switch:** vote over normalized goals; carry forward the winning group's plan (still subject to full
  validation/safety/verification). Switch: `Config.vote_k` (default 1 = off).
- **Implementation:** new `orbit_ros/metatool/voting.py` (`normalize_key`, `vote`, `agreement`). `executor._plan_voted`
  generates K candidates and selects the winner; no unsupported self-critique loop.
- **Evidence:** `tests/test_phase17_metatool.py::TestVoting` (majority goal wins; normalize_key order-invariant).
- **Limitation:** success/latency/token/cost comparisons and reasoning-effort (low/medium/high) comparisons need live
  runs; BLOCKED. Where provider usage/cost is unavailable it is marked unavailable, not estimated.

### Feature 9 — Optional plan judge — CODE-CHECK ONLY (disabled by default)
- **Purpose / switch:** advisory review for missing/redundant/contradictory steps; advisory only (cannot override
  validation/safety/contract/execution). Disabled by default; `MAX_ROUNDS=3`.
- **Implementation:** new `orbit_ros/metatool/judge.py` (deterministic structural judge; not wired to block anything).
- **Evidence:** `tests/test_phase17_metatool.py::TestJudge` (flags all three kinds; clean plan has no notes).
- **Limitation:** enabled-vs-disabled latency/token/success deltas need live runs; BLOCKED.

### Feature 10 — Memory upgrades (consolidated view) — CODE-CHECK ONLY (live BLOCKED)
- **Purpose:** give the planner a relevant scene subgraph + consolidated facts, not raw event logs. Memory stays
  advisory; live observations and the guard always win.
- **Implementation:** new `orbit_ros/metatool/memory_view.py` — `scene_subgraph` (deterministic house→room→items from
  `house.py`), bounded `short_term` (recent observations de-duplicated by label), `consolidated` (scene + short-term +
  taught places + one-line brief), `memory_hit`. READ-ONLY. **No schema migration was needed**: `nao_memory.py` is
  already versioned (v2: events/episodes/strategies/meta) with an in-place migration, so the risky part is avoided.
- **Evidence:** `tests/test_phase17_metatool.py::TestMemoryView` (hierarchical subgraph; room filter; brief present,
  no raw events). Existing `tests/test_nao_memory.py` still passes (no regression).
- **Limitation:** memory hit-rate / stale-memory incidents / prompt-token deltas need live runs; BLOCKED.

### Feature 6 — In-step runtime monitors — CODE-CHECK ONLY (evaluator); live wiring BLOCKED
- **Purpose:** ≥5 Hz checks for progress/heading-drift/contract/person(only if observed)/fall; stop safely and record.
- **Implementation:** new `orbit_ros/metatool/monitors.py` is the deterministic EVALUATOR (`evaluate` -> continue/stop
  with observed values + thresholds). It is **not** wired into the live Webots motion loop in `humanoid.py`: that path
  is the protected motion/safety code and the behaviour can only be validated live, so the integration is deferred and
  documented rather than changed blindly.
- **Evidence:** `tests/test_phase17_metatool.py::TestMonitors` (fall stops; no-progress stops; progress continues;
  person only enforced when observed; ≥5 Hz interval).
- **Limitation:** early-abort-vs-post-step timing comparison and the push/fall disturbance test need a live, approved
  Webots setup; BLOCKED.

### Feature 4 — Calibrated clarification — CODE-CHECK ONLY; calibration data BLOCKED (needs real labels)
- **Purpose:** empirical candidate frequencies + a prediction-set threshold to decide proceed/choose/ask.
- **Implementation:** new `orbit_ros/metatool/clarify.py` (`candidate_frequencies`, `threshold_from_calibration`
  split-conformal style, `decide`). Existing deterministic hard triggers are preserved: with no calibration dataset
  `load_calibration` returns None and `decide` returns `ask_open` with `calibrated=False`, so the caller falls back to
  the existing behaviour. **No formal guarantee is claimed**; the empirical-coverage assumptions (exchangeability,
  enough samples) are documented with every decision.
- **Evidence:** `tests/test_phase17_metatool.py::TestClarify` (uncalibrated -> ask_open; frequencies sum bounded;
  synthetic calibration exercises the math only).
- **BLOCKED dependency:** a trustworthy threshold needs ≥50 real English instructions with user-confirmed goal labels
  (and ≥50 held-out, ≥30 adversarial, ≥20 unsupported, ≥20 ambiguous for evaluation). Labelling requires a real
  person. This dependency is recorded, not faked; `configs/clarify_calibration.json` is intentionally absent.

## 2. Evaluation and ablations (Section 12)

Code-level ablation switches exist and are regression-tested on/off: refusal gate (`check_refusal`), completeness
(`check_complete`), safety gate (`use_safety_gate`), tool retrieval (`use_tool_retrieval`), voting (`vote_k`), judge
(advisory module), consolidated-vs-raw memory (view module). The **measured** ablation tables (success + 95% Wilson
intervals, paired McNemar on matched instructions, wrong-valid-schema rate, harmful-accept/rejected-motion counts,
clarification accuracy/coverage, early-abort timing, root-cause distribution, tool-selection accuracy/misses, memory
hit-rate, latency/tokens/cost) require live runs with the real provider and NAO and are **BLOCKED** this session. No
numbers are fabricated. Historical Phase 8–16 evidence is unchanged.

## 3. Regression + safety invariants

- `tests/test_phase17_metatool.py` = **44 passed** (`results_live/phase17_20261025T140447Z/phase17_tests.txt`).
- metatool core + runtime + phase17 = **69 passed**.
- Full non-live suite (`pytest -m "not live"`): **one failure**, `tests/test_humanoid.py::TestHouse::
  test_world_file_matches_house`. This is **pre-existing** (documented in `docs/METATOOL_AUDIT_REPORT.md` §6): the
  shipped `orbit_house.wbt` carries an angled presentation Viewpoint (`position 5.000 -3.400 12.500`) while
  `tools/make_house_world.py` generates the top-down `5.000 4.000 30.000`. No world/house/generator file was changed
  in Phase 17, so this is not a Phase 17 regression.
- Safety invariants preserved: nothing bypasses `HumanoidSession.robot_call`; the HumanoidGuard never reads memory or
  LLM output; the validator is pure code; new switches default to the prior behaviour.

## 4. Limitations (summary)

1. **Live evaluation BLOCKED:** Webots was not running and port 1235 was not listening; a full live benchmark takes
   hours and only one extern controller may own the NAO. All features above labelled CODE-CHECK ONLY still need their
   live acceptance (`results_live/phase17_20261025T140447Z/LIVE_STATUS.md` lists exact commands).
2. **Calibration data BLOCKED:** feature 4 needs ≥50 real user-labelled instructions; none exist, so clarification
   falls back to deterministic triggers and claims no formal coverage guarantee.
3. **Person sensing absent:** the live NAO has no person-distance sensing wired into the metatool, so the safety gate
   DEFERs when a person is implicated and no person detection is ever claimed.
4. **In-step monitors not wired live:** the evaluator is code-tested, but integrating it into the protected motion
   loop in `humanoid.py` is deferred until it can be validated live without risking the safety path.
5. **Reference prompt missing:** `KIRO_PROMPT_ORBIT.md` (cited by the task) is not in the repo; the audit used the
   actual modules instead.

## 5. Definition of done (honest status)

Every Phase 17 feature is implemented with a configuration switch, preserves the safety pipeline, and has automated
code-check + safety tests. The live acceptance and the measured ablation/evaluation tables are explicitly documented
as BLOCKED (Webots down; calibration needs real labels), with the exact commands to complete them. No harmful task is
accepted and every refused/deferred plan carries zero motion in the code-checked scenarios; the live measurement of
those targets remains to be run.
