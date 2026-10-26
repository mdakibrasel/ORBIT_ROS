# MetaTool Audit Report (2026-10-25)

Evidence-based audit of `orbit_ros/metatool/` + its MCP integration, with one low-risk fix implemented and
verified live. No fabricated results; every claim below cites a command/file I ran or read.

## 1. Current implementation status
The MetaTool is **implemented, well-tested, and previously live-verified** — not missing and not a stub. Modules
(`orbit_ros/metatool/`): `plan.py`, `spec.py`, `schema.py`, `validator.py`, `llm_io.py`, `prompts.py`, `guard.py`,
`executor.py`, `trace.py`, `__init__.py`. MCP tools `validate_plan`, `execute_plan`, `plan_and_execute` are registered
in `orbit_ros/mcp_server.py` (`_metatool`). Prior live acceptance is documented in `docs/PHASE14_METATOOL_REPORT.md`
with evidence packs under `results_live/phase14_metatool_20261025/` and `results_live/m10_20261025/` (I confirmed the
packs exist and read their real `verdict.json`).

## 2. Working / partially working / failing
**Working (evidence):**
- Tool discovery + registration: `schema.plan_json_schema(HUMANOID_TOOLS)` builds one anyOf variant per tool; test
  `test_variants_equal_tool_count` passes. Registry = 33 tools (checked live).
- Parameter validation: `validator.validate` detects UNKNOWN_TOOL, ARG_MISSING/UNKNOWN/RANGE/ENUM/TYPE, BAD_DEP, CYCLE,
  DUP_ID, BAD_ONFAIL, BAD_REF, TOO_LONG, REPEAT, PRECONDITION (forward-sim). All covered by `test_metatool_core.py`.
- Execution + safety: `executor.MetaToolAgent` runs steps in topological order through the unchanged
  `HumanoidSession.robot_call` (arg-check → preconditions → `HumanoidGuard` → motion); stop-latch and call budget
  intact. Loop guard (`guard.py`) blocks the 3rd identical no-progress call and A-B-A-B oscillation.
- Error handling / reliability: `llm_io.py` falls back json_schema → json_object → text, remembers rejected modes,
  retries 403/429/5xx with backoff, always re-validates locally.
- Integration with NAO: live MCP self-test lists `validate_plan`/`execute_plan`/`plan_and_execute` among the tools.
- **Live, this session:** `plan_and_execute("go to the kitchen")` → status `done`, NAO true pose ended in **kitchen**
  (4.098, 5.668), single verified `go_to_room` step. Evidence: `results_live/metatool_verify_20261025T133534Z/metatool_verify.json`.

**Code-check tests:** `tests/test_metatool_core.py` + `tests/test_metatool_runtime.py` → **25 passed** (was 24; +1 new
regression guard). Full non-live suite → **577 passed, 2 skipped, 1 deselected**.

**Partially working / honest gaps:**
- Repair-splice path is exercised by code checks; a *live* repair was not triggered this session (the known-target plan
  succeeded first try). Documented limitation, consistent with Phase 14.
- Advanced features (plan_status/cancel/resume/clarify/skills/…) are intentionally NOT implemented — listed in
  `docs/METATOOL_SYSTEM.md` and `docs/METATOOL_ADVANCED_PLAN.md`.

**Failing (found + FIXED this audit):**
- The system prompt (`prompts.py`) and validator hint (`validator.py` `_pre_hint`) told the model to establish an unknown
  target with `remember_place` / `find_object`. **Neither tool exists** in the 33-tool registry (verified), and there is
  **no place-teaching mechanism** (`nao_memory.py` has no `places` table). For any "go to X" where X is not a known room
  or house item, this risked an UNKNOWN_TOOL validation loop and wasted refine rounds.

## 3. Fix implemented (low-risk; does NOT touch the NAO world, guard, or execution path)
- `prompts.py`: the guidance now says a target must be a known room or house item; otherwise reply `feasible:false` with a
  reason — "do NOT invent a discovery tool."
- `validator.py` `_pre_hint`: the `known(...)` hint no longer names non-existent tools.
- `tests/test_metatool_runtime.py`: added `test_system_prompt_names_no_phantom_tools` (regression guard).
- Backups of the three edited files: `_work_backup/metatool_promptfix_20261025_135330/`.

**Verification (genuine):**
- `test_metatool_core.py` + `test_metatool_runtime.py` → **25 passed**; prompt no longer contains `remember_place`/`find_object`.
- **Live:** `plan_and_execute("go to the spaceship launchpad")` → status `refused`, `feasible:false`,
  reason "Target 'spaceship launchpad' is not a known room or known object in the house.", **0.0 m moved**, 0 steps.
  `plan_and_execute("go to the kitchen")` → `done`, true pose in kitchen. Evidence file cited in §2.

## 4. Recommended upgrades, prioritized (not implemented — would need their own live acceptance)
1. (Med) A real `remember_place`/`go_to_place` capability + `places` table, if teaching new locations is desired — this
   would make the removed prompt guidance valid again. Higher risk (touches memory + tools); out of scope here.
2. (Low) `explain_last_run` / `get_metrics` read-only MCP tools surfacing the existing `trace.metrics()` — low risk, useful
   for demos.
3. (Low) Public `MetaToolAgent` export from `metatool/__init__.py` so callers avoid deep imports.
None were implemented to honor "implement only what is genuinely needed; do not add unnecessary features."

## 5. Incident disclosure (important, fully recovered)
While applying the prompt fix I first used an AST structural-rewrite tool with a plain-string pattern; it **misfired** and
matched the `\n` escape inside string literals, corrupting 20 active files + 6 backup copies (replacing newline escapes with
the new prompt text). The NAO world (`orbit_house.wbt`), the metatool core modules, active `humanoid.py`/`mcp_server.py`, and
controllers were **not** affected. Recovery:
- 13 git-tracked files restored with `git checkout`.
- 7 untracked files + 6 backup copies reversed with a validated regex collapse back to `\n` (each re-checked with
  `py_compile`).
- Verified: **0 residual** injected text (search), all formerly-corrupted files compile, non-live suite **577 passed**,
  `orbit_house.wbt` sha256 `2d6dfcb6…0cdc` **unchanged** throughout. The fix was then re-applied with a surgical,
  single-file `strReplace`. Lesson: never use a plain-string AST pattern that can match escape sequences broadly.

## 6. Remaining issues / unverified
- **Pre-existing test failure (not caused by this work):** `tests/test_humanoid.py::TestHouse::test_world_file_matches_house`
  fails because the shipped `orbit_house.wbt` (and git HEAD) carries an **angled presentation Viewpoint**
  (`orientation -0.4475 0.4475 0.7742 1.8239 / position 5.000 -3.400 12.500`) while `tools/make_house_world.py` still
  generates the old top-down (`5 4 30`). The world hash was unchanged across this session, so this discrepancy predates my
  work. Fixing it means editing either the shipped world's viewpoint or the generator — both touch NAO presentation, so I
  did not change them without direction. Recommend aligning `make_house_world.py`'s viewpoint with the shipped angled view
  (or vice-versa) in a dedicated change.
- Live repair-splice path remains covered only by code checks (see §2).
- Gemini provider not exercised live (Groq used, as in prior phases).
