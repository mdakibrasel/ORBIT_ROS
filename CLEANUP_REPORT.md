# CLEANUP REPORT — ORBIT_ROS_v0.16.2_RUN

Mode: **Safe Project Cleanup** (reversible quarantine, no permanent deletion)
Generated: 2026-10-24
Quarantine folder (outside project): `C:\Users\mdaki\Desktop\LLM_Robot_Project\_ORBIT_cleanup_quarantine_20261024_150533`

Nothing is permanently deleted. Confirmed-unnecessary files are **moved** to the quarantine folder above. Uncertain items, backups, and source/config are **not touched** and are listed under "Flagged — needs your approval".

---

## Method

1. Full recursive scan of the project tree.
2. For each candidate: read its contents, checked references across `**/*.{py,ps1,bat,json,toml,md,cfg,txt}`, and checked git tracking status.
3. Cross-checked against the project's own committed `.gitignore` (in `HEAD`) and the diagnostic narrative in `TASKS.md`.
4. Applied the diagnostic-value rule (instruction 4) to every bridge/Webots log before proposing any move.

Reference-check tooling used: `grep_search` over all source/config globs, `git status`, `git diff`, and `pytest --collect-only` (to prove `.bak` files are not collected).

---

## MOVED to quarantine (confirmed unnecessary)

These are root-level ad-hoc scratch files (all prefixed `_`), untracked by git, referenced by **no** source or test, and whose informational content is already captured in `TASKS.md`. They are the throwaway evidence captures and probe scripts from the recent debugging session.

### Temporary probe / helper scripts (ad-hoc, superseded)

| File | Full path | Size | Reason | Reference check |
|---|---|---|---|---|
| `_tmp_cmdline.ps1` | `...\_tmp_cmdline.ps1` | 453 B | One-off PID 9364 command-line probe | No references; hard-coded dead PID |
| `_tmp_findwebots.ps1` | `...\_tmp_findwebots.ps1` | 171 B | One-off wrapper around `tools/find_webots.ps1` (the real, kept tool) | No references; duplicates kept tool |
| `_tmp_live_move.py` | `...\_tmp_live_move.py` | 2766 B | Ad-hoc live turn/walk probe; its verdict is recorded in `_live_move.txt` and TASKS.md §4 | No imports/references from code or tests |
| `_tmp_probe_bridge.py` | `...\_tmp_probe_bridge.py` | 355 B | Ad-hoc `bridge_up()` probe | No references |
| `_tmp_pyps.ps1` | `...\_tmp_pyps.ps1` | 249 B | Ad-hoc python-process/port-1240 lister | No references |
| `_tmp_restart_bridge.ps1` | `...\_tmp_restart_bridge.ps1` | 651 B | Ad-hoc bridge restart; writes the `_bridge_*2.txt` scratch files also quarantined | No references |
| `_tmp_status.ps1` | `...\_tmp_status.ps1` | 1016 B | Ad-hoc port 1235/1240 status check | No references |
| `_tmp_test_lookaround.py` | `...\_tmp_test_lookaround.py` | 1187 B | Ad-hoc check for a `look_around` tool. Note: current committed tree exposes **41** tools without `look_around`; this script targeted a 42-tool variant (see `_selftest_fake2.txt`) that is not the shipped surface | Not collected by pytest; not referenced |

### Captured test-run evidence (one-off redirected output)

| File | Full path | Size | Reason | Reference check |
|---|---|---|---|---|
| `_baseline_count.txt` | `...\_baseline_count.txt` | 648 B | pytest progress dump (490-count, stale) | Only named in TASKS.md prose |
| `_baseline_pytest.txt` | `...\_baseline_pytest.txt` | 567 B | pytest capture; written by `_run_baseline.bat` | Only named in TASKS.md prose |
| `_baseline_pytest2.txt` | `...\_baseline_pytest2.txt` | 611 B | pytest capture (488/2 run) | Only named in TASKS.md prose |
| `_baseline_summary.txt` | `...\_baseline_summary.txt` | 0 B | Empty | None |
| `_reg2.txt` | `...\_reg2.txt` | 648 B | Duplicate pytest progress dump | None |
| `_selftest_fake.txt` | `...\_selftest_fake.txt` | 1817 B | Fake-sim self-test capture (41 tools) | None |
| `_selftest_fake2.txt` | `...\_selftest_fake2.txt` | 2058 B | Fake-sim self-test capture (42-tool variant) | None |
| `_selftest_live.txt` | `...\_selftest_live.txt` | 2131 B | Live Webots self-test capture; verdict in TASKS.md §4 | None |
| `_live_move.txt` | `...\_live_move.txt` | 769 B | Output of `_tmp_live_move.py` | None |
| `_run_baseline.bat` | `...\_run_baseline.bat` | 239 B | Ad-hoc batch to run pytest with redirection | References only the quarantined `_baseline_pytest.txt` |

### Transient bridge-capture copies — diagnostic value ASSESSED (instruction 4)

| File | Full path | Size | Diagnostic assessment | Decision |
|---|---|---|---|---|
| `_bridge_out.txt` | `...\_bridge_out.txt` | 260 B | Contains only "connected; clients use 127.0.0.1:1240" — a **successful** connect, no failure signal | Move (no diagnostic value) |
| `_bridge_out2.txt` | `...\_bridge_out2.txt` | 260 B | Identical successful-connect copy | Move (no diagnostic value) |
| `_bridge_err.txt` | `...\_bridge_err.txt` | 0 B | Empty | Move |
| `_bridge_err2.txt` | `...\_bridge_err2.txt` | 0 B | Empty | Move |

### Python bytecode cache (`__pycache__`) — regenerable, gitignored

The committed `.gitignore` (in `HEAD`) ignores `__pycache__/`. These are pure bytecode caches regenerated on next import/run. Quarantined (not hard-deleted) for reversibility. `.venv/__pycache__` folders were **excluded** and never touched.

| Folder | Files | Size |
|---|---|---|
| `.\__pycache__` | 16 | 301,652 B |
| `.\examples\__pycache__` | 2 | 7,423 B |
| `.\orbit_ros\__pycache__` | 44 | 903,508 B |
| `.\orbit_ros\pel\__pycache__` | 8 | 130,410 B |
| `.\tests\__pycache__` | 77 | 1,352,835 B |
| `.\tools\__pycache__` | 7 | 100,918 B |

---

## PRESERVED — required or has diagnostic value

### Preserved for diagnostic value (instruction 4 — Webots/bridge failure)
- **`bridge_out.log`** — contains `[bridge] Webots step failed (Webots simulation ended); waiting for it to come back...`. This is the actual signal for the recent connection failure. **Kept.**
- **`bridge_err.log`** — empty, but it is the active stderr log paired with `bridge_out.log`. **Kept.**

### Preserved — application / runtime / config / assets
- All source: `orbit_*.py`, `orbit_ros/**`, `tools/**` (incl. `find_webots.ps1`, `nao_benchmark.py`), `examples/**`, `run_*.py`, `analyze_spatial.py`, `compare_runs.py`, `conformance.py`, `check_env.py`, `failure_taxonomy.py`, `preflight_llm.py`.
- LLM/MCP client entry points: `gemini_mcp_autonomous.py`, `gemini_mcp_readonly.py`, `groq_mcp_autonomous.py` (substantial source; untracked but clearly real code — preserved per "never delete source").
- `webots/**` — house world `orbit_house.wbt`, `.orbit_house.wbproj`, controllers, motions, protos. **Untouched.**
- `configs/**`, `pyproject.toml`, `requirements.txt`, `VERSION`, `run.ps1`, `setup.ps1`, `start_webots.ps1`, `run.ps1`.
- `.env`, `.env.example`, `.venv/`, `.vscode/`, `.kiro/`, `.git`.
- `tests/**`, `docs/**`, `templates/**`, `conformance/**`, `README.md`, `CHANGELOG.md`, `START_HERE.md`, `TASKS.md`.
- Results: `results_mcp/`, `results_nao_benchmark/`, `results_spatial_oracle/` — gitignored run outputs; **not** touched (they may be needed to diagnose benchmark/bridge behavior). Large but preserved by default; say the word to prune old runs.

---

## FLAGGED — needs your approval (NOT touched)

Per instructions 3 and 8, I did not move backups, config, or anything uncertain.

1. **`orbit_ros/humanoid.py.bak`** (95,118 B, 2026-10-23) and **`orbit_ros/mcp_server.py.bak`** (25,250 B, 2026-10-23)
   - These are **backups**. They are older than the current working sources (`humanoid.py` 100,672 B / `mcp_server.py` 26,703 B, both 2026-10-24) and differ substantially (≈243 insertions vs the live `humanoid.py`).
   - `pytest --collect-only` confirms they are **not** collected, so they are inert and safe to remove — but they are backups, so I am asking first.
   - **Recommendation:** quarantine them too (reversible). Awaiting your approval.

2. **`.gitignore` is deleted from the working tree** (present in `HEAD`, missing on disk).
   - This is why `__pycache__/`, `results_mcp/`, `results_nao_benchmark/` now show as untracked in `git status`.
   - This is a config/source change, not cleanup — I did not recreate or remove anything.
   - **Recommendation:** restore it with `git checkout -- ORBIT_ROS_v0.16.2_RUN/.gitignore` so caches/results stay ignored. Awaiting your approval.

---

## Verification performed after the move
See `TASKS.md` → "Cleanup (2026-10-24)" section for the import check, test run, and house-world integrity check with their actual outputs.

---

## Cleanup verification (2026-10-24) — executed, not claimed

After moving the files above to quarantine, the project was verified intact:

- **Imports:** `import orbit_ros.humanoid, orbit_ros.mcp_server, orbit_mcp_server, orbit_bridge, orbit_ros.remote, orbit_chat` → `IMPORTS_OK`.
- **Full unit suite:** `.venv\Scripts\python.exe -m pytest -p no:cacheprovider` → **506 passed, 2 skipped, 0 failed** (the 2 skips are
  environment/data dependent and unrelated to cleanup).
- **House-world integrity:** `tests/test_humanoid.py::TestHouse::test_world_file_matches_house` passed; the shipped
  `webots/worlds/orbit_house.wbt` still matches `orbit_ros/house.py` and was not modified.
- **Live Webots self-test + movement:** PASSED on the direct-1235 transport (see `TASKS.md` for measured before/after pose).

## Second-pass additions (v0.16.5 engineering)

- Additional quarantined items (evidence captures from the live run, kept rather than deleted):
  `_live_selftest_1235.txt`, `_live_verify_1235.txt`, `_live_verify_1235.py`, `_gen_matrix.py`.
- Timestamped backups of every source/config/test/doc file edited in the engineering pass are under `_backups_20261024_151132\`.
- `.gitignore` was **restored** from `HEAD` (it had been deleted from disk); this re-enables ignoring of `__pycache__/`, `results_mcp/`,
  `results_nao_benchmark/` and protects `.env`.

## Still FLAGGED — needs your approval (unchanged, NOT touched)
- `orbit_ros/humanoid.py.bak`, `orbit_ros/mcp_server.py.bak` — stale backups, inert (not collected by pytest). Left in place pending approval.

## Phase 8 additions (2026-10-24)

- Quarantined the Phase 8 diagnostic scratch scripts and their captures (kept as evidence, not deleted):
  `_phase8_scenario_A.py`, `_phase8_reconnect.py`, `_phase8_fault.py`, `_phase8_regression.py`, `_phase8_benchmark.py`, and the
  `_phase8_*.txt` output captures.
- Re-quarantined regenerated `__pycache__` dirs (6) after the test/benchmark runs. `.venv` never touched.
- Kept in the project (real results / backups, not cleanup candidates):
  `results_nao_benchmark/reliability_20261024T145555Z/reliability.json` (live benchmark output; under the gitignored results dir),
  and `_backups_20261024_163632\` (backups of the two files edited in Phase 8).
- No source, config, test, doc, or house-world file was deleted. The two `.bak` files remain flagged for approval (unchanged).

## Phase 9 additions (2026-10-24)

- New durable project files (kept, not cleanup candidates): `tools/phase9_experiment.py` (live LLM/open-loop/planner runner),
  `tools/phase9_consolidate.py`, `tools/phase9_llm_check.py`, `tools/phase9_conn_check.py`, `docs/PHASE9_RESEARCH_REPORT.md`,
  and `tests/test_humanoid.py::TestMalformedToolArgs` (regression).
- Live research data kept under the gitignored results dir: `results_nao_benchmark/phase9_live_*/` (101 raw per-trial traces) and
  `results_nao_benchmark/phase9_consolidated_20261024T190219Z/` (trials.csv/jsonl, summary.json, comparison.md).
- Quarantined the Phase 9/8 throwaway probe scripts and logs (`_p9_*`, `_phase8_*`, `_probe_providers.py`, `_p9_*.log/.err`) and the
  regenerated `__pycache__` dirs. Durable copies of the two pre-flight probes live under `tools/` (phase9_llm_check / phase9_conn_check).
- Protected files untouched and verified byte-identical to backups: `orbit_ros/humanoid.py`, `orbit_ros/house.py`,
  `webots/worlds/orbit_house.wbt`, `webots/protos/`. Backups of files edited this phase: `_backups_20261024_163632\`, `_backups_20261024_172922\`.
- The two `.bak` files remain flagged for your approval (unchanged).

## Phase 10 additions (2026-10-24)

- New durable files (kept): extended `orbit_ros/nao_memory.py` (episodic+semantic memory), `tools/phase10_registry.py`,
  `tools/phase10_experiment.py`, `tools/phase10_exp_bdef.py`, `tools/phase10_exp_f.py`, `docs/PHASE10_MEMORY_REPORT.md`,
  `docs/MEMORY_MODULE.md`, and new tests in `tests/test_nao_memory.py` (`TestEpisodicSemantic`).
- New live/measured data (own dir): `results_nao_benchmark/phase10_memory_20261024T224840Z/` (trace JSONs, trials.csv/jsonl,
  exp{B,D,E}_summary.json, expF_metrics.json, experiment_config.json, historical_registry.json, and the scoped memory dbs — scanned, no secrets).
- Historical Phase 8/9 files: hashed in `phase10_memory_20261024T223310Z/historical_registry.json` and re-verified unchanged
  (`HISTORICAL INTEGRITY: OK (7 files)`). Nothing from Phase 8/9 was recomputed or overwritten.
- Quarantined the Phase 10 throwaway probes/logs (`_p10_*`) and regenerated `__pycache__`; removed the throwaway `results_mcp/_p10_probe.db`.
- Protected files untouched: `webots/worlds/orbit_house.wbt`, `orbit_ros/house.py`, `webots/protos/`. Backups of edited files: `_backups_20261025_003321\`.
- The two `.bak` files remain flagged for your approval (unchanged).
