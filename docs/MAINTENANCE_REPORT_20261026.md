# Maintenance report — 2026-10-26 (v0.16.5)

Scope: fix the Webots–NAO persistent connection (`--keep-alive`), update documentation to match the actual code, and perform an
evidence-based backup/storage audit. No application behaviour was changed beyond the connection-lifecycle fix; the safety guard,
navigation, cleaning, memory and obstacle handling are untouched.

## 1. Code fixes (verified)

| Change | File | Evidence |
|---|---|---|
| `--keep-alive` rewritten as `_keep_webots_alive(s)`: builds the Webots backend explicitly (no `hasattr` swallow), steps the world, exits cleanly on Ctrl+C / Webots-ended, retries once per second on transient errors (no busy loop) | `orbit_mcp_server.py` | `get_diagnostics` clean; tests below pass |
| `--print-config` injects `--keep-alive` for NAO+Webots | `orbit_mcp_server.py` | `--print-config` output includes `--keep-alive` (confirmed) |
| `--keep-alive` added to the VS Code MCP config | `.vscode/mcp.json` | `test_config_files` passes |
| Claude example config already carried `--keep-alive` | `configs/claude_desktop_config.example.json` | `test_config_files` passes |
| `run.ps1 -Mode hold` added (persistent keep-alive server until Ctrl+C); `-Mode server` message clarified | `run.ps1` | reviewed; `ValidateSet` includes `hold` |
| Tests updated to expect `--keep-alive` in client configs | `tests/test_mcp_server.py` | `test_print_config`, `test_config_files` pass |

Root cause addressed: Webots drops an `<extern>` controller the instant its Python process exits. The MCP server **is** that
controller, so when the AI client stopped/reloaded it or a self-test finished, the controller connected then disconnected.
`--keep-alive` now holds the single controller on `tcp://127.0.0.1:1235` across client disconnects; `run.ps1 -Mode hold` holds it
independently of any AI client. `start_webots.ps1` still prevents a second Webots (running-process check + port-1235 listener check),
and Webots itself enforces that only one extern controller owns the robot.

## 2. Test results (newly executed, this environment)

- Code checks: `python -m pytest -q -m "not live"` -> **624 passed, 1 skipped** in 402.14 s (0:06:42), `.venv` Python 3.14, 2026-10-26.
- Targeted suites during development: `tests/test_mcp_server.py`, `tests/test_humanoid.py`, `tests/test_bridge.py` -> all passed (1 skipped).
- `--print-config` verified to emit `--keep-alive` for `--robot nao --sim webots`.
- Failures: none in the not-live suite. The 1 skip is a pre-existing environment-dependent skip (unchanged by this work).

Not executed here (no Webots / no live API keys in this environment): the `@pytest.mark.live` tests, the live Webots self-test
(`orbit_mcp_server.py --selftest --robot nao --sim webots`), and the NAO benchmark. The live persistence behaviour of `--keep-alive`
(controller staying connected across client disconnects in a real Webots window) is therefore **implemented and unit-consistent but
not yet confirmed against live Webots** — run it on a machine with Webots open to complete the evidence pack.

## 3. Documentation updated

- `README.md`: title to v0.16.5; quickstart to `start_webots.ps1` + the `--keep-alive` launch command; removed stale "persistent
  controller / keep the bridge window open" wording (added a transition note explaining old "bridge" language); corrected the install
  step (setup.ps1 runs `unittest discover`, no fixed "441 tests"); fixed the F5 table to the real launch names; added a
  "Keeping the NAO controller connected" section (AI-client keep-alive, `-Mode hold`, reconnect handling, duplicate-process prevention);
  fixed Claude Desktop / VS Code / Step 5.2 / 5.3 / 5.4 / section 6 to drop `--bridge-only` and the bridge window; corrected the NAO MCP
  tool count to 49; refreshed the test-results line to the verified 624/1.
- `CHANGELOG.md`: expanded the 0.16.5 entry with the keep-alive fix, the config changes, `run.ps1 -Mode hold`, and the verified counts.
- `docs/MCP_SERVER.md`: intro updated to `start_webots.ps1` + `--keep-alive`, with a keep-alive/`-Mode hold` note.

Distinguished clearly: historical results (prior CHANGELOG entries, prior report docs) are left intact; the newly executed 624/1 count
and the keep-alive fix are labelled as this session's work; live Webots confirmation is called out as still pending.

## 4. Backup / storage audit

All of the following are **untracked by git and not git-ignored**, and are **not referenced** by any active code, config, test, launch
script, or recovery path (the only in-code references found were self-references *inside* the archive/backup copies). Git history is
the project's primary recovery mechanism.

| Directory | Files | Size | git-tracked | Assessment |
|---|---|---|---|---|
| `_archive_human_experiment_20261025_124510` | 41 | 14.97 MB | no | obsolete archived experiment; safe to reclaim |
| `_work_backup` | 54 | 0.77 MB | no | timestamped working backups (incl. today's pre-edit copies) — keep the latest, older ones reclaimable |
| `_backups_20261025_003321` | 10 | 0.39 MB | no | older snapshot; reclaimable |
| `_backups_20261024_151132` | 12 | 0.21 MB | no | older snapshot; reclaimable |
| `_backups_20261024_163632` | 2 | 0.12 MB | no | older snapshot; reclaimable |
| `_backups_20261024_172922` | 3 | 0.04 MB | no | older snapshot; reclaimable |
| `_human_workspace` | 7 | 0.03 MB | no | isolated human-experiment workspace; reclaimable |
| `__pycache__` (root) | 17 | 0.31 MB | **git-ignored** | regenerated bytecode; always safe to delete |

**Total reclaimable if all removed: ~16.84 MB.**

No deletions were performed. Deleting user recovery backups is irreversible and outside the minimal-change scope of a documentation
task; per the maintenance policy ("never delete the only known-good recovery copy; if uncertain, retain and report"), these are
reported for the owner to confirm. Recommended safe order if you choose to reclaim space:
1. `__pycache__` (git-ignored, auto-regenerated) — always safe.
2. `_archive_human_experiment_20261025_124510` (largest, obsolete) — safe; the human-experiment code is not part of the NAO runtime.
3. Older `_backups_*` and `_human_workspace` — keep the newest `_work_backup/<timestamp>/` (today's pre-edit copies) as a recovery point.

Note: `.kiro/steering/orbit-rules.md` states `_work_backup/` is git-ignored, but `.gitignore` does not list it. Consider adding
`_work_backup/`, `_backups_*/`, `_archive_*/` and `_human_workspace/` to `.gitignore` so these never enter a commit or a release zip.

## 5. Recovery checkpoint and restore

- Pre-edit copies of every file changed this session are in `_work_backup/20261026_002800/`:
  `orbit_mcp_server.py`, `run.ps1`, `.vscode/mcp.json`, `README.md`.
- The repository is a git working tree; `git status` / `git diff` show all modifications, and `git checkout -- <file>` restores any
  tracked file to its committed state.
- To restore a changed file from the session backup:
  `Copy-Item _work_backup/20261026_002800/<file> <destination>` (e.g. `Copy-Item _work_backup/20261026_002800/README.md README.md`).

## 6. Remaining risks / recommendations

- Live Webots confirmation of `--keep-alive` is still pending (see section 2); run the live self-test and watch the Webots console for
  a continuously connected `orbit_nao` controller across an AI-client disconnect.
- Only one extern controller may own the NAO; running two of {AI-client server, `-Mode hold`, benchmark} at once will fail to connect.
- Add the backup/archive directories to `.gitignore` as noted above.
