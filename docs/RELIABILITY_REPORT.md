# ORBIT ROS NAO — Reliability, Reconnection & Benchmark Report (Phase 8, 2026-10-24)

All results below were executed on this PC against the real Webots R2025a house world (`webots/worlds/orbit_house.wbt`), via the
direct extern-controller transport on `tcp://127.0.0.1:1235`. No port 1240 was used. Fake-simulator checks are labelled as such and are
never presented as live results.

Environment during the tests:
- Webots `webots.exe` PID 7360 / `webots-bin.exe` PID 19028, controller port **1235 LISTENING** (owner 19028) throughout.
- No listener or connection on port **1240** at any point (`Get-NetTCPConnection -LocalPort 1240` => 0).
- Python venv `.venv\Scripts\python.exe`.

## Reproduce

```powershell
# 1. Open the house (controller waits on 1235)
.\start_webots.ps1
# 2. Unit + fake-sim reliability tests
.\.venv\Scripts\python.exe -m pytest tests\test_humanoid.py::TestConnectionLoss tests\test_humanoid.py::TestConnectionStatus -p no:cacheprovider
# 3. Live scenarios (each a fresh process = one MCP session)
.\.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim webots --keep-alive   # full guarded path
.\.venv\Scripts\python.exe _phase8_benchmark.py 3                                                 # reliability metrics (temp script, quarantined)
```

## Controller lifecycle — confirmed behavior

**The Webots extern controller is meant to EXIT at the end of each MCP session, not stay resident.** Webots keeps the robot and the world;
when the controller process ends, Webots returns the controller socket to LISTENING and re-accepts the next controller. This is the
standard extern-controller model and it is what the 1235-only transport relies on.

Hard Webots constraint discovered and respected: **only one `Robot`/`Supervisor` instance may be created per controller process**
(`Error: only one Robot instance can be created per controller process.`). Therefore **reconnection means a new process**, not a second
`HumanoidSession` in the same process. Each MCP session is already a fresh process, so this matches production exactly. The Phase 8
benchmark was corrected to run one process per trial after an initial in-process loop hit this limit (documented here rather than hidden).

## Scenario-by-scenario results

| # | Scenario | Method | Result | Evidence |
|---|---|---|---|---|
| 1 | Start Webots, connect MCP | `start_webots.ps1` already up; direct `HumanoidSession(sim="webots")` | **PASS** — connected in 0.63 s | scenario A log 16:34:52 |
| 2 | `connection_status`, `get_pose`, read-only | live session | **PASS** — `connection_status` connected=True, backend=webots, port=1235; get_pose ok; describe_view 5 items | scenario A |
| 3 | Safe turn + walk, measured before/after | true supervisor pose read | **PASS** — turn left 60° = **+59.0° (err 1.0°)**; walk 2 = **0.189 m** | scenario A 16:34:57–16:35:03 |
| 4 | Normal client/server close | process exit | **PASS (expected disconnect)** — exit 0; port 1235 returned to LISTENING (19028) within ~2 s | pre/post port check 16:34:49 → 16:35:05 |
| 5 | New session reconnects | 3 fresh processes, same running Webots | **PASS 3/3** — reconnect in 0.50–0.56 s each, robot live | reconnect log 16:35:26–16:35:32 |
| 6 | Pause/resume, reset/reload | Not executed as a GUI action | **NOT RUN** — would require GUI manipulation that cannot be safely scripted/restored headlessly; the bridge-era idle keep-alive that tolerated pause is retired. The direct session surfaces a paused/ended sim as `{status:error}` (see scenario 7). Documented as a limitation. |
| 7 | Unexpected connection loss (controlled) | Inject `step()`/`pose()` → `RuntimeError("Webots simulation ended")` on a LIVE session; no process killed | **PASS (after fix)** — `walk_forward` and `get_pose` return `{status:error}` with an actionable message; `connection_status` → connected=False; unrelated `house_info` still works. Before the fix, these **raised** out of `robot_call`. | `_phase8_fault.txt` (before) / `_phase8_fault_after2.txt` (after) |

## Confirmed defect and root cause

**Defect:** On an unexpected Webots connection loss, `HumanoidSession.robot_call` raised `RuntimeError` instead of returning a
`{status:error}` dict for motion and sensing tools.

**Root cause (evidence: Python traceback captured during fault injection):**
1. `robot_call` ran `self._ensure()` and `validate_humanoid_call(..., self._pose()["fallen"])` **before** its `try/except`, so the pose
   read (`self.nao.pose()`) raised and escaped.
2. Even after guarding that, `_finish()` → `_trace()` → `debug_state()` → `self._pose()` read the dead backend **again** inside the
   error-reporting path, re-raising.

**Fix (minimal, `orbit_ros/humanoid.py` only):**
- `robot_call`: read the pose for validation defensively. If it fails, remember the error, still run `validate_humanoid_call` (so the
  **stop-latch and argument validation fire even with a dead backend** — safety first), then return a clean, logged `{status:error}`
  ("the robot connection was lost ...; restart the Webots house and start a new MCP session") for calls that would otherwise proceed.
- `_trace`: the audit/overlay path is diagnostic only; wrap its `debug_state()` read so a dead backend records a minimal note instead of
  re-raising. (`_show` was already defended.)
- No change to the safety guard, stop-latch, argument validation, or any tool schema. Normal (connected) behavior is byte-for-byte the same.

**Regression tests (fake sim, `tests/test_humanoid.py::TestConnectionLoss`):** motion call during loss → error (not raise) and counted via
`_finish`; read call during loss → error; `connection_status` never raises and never claims 1240; unrelated read-only MCP survives; and
**the stop-latch still rejects a motion even during a connection loss** (safety invariant preserved).

## Measured reliability metrics (LIVE, 3 trials, one process per trial)

Source file: `results_nao_benchmark/reliability_20261024T145555Z/reliability.json`.

| Metric | Value | Sample size |
|---|---|---|
| Command success rate | **18 / 18 = 100%** | 18 commands |
| Task completion rate | **3 / 3 = 100%** | 3 tasks |
| Turn error (deg) | mean **0.9°**, max **1.1°** (tol 15°) | 9 turns |
| Walk displacement (m) | mean **0.142 m**, range 0.093–0.190 | 6 walks |
| Unexpected disconnects | **0** | 3 trials |
| Reconnections | **3 / 3 = 100%**, mean **0.55 s**, max 0.59 s | 3 cycles |
| Safety vetoes (correct refusals) | **3** | — |
| Safety violations | **0** | — |
| Rejected commands | 0 (all issued commands were valid) | — |

Fake-sim argument/safety regression (labelled fake, not live): registry (44) == live MCP `tools/list` (44), 0 missing / 0 extra;
9/9 validation + veto + stop-latch checks passed (`_phase8_regression.txt`).

### Limitations / sample size
- Live sample is small (3 trials, 9 turns, 6 walks) — enough to confirm correctness and stability, not a statistical study. Re-run
  `_phase8_benchmark.py <N>` for larger N.
- Walking displacement per cycle is intentionally short (~0.09–0.19 m) and open-loop; the session reads true pose back after each motion.
- Scenario 6 (GUI pause/resume/reset) was not executed — see table. The disconnect path it would exercise is covered by scenario 7.
- All numbers are from the live Webots run; fake-sim results are reported separately and never merged in.

## Files changed (this phase) and why
- `orbit_ros/humanoid.py` — `robot_call` connection-loss handling (safety-first ordering) and `_trace` diagnostic resilience. The confirmed defect fix.
- `tests/test_humanoid.py` — new `TestConnectionLoss` (5 tests) covering the fix and the stop-latch-under-loss invariant.
- Docs: this report, `TASKS.md`, `docs/ARCHITECTURE.md`, `docs/COMMAND_MATRIX.md`, `CLEANUP_REPORT.md`.
- Backups of edited sources: `_backups_20261024_163632\`.

## Unresolved / next actions
- None blocking. Optional: a larger-N live benchmark; and an automated (non-GUI) pause/resume test if a future Webots supervisor hook allows it.
- The two `.bak` files remain flagged for approval (unchanged).
