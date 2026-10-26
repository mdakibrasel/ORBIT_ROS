# ORBIT-ROS release evidence — 2026-10-26

This report records commands actually run while preparing the repository for a first public release. It is intentionally separate
from historical phase reports and does not convert an unverified result into a release claim.

## Environment

- Windows PowerShell.
- Repository branch: `main`.
- Python: project `.venv`.
- Webots: `C:\Program Files\Webots` when live checks were run.
- Controller endpoint: `tcp://127.0.0.1:1235/orbit_nao`.
- Robot: `orbit_nao`.

## PASS

| Area | Command/evidence | Outcome |
|---|---|---|
| Focused MCP/discovery tests | `python -m unittest tests.test_mcp_server tests.test_webots_discovery` | 40 tests, 1 optional skip, `OK` before final lifecycle additions |
| Final lifecycle tests before owner-check patch | `python -m unittest tests.test_mcp_server tests.test_humanoid.TestConnectionStatus` | 39 tests, 1 optional skip, `OK` |
| Owner-check regression tests | `python -m unittest tests.test_humanoid.TestConnectionStatus` | 6 tests, `OK`; stale backend invalidation logged as `DISCONNECTED` |
| Complete non-live suite before owner-check patch | `python -m pytest -m "not live"` | **626 passed, 1 skipped, 0 failed, 618.80 seconds** |
| Complete non-live suite after owner-check patch | `.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py"` | **628 tests, 1 skipped, 0 failed, `OK`, 10808.586 seconds** |
| Python syntax | `python -m py_compile orbit_mcp_server.py orbit_ros\humanoid.py tests\test_humanoid.py tests\test_mcp_server.py` | Passed |
| PowerShell syntax | Parser validation of startup, health, Webots, and run scripts | `PowerShell syntax OK` |
| Live health | `python orbit_mcp_server.py --health-check --verify-robot --robot nao --sim webots` | `READY`, pose `(4.800, 3.100, 180.0 deg, fallen=False)`, exit `0` |
| Wrapper integration | `tools\start_orbit_mcp.ps1` from no Webots process | Started Webots, waited for port `1235`, returned valid MCP `initialize` JSON |
| Repeated live calls | MCP `connection_status` and `get_pose` calls | `connected=true`, backend `webots`, correct endpoint and pose |
| Shutdown cleanup | PID-specific stop of wrapper, MCP, and Webots processes | No remaining ORBIT/Webots process or listener on `1235` |
| Reconnect regression | `test_webots_reconnect_restores_scene_without_replaying_motion` | Passed; logs `RECONNECTING` then `CONNECTED` |
| Live wrapper status/pose/motion | `tools\start_orbit_mcp.ps1` with real Webots R2025a; MCP `initialize`, `connection_status`, `get_pose`, `load_scene`, `walk_forward`, repeated `get_pose` | PASS; real pose changed from `(4.800, 3.100, 180.0)` to `(4.703, 3.094, 179.8)`, remained upright; Webots listener cleaned by PID |
| Live stale-status guard | Real live restart/replacement investigation plus owner-change regression | Fixed and covered: `connection_status` now invalidates a replaced Webots listener before probing the stale Supervisor |

## NOT RUN or incomplete

| Area | Status | Detail |
|---|---|---|
| Live Webots interruption/restart | INCOMPLETE | A bounded forced listener-owner interruption was attempted three times; each post-restart `get_pose` timed out inside the synchronous Webots controller API. The stale-status diagnostic path is fixed, but full in-process controller reattachment is not claimed. |
| Live Ctrl+C test | NOT RUN | Explicit PID shutdown was verified; an interactive Ctrl+C session was not recorded. |
| LLM provider tests | NOT RUN | No provider credentials were used. |

The prior README count of `624 passed, 1 skipped` is superseded by the current complete results above. The final suite includes
the owner-check regression and completed with no failures.

## Performance evidence

- Python health probe before optimization: 5-run mean `1643.3 ms`.
- Python health probe after lazy probe change: 5-run mean `1585.3 ms`.
- Native PowerShell health probe: 5-run mean `979.3 ms`, median `964.2 ms`.
- Fresh Webots process to port `1235`: one direct measurement `4001.7 ms`.

## Release decision

`v1.0.0` is **not claimed** by this checkpoint. The remaining release gate is live Webots interruption/restart evidence and an
interactive Ctrl+C shutdown capture. No tag, release, remote branch, or Git history was modified.
