# Webots validation record

> Fill this in on the machine that ran Webots. Keep it next to the run folders; never overwrite an earlier record, start a new dated copy.
> Do not paste API keys or the contents of `.env`.

## 1. Versions

| item | value |
|---|---|
| date (UTC) | |
| ORBIT ROS version and code hash (`run_metadata.json` -> `code_hash`) | |
| OS and version | |
| Webots version | |
| Python version (the one Webots uses) | |
| baseline record used (`baselines/<date>_v<version>/`) | |

## 2. Commands, in the order they ran

```
powershell -ExecutionPolicy Bypass -File .\start_webots.ps1 -Mode probe
python conformance.py --sim webots --motion grid          # inside the extern controller
.\start_webots.ps1 -Mode run -Motion grid -Out results_webots_grid_<date>
.\start_webots.ps1 -Mode run -Motion continuous -Out results_webots_continuous_<date>
```

## 3. Probe

- Result line (expected `[probe] RESULT: all checks passed`):
- Log file:
- If it failed: what failed, the fix, and the re-run result. Do not start a full run until the probe passes.

## 4. Conformance (grid, then continuous)

| check | grid | continuous | note |
|---|---|---|---|
| capabilities | | | |
| frame_units | | | |
| turn_convention | | | |
| camera_convention | | | |
| freshness | | | |
| invalid_args | | | |
| collision_stop | | | |
| stop | | | |
| reset_semantics | | | |
| ground_truth_split | | | |
| action_timeout | | | |

## 5. Gate C1 (grid): agreement with World2D

| episodes | agree | agreement % | gate (>= 95%) | disagreements |
|---|---|---|---|---|
| | | | PASS / FAIL | |

Every row of `disagreements.csv` must be explained:

| task_id | World2D | Webots | class (timing, physics, perception, adapter bug, other) | explanation |
|---|---|---|---|---|
| | | | | |

## 6. Continuous motion

Report each separately; do not compare grid and continuous as the same kind of motion (grid mode places the robot cell by cell).

| metric | value |
|---|---|
| stalls | |
| contacts / collisions | |
| odometry error, max and mean (m) | |
| final pose error (m) | |
| view mismatches | |
| simulated time / wall-clock time (s) | |

## 7. Limitations and open issues

-

## 8. What may be claimed

"Webots validated (grid / continuous)" only if the probe passed, conformance passed for that motion mode, the gate passed, and every
disagreement is explained above.
