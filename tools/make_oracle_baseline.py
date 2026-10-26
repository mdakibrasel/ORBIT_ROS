"""Regenerate results_spatial_oracle/trial_results.csv, the snapshot tests/test_instruction_ladder.py compares against.

Run it only when a deliberate change to the 2D tasks or agent ladder changes the oracle results:
    python tools/make_oracle_baseline.py
"""
from __future__ import annotations
import csv, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros.tasks2d import make_task, FAMILIES
from orbit_ros.agent2d import S_LADDER, run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend
from run_spatial import grid

COLUMNS = ("task_id", "config", "success", "llm_calls", "env_tool_calls", "collisions", "final_x", "final_y", "final_yaw")

def main():
    rows = []
    for fam, s, yaw, k, m, lv in grid(FAMILIES, 1):
        for name in S_LADDER:
            w, t = make_task(fam, s, yaw, k, m, lv)
            r = run_episode(w, t, PRESETS[name], OracleBackend(0.05, 0.30, abstain=False))
            rows.append({"task_id": t.task_id, "config": name, **{c: r[c] for c in COLUMNS[2:]}})
    out = os.path.join(ROOT, "results_spatial_oracle"); os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "trial_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS); w.writeheader(); w.writerows(rows)
    print(f"wrote {len(rows)} rows to {out}")

if __name__ == "__main__":
    main()
