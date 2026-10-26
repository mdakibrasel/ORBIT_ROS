"""Simulator agreement check (v0.14 milestone M5): run the same task instances and configurations on World2D and inside a simulator
backend, and compare every non-timing column of the episode rows.

    python tools/sim_agreement.py --sim fake --motion grid --configs C4 PEL --seeds 1
    python tools/sim_agreement.py --sim fake --motion continuous --configs C4 PEL --seeds 1 --limit 60

Planner: the synthetic OracleBackend (same settings as run_spatial.py), so results are SYNTHETIC. A disagreement is listed with the
columns that differ, the final poses on both backends and the simulator's own diagnostics (sync error, view mismatches, contact
collisions, odometry error). Writes <out>/sim_agreement_<sim>_<motion>.md and .json; exit code 0 only when every row agrees.
"""
from __future__ import annotations
import argparse, json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orbit_ros.tasks2d import make_task, ALL_FAMILIES
from orbit_ros.agent2d import run_episode, config_by_name
from orbit_ros.backends2d import OracleBackend
from orbit_ros.sim_select import in_simulator
from run_spatial import select_instances
from tools.compare_rows import TIMING

SIM_DIAG = ("sync_error_max", "view_calls", "view_mismatch", "contact_collisions", "odom_error_max")

def is_timing(k): return k in TIMING or k.endswith("_time_s")

def episode(inst, cfg, sim, motion):
    fam, s, yaw, n, m, lv = inst
    world, task = make_task(fam, s, yaw, n, m, lv)
    world = in_simulator(world, sim, motion)
    t0 = time.perf_counter()
    row = run_episode(world, task, cfg, OracleBackend(0.05, 0.30, 0.10, abstain=True))
    row["_wall_s"] = time.perf_counter() - t0
    row["_diag"] = {k: getattr(world, k) for k in SIM_DIAG if hasattr(world, k)}
    return row

def compare(a, b):
    keys = sorted((set(a) | set(b)) - {"_wall_s", "_diag"})
    return [k for k in keys if not is_timing(k) and a.get(k) != b.get(k)]

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sim", choices=["fake", "webots", "ros2"], default="fake")
    ap.add_argument("--motion", choices=["grid", "continuous"], default="grid")
    ap.add_argument("--configs", nargs="+", default=["C4", "PEL"])
    ap.add_argument("--families", nargs="*", default=list(ALL_FAMILIES))
    ap.add_argument("--seeds", type=int, default=1); ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default="sim_agreement")
    a = ap.parse_args(argv)
    inst = select_instances(a.families, a.seeds, "all", a.limit)
    cfgs = [config_by_name(c) for c in a.configs]
    res = []
    for i, it in enumerate(inst):
        for cfg in cfgs:
            r0, r1 = episode(it, cfg, "world2d", "grid"), episode(it, cfg, a.sim, a.motion)
            diff = compare(r0, r1)
            res.append({"task_id": r0["task_id"], "config": cfg.name, "agree": not diff, "differs": diff,
                        "success": [r0["success"], r1["success"]], "safety_violations": [r0["safety_violations"], r1["safety_violations"]],
                        "final_pose": [[r0["final_x"], r0["final_y"], r0["final_yaw"]], [r1["final_x"], r1["final_y"], r1["final_yaw"]]],
                        "sim_diag": r1["_diag"], "wall_s": [round(r0["_wall_s"], 4), round(r1["_wall_s"], 4)]})
        print(f"\rinstance {i + 1}/{len(inst)}", end="", file=sys.stderr)
    print(file=sys.stderr)
    n = len(res); agree = sum(r["agree"] for r in res)
    by = {}
    for r in res:
        d = by.setdefault(r["config"], {"episodes": 0, "agree": 0, "success_world2d": 0, "success_sim": 0, "safety_world2d": 0, "safety_sim": 0, "pose_differs": 0})
        d["episodes"] += 1; d["agree"] += r["agree"]; d["success_world2d"] += r["success"][0]; d["success_sim"] += r["success"][1]
        d["safety_world2d"] += r["safety_violations"][0]; d["safety_sim"] += r["safety_violations"][1]; d["pose_differs"] += r["final_pose"][0] != r["final_pose"][1]
    os.makedirs(a.out, exist_ok=True); base = os.path.join(a.out, f"sim_agreement_{a.sim}_{a.motion}")
    with open(base + ".json", "w") as f: json.dump({"sim": a.sim, "motion": a.motion, "configs": a.configs, "seeds": a.seeds, "limit": a.limit, "summary": by, "episodes": res}, f, indent=1)
    L = [f"# Simulator agreement: World2D vs {a.sim} ({a.motion} motion)", "", "SYNTHETIC (oracle planner). Every non-timing episode column compared.", "",
         f"**{agree}/{n} episodes agree.**", "", "| config | episodes | rows identical | success World2D | success sim | safety viol. World2D | safety viol. sim | final pose differs |",
         "|---|---|---|---|---|---|---|---|"]
    for c, d in by.items():
        L.append(f"| {c} | {d['episodes']} | {d['agree']} | {d['success_world2d']} | {d['success_sim']} | {d['safety_world2d']} | {d['safety_sim']} | {d['pose_differs']} |")
    dis = [r for r in res if not r["agree"]]
    if dis:
        L += ["", "## Disagreements (first 40)", "", "| task | config | differing columns | final pose World2D / sim | sim diagnostics |", "|---|---|---|---|---|"]
        for r in dis[:40]:
            L.append(f"| {r['task_id']} | {r['config']} | {', '.join(r['differs'][:8])} | {r['final_pose'][0]} / {r['final_pose'][1]} | {json.dumps(r['sim_diag'])} |")
    with open(base + ".md", "w") as f: f.write("\n".join(L) + "\n")
    print("\n".join(L[:12 + len(by)])); print("wrote", base + ".md")
    return 0 if agree == n else 1

if __name__ == "__main__": sys.exit(main())
