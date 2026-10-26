"""Webots validation runner (stage C1 grid-faithful, and the continuous-motion mode).

The same Oracle episodes run twice: on pure World2D (reference) and with the task world AND the calibration bay running in the
simulator. Agreement = equal success, abstention, collisions and final pose.

Inside Webots this was called by the removed TurtleBot controller. Without Webots, `--sim fake` runs the identical
code path against FakeSim (plumbing test, NOT Webots evidence):
    python run_webots_c1.py --sim fake --mode probe
    python run_webots_c1.py --sim fake --motion continuous --limit 24 --out results_webots_fake
"""
import argparse, itertools, json, os, platform, sys, time
from datetime import datetime, timezone
from orbit_ros import embodiment_runtime
from orbit_ros.world2d import World2D, Obj
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import PRESETS
from orbit_ros.agent_robust import run_episode_robust
from orbit_ros.agent_recovery import run_episode_v08, V08_KEYS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment
from orbit_ros.webots_adapter import WebotsWorld2D, FakeSim, calibrate_lidar
from run_v08 import arms as v08_arms, make_strategy
from run_spatial import write_csv

def one_episode(cfg, arm, cond, fam, yaw, seed, h, sim=None, perception="camera", motion="grid"):
    """One episode. sim=None -> plain World2D. With a sim, the task world and every real bay run are simulator-backed.
    Returns (row, worlds) where worlds lists every simulator-backed world that was used (task world first)."""
    world, task = make_task(fam, seed, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1)
    apply_embodiment(world, h)
    if cond["drift_at_call"] is not None:
        world.drift_schedule.append((cond["drift_at_call"], {"move_gain": 0.5 if world.move_gain != 0.5 else 1.0}))
    worlds = []
    if sim is not None:
        world = WebotsWorld2D.adopt(world, sim, perception, motion); worlds.append(world)
        embodiment_runtime.BAY_HOOK = lambda bay: worlds.append(WebotsWorld2D.adopt(bay, sim, perception, motion)) or worlds[-1]
    try:
        st = make_strategy(cfg, arm["kind"], cond["noise"], seed, arm.get("integrity", False))
        agent, be = PRESETS[cfg["agent_config"]], OracleBackend(cfg["slip"], 0.0)
        row = run_episode_robust(world, task, agent, be, st) if arm["kind"] == "old" else run_episode_v08(world, task, agent, be, st, arm["rcfg"])
    finally:
        embodiment_runtime.BAY_HOOK = None
    return row, worlds

GATE_AGREEMENT = 0.95

def outcome(row):
    return (int(row["success"]), int(float(row["aborted"]) > 0), float(row["collisions"]), int(float(row["final_x"])), int(float(row["final_y"])), int(float(row["final_yaw"])))

def classify(agree, worlds, motion):
    """Automatic first guess only; every disagreement still needs a human explanation (Gate C1)."""
    if agree: return ""
    if sum(w.view_mismatch for w in worlds): return "perception"
    if max(w.sync_error_max for w in worlds) > 0.05 and motion == "grid": return "motion"
    if motion == "continuous": return "contact" if sum(w.contact_collisions for w in worlds) else "motion"
    return "unclassified"

def run_c1(sim, cfg, arm_name, cond_name, seeds, out, perception="camera", motion="grid", limit=None, log=print):
    arm = next(a for a in v08_arms(cfg) if a["arm"] == arm_name)
    cond = next(c for c in cfg["conditions"] if c["name"] == cond_name)
    if motion == "continuous": log(f"[C1] lidar calibrated: front ray index {calibrate_lidar(sim)}")
    jobs = list(itertools.product(cfg["families"], cfg["yaws"], seeds, hypothesis_space()))
    if limit: jobs = jobs[:limit]
    rows, t_start = [], time.perf_counter()
    for i, (fam, yaw, seed, h) in enumerate(jobs):
        ref, _ = one_episode(cfg, arm, cond, fam, yaw, seed, h)                                 # World2D reference
        s0, t0 = sim.sim_time(), time.perf_counter()
        row, ws = one_episode(cfg, arm, cond, fam, yaw, seed, h, sim, perception, motion)       # simulator-backed
        a, b = outcome(ref), outcome(row)
        agree = a == b
        rows.append({"episode": f"{fam}|y{yaw}|s{seed}|{h.key}", "arm": arm_name, "condition": cond_name, "family": fam, "yaw": yaw, "seed": seed,
                     "true_embodiment": h.key, "sim_mode": motion, "perception": perception,
                     "w2d_success": a[0], "wb_success": b[0], "w2d_aborted": a[1], "wb_aborted": b[1],
                     "w2d_collisions": a[2], "wb_collisions": b[2], "w2d_final": f"{a[3]},{a[4]},{a[5]}", "wb_final": f"{b[3]},{b[4]},{b[5]}",
                     "agree": int(agree), "disagree_cause": classify(agree, ws, motion), "bay_runs": len(ws) - 1,
                     "view_calls": sum(w.view_calls for w in ws), "view_mismatch": sum(w.view_mismatch for w in ws),
                     "sync_error_max": round(max(w.sync_error_max for w in ws), 4), "contact_collisions": sum(w.contact_collisions for w in ws),
                     "odom_error_m": round(max(w.odom_error_max for w in ws), 4), "sim_seconds": round(sim.sim_time() - s0, 3), "wall_seconds": round(time.perf_counter() - t0, 3)})
        if (i + 1) % 10 == 0 or i + 1 == len(jobs): log(f"[C1] {i + 1}/{len(jobs)} episodes, agreement so far {sum(r['agree'] for r in rows)}/{len(rows)}")
    os.makedirs(out, exist_ok=True)
    write_csv(os.path.join(out, "webots_results.csv"), rows)
    n, k = len(rows), sum(r["agree"] for r in rows)
    # Gate C1 (v0.10): Webots must reproduce World2D on >= 95% of episodes AND every disagreement needs a written explanation.
    dis = [dict(r, human_explanation="") for r in rows if not r["agree"]]
    write_csv(os.path.join(out, "disagreements.csv"), dis)
    gate = {"threshold": GATE_AGREEMENT, "agreement_ok": k / max(1, n) >= GATE_AGREEMENT, "disagreements_to_explain": len(dis),
            "by_auto_cause": {c: sum(1 for r in dis if r["disagree_cause"] == c) for c in sorted({r["disagree_cause"] for r in dis})}}
    meta = {"gate_c1": gate, "timestamp_utc": datetime.now(timezone.utc).isoformat(), "python": sys.version, "platform": platform.platform(), "sim": type(sim).__name__,
            "webots_home": os.environ.get("WEBOTS_HOME", ""), "arm": arm_name, "condition": cond_name, "seeds": list(seeds), "perception": perception, "motion": motion,
            "episodes": n, "agree": k, "agreement": k / max(1, n), "wall_seconds": round(time.perf_counter() - t_start, 1),
            "note": "FakeSim runs are plumbing tests, not Webots evidence." if isinstance(sim, FakeSim) else "Record the Webots version shown in Help > About Webots."}
    with open(os.path.join(out, "run_metadata.json"), "w") as f: json.dump(meta, f, indent=2)
    log(f"[C1] agreement {k}/{n} = {k / max(1, n):.3f}  -> {out}")
    log(f"[C1] GATE (>= {GATE_AGREEMENT:.0%}): {'PASS' if gate['agreement_ok'] else 'FAIL'}; {len(dis)} disagreement(s) to explain in disagreements.csv "
        f"(column human_explanation); automatic first guess: {gate['by_auto_cause']}")
    return rows, meta

def view_of(w): return World2D.describe_view(w)

def probe(sim, log=print, motion_test=True):
    """Compare what the simulator reports with World2D's own maths for many viewpoints, then (optionally) test real wheel motion."""
    worst_ix = worst_size = 0.0; bad = lines = seen = 0
    for fam, seed in (("goto", 3), ("remember_return", 3), ("clean_left", 3)):
        for yaw in (0, 90, 180, 270):
            for mirrored in (False, True):
                world, task = make_task(fam, seed, yaw, 3, False, 1); world.mirrored = mirrored
                w = WebotsWorld2D.adopt(world, sim, "camera")
                for (cx, cy) in [(x, y) for x in range(w.w) for y in range(w.h) if w.free(x, y)][::5]:     # many robot cells, so objects are really in view
                    w.x, w.y = cx, cy; w._push()
                    cam, truth = w._camera_view(), view_of(w)
                    ok = w.views_match(cam, truth)
                    for o in cam:
                        t = [q for q in truth if q["label"] == o["label"]]
                        if t: worst_ix = max(worst_ix, min(abs(q["image_x"] - o["image_x"]) for q in t)); worst_size = max(worst_size, min(abs(q["apparent_size"] - o["apparent_size"]) for q in t))
                    bad += 0 if ok else 1; lines += 1; seen += len(truth)
                    if not ok: log(f"[probe] {fam} yaw={yaw} mirrored={int(mirrored)} cell=({cx},{cy}) MISMATCH\n   camera: {cam}\n   truth : {truth}")
                    got = w.pose()
                    if (got["x"], got["y"], got["yaw_deg"]) != (w.x, w.y, w.yaw):
                        bad += 1; log(f"[probe] POSE MISMATCH read-back {got} vs expected {(w.x, w.y, w.yaw)}")
    log(f"[probe] camera: {lines} viewpoints, {seen} visible objects compared, {bad} problems, worst |image_x| error {worst_ix:.3f}, worst |apparent_size| error {worst_size:.3f}")
    if motion_test: bad += probe_motion(sim, log)
    log("[probe] RESULT: " + ("all checks passed" if bad == 0 else f"{bad} problem(s): read the lines above"))
    return bad == 0

def probe_motion(sim, log=print):
    """Real wheel motion: lidar calibration, a 2-cell drive, blocked detection at an obstacle, a 90-degree turn. Returns the number of problems."""
    bad = 0
    try: idx = calibrate_lidar(sim); log(f"[probe] lidar calibrated: front ray index {idx}")
    except Exception as e: log(f"[probe] LIDAR PROBLEM: {e}"); return 1
    w = WebotsWorld2D.adopt(World2D(9, 7, (4, 3), 0, [Obj("c", "calibration marker", 7, 3)]), sim, "camera", "continuous")
    def check(name, ok, detail):
        nonlocal bad
        log(f"[probe] motion {name}: {'ok' if ok else 'PROBLEM'} ({detail})")
        bad += 0 if ok else 1
    r = w.execute("move_forward", {"distance": 2}); x, y, yaw = sim.robot_pose()
    check("drive 2 cells", r["ok"] and abs(x - 6) < 0.05 and abs(y - 3) < 0.05, f"pose x={x:.3f} y={y:.3f} yaw={yaw:.1f}, message '{r['message']}'")
    r = w.execute("move_forward", {"distance": 1}); x2, y2, _ = sim.robot_pose()
    check("blocked before the obstacle", (not r["ok"]) and r["message"] == "blocked after 0 m" and abs(x2 - x) < 0.05, f"message '{r['message']}', moved {abs(x2 - x):.3f} m, contacts {w.contact_collisions}")
    r = w.execute("turn", {"direction": "left", "degrees": 90}); _, _, yaw = sim.robot_pose()
    check("turn left 90", r["ok"] and abs(((yaw - 90) + 180) % 360 - 180) < 1.0, f"yaw {yaw:.2f} deg")
    r = w.execute("move_forward", {"distance": 3}); x3, y3, _ = sim.robot_pose()
    check("drive 3 cells north", r["ok"] and abs(x3 - 6) < 0.05 and abs(y3 - 6) < 0.05, f"pose x={x3:.3f} y={y3:.3f}, message '{r['message']}'")
    r = w.execute("move_forward", {"distance": 1})
    check("room wall stops the robot", not r["ok"], f"message '{r['message']}'")
    check("odometry vs truth", w.odom_error_max < 0.05, f"max {w.odom_error_max:.4f} m")
    return bad

def main(argv=None, sim=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/v0_8_1.json"); ap.add_argument("--arm", default="hybrid_old+integrity")
    ap.add_argument("--condition", default="static_n0.0"); ap.add_argument("--seeds", default="3"); ap.add_argument("--out", default="results_webots_c1")
    ap.add_argument("--sim", choices=["fake", "webots", "ros2"], default="webots"); ap.add_argument("--mode", choices=["probe", "run"], default="run")
    ap.add_argument("--perception", choices=["camera", "truth"], default="camera"); ap.add_argument("--motion", choices=["grid", "continuous"], default="grid")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--ros2-config", default=None, help="v0.13: JSON file of Ros2Sim arguments (topics, world and robot names), e.g. configs/ros2_jazzy_tb3.json")
    a = ap.parse_args(argv)
    from orbit_ros.env_config import load_env; load_env()
    with open(a.config) as f: cfg = json.load(f)
    if sim is None:
        if a.sim == "fake": sim = FakeSim()
        elif a.sim == "ros2":
            from orbit_ros.ros2_adapter import load_ros2_config, Ros2Sim     # untested against ROS 2: run --mode probe first (docs/ROS2_BRIDGE.md)
            sim = Ros2Sim(**load_ros2_config(a.ros2_config))
        else:
            from orbit_ros.webots_adapter import WebotsSim; sim = WebotsSim()
    if a.mode == "probe": return 0 if probe(sim) else 2
    from run_v08 import parse_seeds
    run_c1(sim, cfg, a.arm, a.condition, parse_seeds(a.seeds), a.out, a.perception, a.motion, a.limit or None)
    return 0

if __name__ == "__main__":
    sys.exit(main())
