"""Embodiment-identification experiments (OracleBackend, harness-level evidence only).

  python run_embodiment.py --out results_embodiment            # all three experiments
  python run_embodiment.py --obs-noise 0.1 --reps 40 --out results_embodiment_noisy

A  identification : accuracy / probes / bay travel per strategy over all 12 hidden embodiments (optionally noisy)
B  task success   : S5 agent + strategy on hidden embodiments, paired worlds, Wilson 95% CIs
C  surprise monitor: detection delay vs false-alarm rate vs threshold under pose noise and mid-run drift
The Oracle backend validates the harness. These numbers say nothing about any real LLM.
"""
import argparse, csv, itertools, json, math, os, random, statistics, sys
from datetime import datetime, timezone
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.world2d import World2D
from orbit_ros.embodiment import SurpriseMonitor
from orbit_ros.embodiment_runtime import (hypothesis_space, apply_embodiment, hypothesis_of, EmbodimentStrategy, EmbodimentAdapter,
                                          IDENTIFIERS, predict_pose, STRATEGY_NAMES, NOMINAL)
from run_spatial import write_csv

ID_METHODS = ("fixed_probe", "random_probe", "kinematic_cal", "infogain")

def wilson(k, n, z=1.96):
    if n == 0: return (float("nan"),) * 2
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n); r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - r) / d, (c + r) / d

def key3(h): return (h.mirrored_camera, h.left_right_swapped, round(h.move_gain, 3))

def exp_identification(reps, eps, noise_p, cost_weight):
    rows = []
    for method in ID_METHODS:
        for h in hypothesis_space():
            for r in range(reps):
                rng = random.Random(f"id|{method}|{h.key}|{r}")
                kw = {"eps": eps, "noise_p": noise_p, "rng": rng}
                if method == "infogain": kw["cost_weight"] = cost_weight; kw["max_probes"] = 6
                if method == "random_probe": kw["max_probes"] = 6
                res = IDENTIFIERS[method](h, **kw)
                rows.append({"method": method, "truth": h.key, "belief": res.belief.key, "correct": int(key3(res.belief) == key3(h)),
                             "probes": res.probes, "bay_actions": res.bay_actions, "bay_travel": res.bay_travel,
                             "confidence": round(res.confidence, 4), "status": res.status})
    return rows

def exp_tasks(strategies, seeds, families, eps, noise_p, cfg_name, slip):
    rows = []
    for fam, yaw, s, h, name in itertools.product(families, (0, 90, 180, 270), range(seeds), hypothesis_space(), strategies):
        n = 2 if fam == "remember_return" else 3
        world, task = make_task(fam, s, yaw, n, False, 2 if fam == "motion" else 1)
        apply_embodiment(world, h)                                   # identical world for every strategy (paired)
        strat = EmbodimentStrategy(name, eps=eps, noise_p=noise_p, seed=s)
        try: row = run_episode(world, task, PRESETS[cfg_name], OracleBackend(slip, 0.0), embodiment=strat)
        except Exception as e: row = {"task_id": task.task_id, "family": fam, "config": cfg_name, "success": 0, "error": repr(e)[:200],
                                      "embodiment_method": name, "true_embodiment": h.key}
        rows.append(row)
    return rows

def exp_monitor(thresholds, noise_ps, drifts, trials, n_steps=40, drift_step=20):
    """Random action stream on an empty world; hardware drifts at drift_step; pose reading is wrong with prob noise_p."""
    rows = []
    for thr, noise_p, (dname, change), t in itertools.product(thresholds, noise_ps, drifts, range(trials)):
        rng = random.Random(f"mon|{thr}|{noise_p}|{dname}|{t}")
        w = World2D(31, 31, (15, 15), 0, [])
        belief = NOMINAL; ad = EmbodimentAdapter(belief); mons = {}
        detected_at, false_alarm = None, False
        for step in range(n_steps):
            if step == drift_step:
                for k, v in change.items(): setattr(w, k, v)
            if rng.random() < 0.5: tool, args = "turn", {"direction": rng.choice(["left", "right"]), "degrees": rng.choice([90, 270])}
            else: tool, args = "move_forward", {"distance": rng.randint(2, 6)}
            for hw_tool, hw_args in ad.translate(tool, args):
                pre = w.pose(); r = w.execute(hw_tool, hw_args)
                if not r["ok"]:
                    w.x, w.y = 15, 15; break                          # re-centre after hitting a wall; not an observation
                pred = predict_pose(pre, hw_tool, hw_args, belief); p = w.pose(); obs = [p["x"], p["y"], p["yaw_deg"]]
                if rng.random() < noise_p: obs[rng.choice([0, 1, 2])] += rng.choice([-1, 1]) * (90 if False else 1)   # pose-reading glitch
                ev = mons.setdefault(hw_tool, SurpriseMonitor(thr)).observe(pred, tuple(obs), hw_tool)
                if ev["trigger"] and detected_at is None:
                    if step < drift_step: false_alarm = True
                    detected_at = step
        rows.append({"threshold": thr, "pose_noise": noise_p, "drift": dname, "trial": t,
                     "false_alarm_before_drift": int(false_alarm),
                     "detected_after_drift": int(detected_at is not None and detected_at >= drift_step and not false_alarm),
                     "delay_steps": (detected_at - drift_step) if (detected_at is not None and detected_at >= drift_step) else ""})
    return rows

def summarize_tasks(rows):
    out = {}
    for r in rows: out.setdefault(r["embodiment_method"], []).append(r)
    lines = ["| strategy | success (95% CI) | belief correct | probes/ep | bay travel/ep (m) | repairs/ep | LLM calls/ep |", "|---|---|---|---|---|---|---|"]
    for name in STRATEGY_NAMES:
        rs = out.get(name)
        if not rs: continue
        k, n = sum(int(r["success"]) for r in rs), len(rs); lo, hi = wilson(k, n)
        m = lambda key: statistics.mean(float(r.get(key) or 0) for r in rs)
        bc = sum(int(r.get("belief_correct") or 0) for r in rs) / n
        lines.append(f"| {name} | {k/n:.3f} ({lo:.3f}–{hi:.3f}) | {bc:.3f} | {m('id_probes'):.2f} | {m('id_bay_travel'):.1f} | {m('repairs'):.2f} | {m('llm_calls'):.1f} |")
    return lines

def by_factor(rows, factor):
    names = [n for n in STRATEGY_NAMES if any(r["embodiment_method"] == n for r in rows)]
    vals = sorted({r[factor] for r in rows}, key=str)
    lines = [f"| {factor} | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for v in vals:
        cells = []
        for n in names:
            rs = [r for r in rows if r["embodiment_method"] == n and r[factor] == v]
            cells.append(f"{sum(int(r['success']) for r in rs) / max(1, len(rs)):.2f}")
        lines.append(f"| {v} | " + " | ".join(cells) + " |")
    return lines

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results_embodiment"); ap.add_argument("--reps", type=int, default=20)
    ap.add_argument("--seeds", type=int, default=2); ap.add_argument("--cfg", default="S5")
    ap.add_argument("--strategies", nargs="*", default=list(STRATEGY_NAMES))
    ap.add_argument("--families", nargs="*", default=["motion", "goto", "clean_left", "remember_return"])
    ap.add_argument("--obs-noise", type=float, default=0.0, help="probability a probe's camera reading is flipped")
    ap.add_argument("--eps", type=float, default=None, help="identifier's assumed misread rate (default = obs-noise)")
    ap.add_argument("--cost-weight", type=float, default=0.0); ap.add_argument("--slip", type=float, default=0.0)
    ap.add_argument("--monitor-trials", type=int, default=100)
    a = ap.parse_args()
    eps = a.obs_noise if a.eps is None else a.eps
    os.makedirs(a.out, exist_ok=True)
    print("A identification...", file=sys.stderr); A = exp_identification(a.reps, eps, a.obs_noise, a.cost_weight)
    print("B task success...", file=sys.stderr); B = exp_tasks(a.strategies, a.seeds, a.families, eps, a.obs_noise, a.cfg, a.slip)
    for r in B: r["true_swapped"] = r["true_embodiment"].split(";")[1][-1] if "true_embodiment" in r else ""; r["true_mirror"] = r["true_embodiment"].split(";")[0][-1] if "true_embodiment" in r else ""; r["true_gain"] = r["true_embodiment"].split("mg=")[1].split(";")[0] if "true_embodiment" in r else ""
    print("C surprise monitor...", file=sys.stderr)
    drifts = [("none", {}), ("swap", {"swapped": True}), ("gain0.5", {"move_gain": 0.5})]
    C = exp_monitor((1, 2, 3, 4), (0.0, 0.05, 0.15), drifts, a.monitor_trials)
    write_csv(os.path.join(a.out, "id_accuracy.csv"), A); write_csv(os.path.join(a.out, "task_results.csv"), B); write_csv(os.path.join(a.out, "monitor_eval.csv"), C)

    L = ["# Embodiment identification: harness-level results", "",
         "**Oracle backend only.** These runs test the harness and the identification mechanics; they are not evidence about any LLM.",
         f"Settings: probe-reading flip prob = {a.obs_noise}, identifier eps = {eps}, cost weight = {a.cost_weight}, agent config = {a.cfg}, "
         f"slip = {a.slip}, tasks per cell = {a.seeds} seeds x 4 yaws x 12 hidden embodiments.", "",
         "## A. Identification (12 hidden embodiments)", "", "| method | accuracy (95% CI) | mean probes | mean bay actions | mean bay travel (m) |", "|---|---|---|---|---|"]
    for m in ID_METHODS:
        rs = [r for r in A if r["method"] == m]; k, n = sum(r["correct"] for r in rs), len(rs); lo, hi = wilson(k, n)
        L.append(f"| {m} | {k/n:.3f} ({lo:.3f}–{hi:.3f}) | {statistics.mean(r['probes'] for r in rs):.2f} | {statistics.mean(r['bay_actions'] for r in rs):.2f} | {statistics.mean(r['bay_travel'] for r in rs):.1f} |")
    L += ["", "kinematic_cal is the classical-calibration analogue: it sees actuators but has no camera-convention model, so it is wrong on every mirrored embodiment by construction.", "",
          "## B. Task success under hidden embodiment (paired worlds)", ""] + summarize_tasks(B)
    for f in ("true_mirror", "true_swapped", "true_gain", "family"):
        L += ["", f"Success by {f}:", ""] + by_factor(B, f)
    L += ["", "## C. Surprise monitor (actuation surprises only)", "",
          "False alarm = trigger before drift. Delay = steps from drift to trigger. Monitors are per channel (turn / move); 180-degree turns and zero moves are skipped as uninformative.", "",
          "| threshold | pose noise | drift | false-alarm rate | detection rate | median delay (steps) |", "|---|---|---|---|---|---|"]
    for thr, npz in itertools.product((1, 2, 3, 4), (0.0, 0.05, 0.15)):
        for dn, _ in drifts:
            rs = [r for r in C if r["threshold"] == thr and r["pose_noise"] == npz and r["drift"] == dn]
            fa = sum(r["false_alarm_before_drift"] for r in rs) / len(rs); det = sum(r["detected_after_drift"] for r in rs) / len(rs)
            dl = [r["delay_steps"] for r in rs if r["delay_steps"] != "" and not r["false_alarm_before_drift"]]
            L.append(f"| {thr} | {npz} | {dn} | {fa:.2f} | {det:.2f} | {statistics.median(dl) if dl else 'n/a'} |")
    L += ["", "## Limitations", "",
          "- Yaw is quantised to 90 degrees, so turn gain is not identifiable here; only mirror x swap x move gain {0.25, 0.5, 1.0} are in the hypothesis space.",
          "- The monitor sees only actuation (pose) surprises. A camera that becomes mirrored mid-run changes no pose and is NOT detected.",
          "- Probes run in a simulated calibration bay with a fixture at a known position; pose comes from simulator state (stand-in for odometry).",
          "- Baselines are simplified analogues of the *ideas* in EnvProbe / CoRAL / classical calibration, not reproductions.",
          "- Hypothesis set is finite and given; an embodiment outside it is not identified (it shows up as low confidence or a surprise)."]
    open(os.path.join(a.out, "REPORT.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    json.dump({"utc": datetime.now(timezone.utc).isoformat(), "args": vars(a), "backend": "OracleBackend-v1 (harness validation only)"},
              open(os.path.join(a.out, "run_metadata.json"), "w"), indent=2)
    print("\n".join(L[:40]))
if __name__ == "__main__": main()
