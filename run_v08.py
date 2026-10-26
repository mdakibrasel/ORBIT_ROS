"""v0.8 evaluation: runtime monitoring + failure recovery vs abstention (OracleBackend only; harness evidence, not LLM evidence).

  python run_v08.py --config configs/v0_8.json --out results_v08 --workers 2       (v0.8.0 protocol: 10 held-out seeds 100-109)
  python run_v08.py --config configs/v0_8_1.json --out results_v081 --workers 2    (v0.8.1 protocol: + integrity arms, fresh seeds 200-209)
  python run_v08.py --quick --out /tmp/v08q                                        (smoke test: 2 dev seeds, 3 test seeds, B=500)

Arms (all identification = hybrid, S5 agent, same tasks / hypotheses / seeds => paired):
  hybrid_old                       v0.5 verifier (reference: executes under MAP belief)
  hybrid_robust                    v0.7 strict robust verifier (abstains)          <- baseline for recovery
  hybrid_robust+monitor            + execution monitor only (stops a contradicted plan)
  hybrid_robust+recovery           + monitor + recovery ladder (reprobe -> guarded steps -> abstain)
  hybrid_robust+rec_reprobe_only   ablation: rung 1 only
  hybrid_robust+rec_guarded_only   ablation: rung 2 only
  hybrid_robust_voi (+recovery)    v0.7 VOI gate, with and without recovery
  hybrid_risk (+recovery)          v0.7 risk verifier at the v0.7 dev-selected tau, with and without recovery
  <arm>+integrity                  v0.8.1: same arm with the belief-integrity fixes of orbit_ros/integrity.py (only if the config lists it)
Conditions: static (as v0.7) and `drift`: the true move gain changes once, at a fixed tool-call count, during the episode.

v0.8.1 changes (results of the v0.8.0 config are unchanged; tests/test_integrity.py checks this against results_v08_quick):
  * arms can be declared in the config (`"arms": [...]`); without that key the v0.8.0 arm list is used;
  * `--workers N` runs (condition, arm, seed) cells in parallel; rows are re-assembled in the sequential order;
  * `--test-seeds` overrides the config's held-out seeds; `--skip-dev` skips the tau_action dev sensitivity;
  * new per-episode columns: abstained (aborted > 0), task_probes_censored, collapse_resets, fail_closed_rejects, posterior_collapsed.
"""
import argparse, itertools, json, os, statistics, sys
from datetime import datetime, timezone
from multiprocessing import Pool
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import PRESETS
from orbit_ros.agent_robust import run_episode_robust
from orbit_ros.agent_recovery import run_episode_v08, V08_KEYS
from orbit_ros.recovery import RecoveryConfig
from orbit_ros.voi import VoiStrategy
from orbit_ros.robust import RobustStrategy
from orbit_ros.integrity import IntegrityConfig, SafeRobustStrategy, SafeVoiStrategy, integrity_row_of, INTEGRITY_KEYS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment
from orbit_ros.stats07 import holm, cluster_bootstrap_diff
from run_embodiment import wilson
from run_spatial import write_csv

V080_ARMS = [("hybrid_old", "old", None), ("hybrid_robust", "robust", "off"), ("hybrid_robust+monitor", "robust", "monitor"),
             ("hybrid_robust+recovery", "robust", "full"), ("hybrid_robust+rec_reprobe_only", "robust", "reprobe_only"),
             ("hybrid_robust+rec_guarded_only", "robust", "guarded_only"), ("hybrid_robust_voi", "voi", "off"), ("hybrid_robust_voi+recovery", "voi", "full"),
             ("hybrid_risk", "risk", "off"), ("hybrid_risk+recovery", "risk", "full")]

def recovery_config(cfg, mode):
    r = cfg["recovery"]
    if mode is None: return None
    return {"off": lambda: RecoveryConfig(), "monitor": lambda: RecoveryConfig(monitor=True), "full": lambda: RecoveryConfig(enabled=True, **r),
            "reprobe_only": lambda: RecoveryConfig(enabled=True, **dict(r, guarded=False)),
            "guarded_only": lambda: RecoveryConfig(enabled=True, **dict(r, extra_probes=0))}[mode]()

def arms(cfg):
    """v0.8.0 arm list, or the config's `arms` = [{"arm", "kind", "recovery", "integrity"}] (recovery: off|monitor|full|reprobe_only|guarded_only)."""
    spec = cfg.get("arms") or [dict(arm=a, kind=k, recovery=m, integrity=False) for a, k, m in V080_ARMS]
    out = []
    for s in spec:
        if s["kind"] not in ("old", "robust", "voi", "risk"): raise ValueError(s)
        if s["kind"] == "old" and s.get("recovery") not in (None, "off"): raise ValueError("the old verifier runs on the v0.6 runner (no recovery)")
        out.append(dict(arm=s["arm"], kind=s["kind"], rcfg=recovery_config(cfg, None if s["kind"] == "old" else (s.get("recovery") or "off")), integrity=bool(s.get("integrity"))))
    names = [a["arm"] for a in out]
    if len(set(names)) != len(names): raise ValueError("duplicate arm names")
    return out

def make_strategy(cfg, kind, noise, seed, integrity=False):
    common = dict(eps=noise, noise_p=noise, seed=seed, stop=cfg["stop_confidence"], max_task_probes=cfg["max_task_probes"], probe_len=cfg["probe_len"],
                  hybrid_names=tuple(cfg["hybrid_bay_probes"]), hybrid_probes=cfg["hybrid_bay_budget"])
    R, V = (SafeRobustStrategy, SafeVoiStrategy) if integrity else (RobustStrategy, VoiStrategy)
    if integrity: common["integrity"] = IntegrityConfig(**cfg.get("integrity", {}))
    if kind == "old": return R("hybrid", "old", **common)
    if kind == "robust": return V("hybrid", voi=False, **common)          # == RobustStrategy robust verifier (checked in tests)
    if kind == "voi": return V("hybrid", voi=True, **cfg["voi"], **common)
    if kind == "risk": return V("hybrid", voi=False, tau=cfg["risk_tau"], **common)
    raise ValueError(kind)

def run_cell(cfg, arm, cond, seeds):
    rows, noise = [], cond["noise"]
    for fam, yaw, s, h in itertools.product(cfg["families"], cfg["yaws"], seeds, hypothesis_space()):
        world, task = make_task(fam, s, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1)
        apply_embodiment(world, h)
        if cond["drift_at_call"] is not None:
            world.drift_schedule.append((cond["drift_at_call"], {"move_gain": 0.5 if world.move_gain != 0.5 else 1.0}))
        st, agent, be = make_strategy(cfg, arm["kind"], noise, s, arm.get("integrity", False)), PRESETS[cfg["agent_config"]], OracleBackend(cfg["slip"], 0.0)
        row = run_episode_robust(world, task, agent, be, st) if arm["kind"] == "old" else run_episode_v08(world, task, agent, be, st, arm["rcfg"])
        t = row["true_embodiment"]
        row.update(arm=arm["arm"], condition=cond["name"], noise=noise, seed=s, yaw=yaw, cluster=f"{fam}|{yaw}|{s}",
                   total_travel=float(row["id_bay_travel"]) + float(row["task_probe_travel"]), episode_with_collision=int(float(row["collisions"]) > 0),
                   drift_applied=int(cond["drift_at_call"] is not None and not world.drift_schedule), true_gain=t.split("mg=")[1].split(";")[0],
                   coll_per100=100.0 * float(row["collisions"]) / max(1, int(row["actions_executed"])))
        for k in V08_KEYS: row.setdefault(k, 0)
        row.update(abstained=int(float(row["aborted"]) > 0), **integrity_row_of(st))                                   # V081
        rows.append(row)
    return rows

def _job(args):
    cfg, arm, cond, seeds = args
    return run_cell(cfg, arm, cond, seeds)

def run_cells(cfg, jobs, workers):
    """jobs = [(arm, cond, seeds)] -> rows in exactly the order a sequential run_cell loop over the jobs would produce."""
    fams, yaws, H = cfg["families"], cfg["yaws"], [h.key for h in hypothesis_space()]
    split = [(j, arm, cond, s) for j, (arm, cond, seeds) in enumerate(jobs) for s in seeds]
    if workers <= 1: parts = [_job((cfg, arm, cond, [s])) for _, arm, cond, s in split]
    else:
        with Pool(workers) as p: parts = p.map(_job, [(cfg, arm, cond, [s]) for _, arm, cond, s in split], chunksize=1)
    out = []
    for j, (arm, cond, seeds) in enumerate(jobs):
        rows = [r for (jj, _, _, _), part in zip(split, parts) if jj == j for r in part]
        order = {(f, y, s, h): i for i, (f, y, s, h) in enumerate(itertools.product(fams, yaws, seeds, H))}
        rows.sort(key=lambda r: order[(r["family"], r["yaw"], r["seed"], r["true_embodiment"])])
        out += rows
    return out

def mean(rs, k): return statistics.mean(float(r.get(k) or 0) for r in rs) if rs else float("nan")
def summ(rs):
    n, k = len(rs), sum(int(r["success"]) for r in rs); lo, hi = wilson(k, n)
    acts, col = sum(int(r["actions_executed"]) for r in rs), sum(float(r["collisions"]) for r in rs)
    ent = [r for r in rs if float(r["rec_entries"]) > 0]
    return {"n": n, "success": k / n, "ci_lo": lo, "ci_hi": hi, "abstain": sum(1 for r in rs if float(r["aborted"]) > 0) / n, "collisions": col / n,
            "ep_collision": mean(rs, "episode_with_collision"), "coll_per100_actions": 100 * col / max(1, acts),
            "coll_per_completed": col / max(1, k), "llm_calls": mean(rs, "llm_calls"), "env_calls": mean(rs, "env_tool_calls"), "actions": mean(rs, "actions_executed"),
            "rec_episodes": len(ent) / n, "rec_success_given_entry": (sum(int(r["success"]) for r in ent) / len(ent)) if ent else float("nan"),
            "rec_phase_resolved": (sum(float(r["rec_resolved_phases"]) for r in ent) / max(1, sum(float(r["rec_entered_phases"]) for r in ent) )) if ent else float("nan"),
            "rec_collisions": mean(rs, "rec_collisions"), "rec_actions": mean(ent, "rec_actions"), "rec_llm_calls": mean(ent, "rec_llm_calls"), "rec_env_calls": mean(ent, "rec_env_calls"),
            "monitor_trips": mean(rs, "monitor_trips"), "guarded_creeps": mean(rs, "guarded_creeps"), "guarded_unsafe": mean(rs, "guarded_unsafe"),
            # V081 columns (appended so that the v0.8.0 columns keep their order)
            "success_zero_coll": sum(1 for r in rs if int(r["success"]) == 1 and float(r["collisions"]) == 0) / n,
            "aborted_phases": mean(rs, "aborted"), "drift_applied": mean(rs, "drift_applied"), "belief_correct": mean(rs, "belief_correct"),
            "posterior_collapsed": mean(rs, "posterior_collapsed"), "probes_censored": mean(rs, "task_probes_censored"), "collapse_resets": mean(rs, "collapse_resets")}

def parse_seeds(s):
    out = []
    for part in s.split(","):
        if "-" in part: a, b = part.split("-"); out += list(range(int(a), int(b) + 1))
        elif part: out.append(int(part))
    return out

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--config", default="configs/v0_8.json"); ap.add_argument("--out", default="results_v08")
    ap.add_argument("--quick", action="store_true", help="2 dev seeds, 3 test seeds, small bootstrap (smoke test)")
    ap.add_argument("--only", default="", help="comma-separated condition names to run")
    ap.add_argument("--workers", type=int, default=1, help="parallel processes (results are identical to --workers 1)")
    ap.add_argument("--test-seeds", default="", help="override the config's held-out seeds, e.g. 100-109 or 100,101")
    ap.add_argument("--skip-dev", action="store_true", help="skip the tau_action dev-seed sensitivity")
    a = ap.parse_args(); cfg = json.load(open(a.config)); os.makedirs(a.out, exist_ok=True)
    if a.test_seeds: cfg["test_seeds"] = parse_seeds(a.test_seeds); cfg["test_seeds_overridden"] = True
    if a.quick: cfg["dev_seeds"], cfg["test_seeds"], cfg["bootstrap_B"] = cfg["dev_seeds"][:2], cfg["test_seeds"][:3], 500
    if a.skip_dev: cfg["tau_action_sensitivity_dev"] = []
    conds = [c for c in cfg["conditions"] if not a.only or c["name"] in a.only.split(",")]
    A = arms(cfg)
    for cond in conds:
        for arm in A: print(f"test {cond['name']} {arm['arm']}", file=sys.stderr, flush=True)
    test = run_cells(cfg, [(arm, cond, cfg["test_seeds"]) for cond in conds for arm in A], a.workers)
    # dev-seed sensitivity of tau_action (reported, not tuned on)
    base = next(x for x in A if x["arm"] == cfg.get("sensitivity_arm", "hybrid_robust+recovery"))
    sjobs = [(ta, cond) for ta in cfg["tau_action_sensitivity_dev"] for cond in conds if cond["noise"] > 0]
    sarms = [dict(base, rcfg=RecoveryConfig(enabled=True, **dict(cfg["recovery"], tau_action=ta))) for ta, _ in sjobs]
    srows = run_cells(cfg, [(arm, cond, cfg["dev_seeds"]) for arm, (_, cond) in zip(sarms, sjobs)], a.workers) if sjobs else []
    sens, per = [], len(cfg["families"]) * len(cfg["yaws"]) * len(cfg["dev_seeds"]) * len(hypothesis_space())
    for i, (ta, cond) in enumerate(sjobs): sens.append(dict(tau_action=ta, condition=cond["name"], **summ(srows[i * per:(i + 1) * per])))
    stats = []
    for cond in conds:
        fam = []
        for Aa, Bb in cfg["primary_comparisons"]:
            ra = sorted([r for r in test if r["arm"] == Aa and r["condition"] == cond["name"]], key=lambda r: (r["task_id"], r["true_embodiment"], r["seed"]))
            rb = sorted([r for r in test if r["arm"] == Bb and r["condition"] == cond["name"]], key=lambda r: (r["task_id"], r["true_embodiment"], r["seed"]))
            assert [(r["task_id"], r["true_embodiment"]) for r in ra] == [(r["task_id"], r["true_embodiment"]) for r in rb]
            for m in cfg["primary_metrics"]:
                d, lo, hi, p = cluster_bootstrap_diff([float(r.get(m) or 0) for r in ra], [float(r.get(m) or 0) for r in rb], [r["cluster"] for r in ra], cfg["bootstrap_B"])
                fam.append(dict(condition=cond["name"], A=Aa, B=Bb, metric=m, mean_A=mean(ra, m), mean_B=mean(rb, m), diff=d, ci_lo=lo, ci_hi=hi, p=p))
        for r, adj in zip(fam, holm([f["p"] for f in fam])): r["p_holm"] = adj
        stats += fam
    write_csv(os.path.join(a.out, "test_results.csv"), test); write_csv(os.path.join(a.out, "stats.csv"), stats); write_csv(os.path.join(a.out, "tau_action_sensitivity_dev.csv"), sens)
    summary = [dict(condition=c["name"], arm=x["arm"], **summ([r for r in test if r["arm"] == x["arm"] and r["condition"] == c["name"]])) for c in conds for x in A]
    write_csv(os.path.join(a.out, "summary.csv"), summary)
    json.dump({"utc": datetime.now(timezone.utc).isoformat(), "config": cfg, "config_file": os.path.basename(a.config), "workers": a.workers,
               "backend": "OracleBackend-v1 (harness validation only)"}, open(os.path.join(a.out, "run_metadata.json"), "w"), indent=2, default=str)
    print("done", len(test), "episodes", file=sys.stderr)

if __name__ == "__main__": main()
