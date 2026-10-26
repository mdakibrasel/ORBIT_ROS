"""Cost-effectiveness report for one run folder (v0.14 milestone M6).

    python tools/cost_report.py results_pilot_gemini --baseline C4 [--pricing configs/pricing.json] [--out cost_report.md]

Reads trial_results.csv (one row per episode) and, for real-model runs, api_usage.csv (one row per model call). Money comes ONLY from
configs/pricing.json (see orbit_ros/pel/cost.py); a model without a price gets "unknown", never a guessed number. Oracle runs have no
api_usage.csv: they are SYNTHETIC and have no money cost (their llm_calls count oracle decisions, not API calls).

Per configuration: attempted episodes, infrastructure errors (provider / runner failures, reported separately and excluded from task
success), successes, success rate, model calls, tokens (input, cached, output, reasoning), cost total, cost per attempted task, cost per
successful task (undefined when there is no success), mean model latency per call, mean episode time, invalid calls, re-plans,
guard rejections (registry rejections + guard vetoes, PEL only), safety violations. Against --baseline: success difference in
percentage points and incremental cost per percentage point (undefined unless the success gain is > 0).
"""
from __future__ import annotations
import argparse, csv, json, os, sys
from collections import defaultdict
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orbit_ros.pel.cost import load_pricing, price_for, call_cost, default_pricing_path

def _rows(path):
    if not os.path.exists(path): return []
    with open(path, encoding="utf-8", newline="") as f: return list(csv.DictReader(f))

def _int(v):
    try: return int(float(v))
    except (TypeError, ValueError): return 0

def _float(v):
    try: return float(v)
    except (TypeError, ValueError): return None

def _usage(u):
    """api_usage.csv row -> the dict call_cost expects (empty cells = not reported)."""
    g = lambda k: int(float(u[k])) if u.get(k) not in (None, "") else None
    return {"input_tokens": g("input_tokens"), "output_tokens": g("output_tokens"), "cached_tokens": g("cached_tokens"), "reasoning_tokens": g("reasoning_tokens")}

def summarise(run_dir, pricing=None):
    trials, usage = _rows(os.path.join(run_dir, "trial_results.csv")), _rows(os.path.join(run_dir, "api_usage.csv"))
    meta = {}
    mp = os.path.join(run_dir, "run_metadata.json")
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as f: meta = json.load(f)
    real = bool(usage)
    by_cfg = defaultdict(list); calls_by_cfg = defaultdict(list)
    for r in trials: by_cfg[r["config"]].append(r)
    for u in usage: calls_by_cfg[u["config"]].append(u)
    out = {}
    for cfg, rs in by_cfg.items():
        infra = [r for r in rs if r.get("outcome_code") == "infra_error" or r.get("error")]
        ok_rows = [r for r in rs if r not in infra]
        n, succ = len(ok_rows), sum(_int(r["success"]) for r in ok_rows)
        calls = calls_by_cfg.get(cfg, [])
        tok = {k: sum(_int(c.get(k)) for c in calls) for k in ("input_tokens", "cached_tokens", "output_tokens", "reasoning_tokens")}
        cost, unknown = 0.0, []
        for c in calls:
            p = price_for(pricing, c.get("provider", ""), c.get("model_id", ""))
            v = call_cost(_usage(c), p) if p else None
            if v is None: unknown.append(c.get("model_id", "?"))
            else: cost += v
        priced = real and not unknown
        lat = [x for x in (_float(c.get("latency_s")) for c in calls) if x is not None]
        el = [x for x in (_float(r.get("elapsed_s")) for r in ok_rows) if x is not None]
        d = {"attempted": n, "infra_errors": len(infra), "successes": succ, "success_rate": succ / n if n else None,
             "model_calls": len(calls) if real else None, "oracle_decisions": None if real else sum(_int(r.get("llm_calls")) for r in ok_rows),
             **({k: v for k, v in tok.items()} if real else {k: None for k in tok}),
             "provider_retries": sum(max(0, _int(c.get("attempts")) - 1) for c in calls) if real else None,
             "cost_total": cost if priced else None, "unpriced_models": sorted(set(unknown)),
             "cost_per_attempted": (cost / (n + len(infra))) if priced and (n + len(infra)) else None,
             "cost_per_success": (cost / succ) if priced and succ else None,
             "mean_call_latency_s": sum(lat) / len(lat) if lat else None, "mean_episode_s": sum(el) / len(el) if el else None,
             "invalid_calls": sum(_int(r.get("invalid_calls")) for r in ok_rows), "replans": sum(_int(r.get("replans")) + _int(r.get("pel_replans")) + _int(r.get("feedback_replans")) for r in ok_rows),
             "guard_rejections": sum(_int(r.get("pel_rejections")) + _int(r.get("pel_vetoes")) for r in ok_rows),
             "safety_violations": sum(_int(r.get("safety_violations")) for r in ok_rows),
             "recovered": sum(1 for r in ok_rows if _int(r["success"]) and (_int(r.get("pel_replans")) or _int(r.get("repairs")) or _int(r.get("feedback_replans")))),
             "per_task": {k: (v / n if n else None) for k, v in (("calls", len(calls) if real else sum(_int(r.get("llm_calls")) for r in ok_rows)),
                                                                  ("invalid_calls", sum(_int(r.get("invalid_calls")) for r in ok_rows)))}}
        out[cfg] = d
    return {"real": real, "meta": meta, "configs": out}

def incremental(base, arm):
    """Incremental cost per percentage point of success: only when both costs are known and the arm succeeds more often."""
    if base["success_rate"] is None or arm["success_rate"] is None: return None, "no episodes"
    dpp = 100 * (arm["success_rate"] - base["success_rate"])
    if base["cost_per_attempted"] is None or arm["cost_per_attempted"] is None: return dpp, "cost unknown"
    if dpp <= 0: return dpp, "undefined (no success gain)"
    return dpp, (arm["cost_per_attempted"] - base["cost_per_attempted"]) / dpp

def saving_pct(base, arm, key):
    """How much cheaper the arm is than the baseline, in percent of the baseline's cost (negative = more expensive)."""
    b, a = base.get(key), arm.get(key)
    if b is None or a is None or b <= 0: return None
    return 100 * (b - a) / b

def fmt(v, d=4):
    if v is None: return "n/a"
    if isinstance(v, float): return f"{v:.{d}f}"
    return str(v)

def report(run_dir, pricing_path=None, baseline=None):
    pricing = load_pricing(pricing_path) if pricing_path and os.path.exists(pricing_path) else None
    s = summarise(run_dir, pricing)
    cur = pricing["currency"] if pricing else "?"
    kind = "REAL-LLM" if s["real"] else "SYNTHETIC (oracle; no API calls, no money)"
    L = [f"# Cost-effectiveness report: {os.path.basename(os.path.abspath(run_dir))}", "",
         f"Run type: **{kind}**. Model: {s['meta'].get('model', '?')}. Money: computed from recorded token usage and "
         f"`{pricing_path}` ({cur}, paid-tier prices; an estimate, not an invoice).", ""]
    hdr = ["config", "attempted", "infra errors", "successes", "success %", "calls" if s["real"] else "oracle decisions", "input tok", "cached tok",
           "output tok", "reasoning tok", "provider retries", f"cost ({cur})", "cost/attempted", "cost/success", "mean call latency s",
           "mean episode s", "invalid calls", "re-plans", "guard rejections", "safety violations", "recovered successes"]
    L += ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for cfg, d in s["configs"].items():
        L.append("| " + " | ".join([cfg, fmt(d["attempted"]), fmt(d["infra_errors"]), fmt(d["successes"]),
                                    fmt(100 * d["success_rate"], 1) if d["success_rate"] is not None else "n/a",
                                    fmt(d["model_calls"] if s["real"] else d["oracle_decisions"]), fmt(d["input_tokens"]), fmt(d["cached_tokens"]),
                                    fmt(d["output_tokens"]), fmt(d["reasoning_tokens"]), fmt(d["provider_retries"]), fmt(d["cost_total"], 6),
                                    fmt(d["cost_per_attempted"], 6), fmt(d["cost_per_success"], 6) if d["successes"] else "undefined (0 successes)",
                                    fmt(d["mean_call_latency_s"], 3), fmt(d["mean_episode_s"], 3), fmt(d["invalid_calls"]), fmt(d["replans"]),
                                    fmt(d["guard_rejections"]), fmt(d["safety_violations"]), fmt(d["recovered"])]) + " |")
    if baseline and baseline in s["configs"]:
        L += ["", f"Against baseline **{baseline}** (incremental cost per percentage point of success = extra cost per attempted task / success gain; "
              "cost saving = how much cheaper than the baseline, in percent of the baseline's cost; negative = more expensive):", "",
              "| config | success difference (pp) | incremental cost per pp | cost saving per attempted task | cost saving per successful task |",
              "|---|---|---|---|---|"]
        for cfg, d in s["configs"].items():
            if cfg == baseline: continue
            dpp, inc = incremental(s["configs"][baseline], d)
            sa, ss = saving_pct(s["configs"][baseline], d, "cost_per_attempted"), saving_pct(s["configs"][baseline], d, "cost_per_success")
            L.append(f"| {cfg} | {fmt(dpp, 1)} | {fmt(inc, 6) if isinstance(inc, float) else inc} | "
                     f"{fmt(sa, 1) + ' %' if sa is not None else 'n/a'} | {fmt(ss, 1) + ' %' if ss is not None else 'n/a'} |")
    unp = sorted({m for d in s["configs"].values() for m in d["unpriced_models"]})
    if unp: L += ["", f"No price in the pricing file for: {', '.join(unp)}. Their cost is unknown (n/a), not zero."]
    L += ["", "Infrastructure errors (provider or runner failures) are excluded from attempted/success counts and listed separately; "
          "cost per attempted task divides by all episodes including them, because their calls were paid."]
    return "\n".join(L) + "\n", s

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir"); ap.add_argument("--pricing", default=default_pricing_path()); ap.add_argument("--baseline", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    text, _ = report(a.run_dir, a.pricing, a.baseline)
    out = a.out or os.path.join(a.run_dir, "cost_report.md")
    with open(out, "w", encoding="utf-8") as f: f.write(text)
    print(text); print("wrote", out)

if __name__ == "__main__": main()
