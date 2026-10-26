r"""Consolidate all Phase 9 live trace files into one authoritative dataset + analysis.

Reads every results_nao_benchmark/phase9_live_*/**/*.trace.json, keeps the MOST RECENT trial per (scenario, strategy)
(by file mtime), and writes a consolidated directory with trials.jsonl, trials.csv, summary.json and comparison.md.
Only real live traces are included (each trace carries connected=True and a true start/final pose).

    .venv\Scripts\python.exe tools\phase9_consolidate.py
"""
from __future__ import annotations
import csv, glob, json, os, sys, time, datetime
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results_nao_benchmark")

def load_latest():
    best = {}                                                       # (scenario, strategy) -> (mtime, trace, path)
    for p in glob.glob(os.path.join(RES, "phase9_live_*", "*.trace.json")):
        try:
            tr = json.load(open(p, encoding="utf-8"))
        except Exception: continue
        key = (tr.get("scenario"), tr.get("strategy"))
        mt = os.path.getmtime(p)
        if key[0] is None or key[1] is None: continue
        if key not in best or mt > best[key][0]: best[key] = (mt, tr, p)
    return {k: v for k, v in best.items()}

def row(tr):
    tok = tr.get("tokens") or {}
    return {"scenario": tr.get("scenario"), "strategy": tr.get("strategy"),
            "success": tr.get("success"), "actions": tr.get("actions"), "replans": tr.get("replans"),
            "vetoes": tr.get("vetoes"), "failed_actions": tr.get("failed_actions"),
            "llm_calls": tr.get("llm_calls"), "llm_latency_s": round(tr.get("llm_latency_s", 0) or 0, 2),
            "total_tokens": tok.get("total_tokens"), "turn_error_deg": tr.get("turn_error_deg"),
            "heading_drift_deg": tr.get("heading_drift_deg"), "collisions": tr.get("collisions"),
            "wall_s": tr.get("wall_s"),
            "start": _p(tr.get("start_pose")), "final": _p(tr.get("final_pose")),
            "note": tr.get("skipped") or tr.get("exception") or tr.get("llm_error") or ""}

def _p(d):
    if not d: return ""
    return f"({d.get('x')},{d.get('y')},{d.get('yaw_deg')})"

def main():
    best = load_latest()
    if not best: print("no phase9 traces found"); return 1
    out = os.path.join(RES, "phase9_consolidated_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(out, exist_ok=True)
    traces = [v[1] for v in best.values()]
    with open(os.path.join(out, "trials.jsonl"), "w", encoding="utf-8") as f:
        for _, tr, _p2 in sorted(best.values(), key=lambda v: (v[1]["scenario"], v[1]["strategy"])):
            f.write(json.dumps(tr, default=str) + "\n")
    cols = ["scenario", "strategy", "success", "actions", "replans", "vetoes", "failed_actions",
            "llm_calls", "llm_latency_s", "total_tokens", "turn_error_deg", "heading_drift_deg", "collisions", "wall_s", "start", "final", "note"]
    rows = [row(tr) for _, tr, _p2 in sorted(best.values(), key=lambda v: (v[1]["scenario"], v[1]["strategy"]))]
    with open(os.path.join(out, "trials.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(rows)
    # summary per strategy
    summary = {"generated_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
               "total_trials": len(traces), "source_dirs": sorted({os.path.basename(os.path.dirname(v[2])) for v in best.values()}),
               "by_strategy": {}}
    for strat in ("A", "B", "C"):
        ts = [tr for tr in traces if tr.get("strategy") == strat]
        judged = [tr for tr in ts if tr.get("success") is not None]
        ok = sum(1 for tr in judged if tr.get("success") is True)
        lat = [tr["llm_latency_s"] for tr in ts if tr.get("llm_latency_s")]
        toks = [tr.get("tokens", {}).get("total_tokens") for tr in ts if (tr.get("tokens") or {}).get("total_tokens")]
        summary["by_strategy"][strat] = {
            "trials": len(ts), "judged": len(judged), "skipped": len(ts) - len(judged),
            "successes": ok, "success_rate_pct": round(100 * ok / len(judged), 1) if judged else None,
            "success_rate_fraction": f"{ok}/{len(judged)}" if judged else "0/0",
            "total_llm_calls": sum(tr.get("llm_calls", 0) for tr in ts),
            "llm_mean_latency_s": round(sum(lat) / len(lat), 2) if lat else None,
            "llm_total_tokens": sum(toks) if toks else None,
            "total_vetoes": sum(tr.get("vetoes", 0) for tr in ts), "total_collisions": sum(tr.get("collisions", 0) or 0 for tr in ts)}
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f: json.dump(summary, f, indent=2)
    # per-scenario comparison matrix
    scen_order = [r["scenario"] for r in rows]
    scenarios = sorted(set(scen_order))
    lines = ["# Phase 9 consolidated comparison (latest live trial per scenario x strategy)", "",
             f"Total trials: {len(traces)}  |  Source run dirs: {len(summary['source_dirs'])}", "",
             "Legend: PASS / FAIL / SKIP (SKIP = strategy not applicable, e.g. open-loop has no script for an abstract instruction).", "",
             "| Scenario | A open-loop | B planner | C LLM (groq) |", "|---|---|---|---|"]
    def cell(sc, st):
        tr = best.get((sc, st))
        if not tr: return "-"
        s = tr[1].get("success"); mark = "PASS" if s is True else ("SKIP" if s is None else "FAIL")
        extra = ""
        if st == "C" and tr[1].get("llm_calls"): extra = f" ({tr[1]['llm_calls']} calls)"
        return mark + extra
    for sc in scenarios:
        lines.append(f"| {sc} | {cell(sc,'A')} | {cell(sc,'B')} | {cell(sc,'C')} |")
    lines += ["", "## Success rate by strategy", "", "| Strategy | Success | Rate |", "|---|---|---|"]
    for strat in ("A", "B", "C"):
        b = summary["by_strategy"][strat]
        lines.append(f"| {strat} | {b['success_rate_fraction']} | {b['success_rate_pct']}% |")
    open(os.path.join(out, "comparison.md"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines)); print("\n", out)
    print("\n", json.dumps(summary["by_strategy"], indent=2))
    return 0

if __name__ == "__main__": sys.exit(main())
