"""Paired arm comparisons inside ONE run (v0.14 milestone M7), as pre-registered in docs/PEL_PROTOCOL.md.

    python tools/pel_arms.py results_v014_heldout --out results_v014_heldout/PEL_ARMS.md

Every contrast pairs the two arms on the same (task_id, repeat), so both saw the identical world. For success: paired risk difference
in percentage points with a 95% interval (Agresti-Min, as compare_runs.py), exact McNemar p, Holm correction over the pre-registered
family of contrasts. For counts (safety violations, invalid calls, planner calls per episode): paired mean difference with a 95%
interval (normal approximation; t-quantile not needed at n >= 100) and a Wilcoxon signed-rank p. Episodes with an infra_error or
cost_cap_reached outcome in either arm are excluded from that contrast and counted.
"""
from __future__ import annotations
import argparse, csv, json, math, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from analyze_spatial import mcnemar_exact, holm, wilcoxon_signed_rank, FAM_ORDER
from compare_runs import paired_diff_ci

PRIMARY = [("C4", "PEL"), ("C5", "PEL")]
SECONDARY = [("C1", "PEL"), ("C2", "PEL"), ("C3", "PEL"), ("S1", "PEL")]
ABLATIONS = [("PEL", "PEL-registry"), ("PEL", "PEL-guard"), ("PEL", "PEL-verify"), ("PEL", "PEL-replan")]
COUNTS = (("safety_violations", "safety violations / episode"), ("invalid_calls", "invalid calls / episode"), ("llm_calls", "planner calls / episode"))
EXCLUDE = ("infra_error", "cost_cap_reached")

def load(run_dir):
    with open(os.path.join(run_dir, "trial_results.csv"), encoding="utf-8", newline="") as f: rows = list(csv.DictReader(f))
    meta = {}
    p = os.path.join(run_dir, "run_metadata.json")
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f: meta = json.load(f)
    return {(r["task_id"], r["config"], r.get("rep") or "0"): r for r in rows}, meta

def pairs(R, a, b, family=None):
    ka = {(k[0], k[2]) for k in R if k[1] == a}; kb = {(k[0], k[2]) for k in R if k[1] == b}
    both = sorted(ka & kb); out, excl = [], 0
    for t, rep in both:
        ra, rb = R[(t, a, rep)], R[(t, b, rep)]
        if family and ra["family"] != family: continue
        if ra.get("outcome_code") in EXCLUDE or rb.get("outcome_code") in EXCLUDE or ra.get("error") or rb.get("error"): excl += 1; continue
        out.append((ra, rb))
    return out, excl, len(ka ^ kb)

def num(v):
    try: return float(v)
    except (TypeError, ValueError): return 0.0

def mean_ci(d, z=1.96):
    if not d: return 0.0, 0.0, 0.0
    m = st.fmean(d); se = st.stdev(d) / math.sqrt(len(d)) if len(d) > 1 else 0.0
    return m, m - z * se, m + z * se

def contrast(R, a, b, family=None):
    pr, excl, unpaired = pairs(R, a, b, family)
    sa = [int(x["success"]) for x, _ in pr]; sb = [int(y["success"]) for _, y in pr]
    d, lo, hi = paired_diff_ci(sa, sb)
    p = mcnemar_exact(sum(1 for x, y in zip(sa, sb) if not x and y), sum(1 for x, y in zip(sa, sb) if x and not y))
    counts = {}
    for col, _ in COUNTS:
        diffs = [num(y.get(col)) - num(x.get(col)) for x, y in pr]
        m, l, h = mean_ci(diffs); _, _, pw = wilcoxon_signed_rank(diffs) if any(diffs) else (0, 0, 1.0)
        counts[col] = {"a": st.fmean([num(x.get(col)) for x, _ in pr]) if pr else 0.0, "b": st.fmean([num(y.get(col)) for _, y in pr]) if pr else 0.0,
                       "diff": m, "lo": l, "hi": h, "p": pw}
    return {"a": a, "b": b, "n": len(pr), "excluded": excl, "unpaired": unpaired, "succ_a": sum(sa), "succ_b": sum(sb),
            "diff_pp": d, "lo": lo, "hi": hi, "p": p, "counts": counts}

def report(run_dir):
    R, meta = load(run_dir)
    cfgs = {k[1] for k in R}
    kind = "SYNTHETIC (oracle planner)" if "Oracle" in str(meta.get("model", "Oracle")) else f"REAL-LLM ({meta.get('model')})"
    L = [f"# Paired arm comparisons: `{os.path.basename(os.path.abspath(run_dir))}`", "",
         f"Run type: **{kind}**. Held-out seeds: {meta.get('heldout_seeds')}. Instances: {meta.get('instances')}, seeds per cell: {meta.get('seeds_per_cell')}, "
         f"repeats: {meta.get('repeats')}. Protocol: docs/PEL_PROTOCOL.md (status: hash recorded, text not preserved; see baselines/m7/PROTOCOL_HASH_STATUS.md).", ""]
    out = {}
    for title, fam in (("Primary contrasts (Holm over this family)", PRIMARY), ("Secondary contrasts (Holm over this family)", SECONDARY),
                       ("Component ablations: full PEL vs one component off (Holm over this family)", ABLATIONS)):
        cs = [contrast(R, a, b) for a, b in fam if a in cfgs and b in cfgs]
        if not cs: continue
        adj = holm([c["p"] for c in cs])
        L += [f"## {title}", "", "| A | B | paired n | excluded | A success | B success | B - A (pp) [95% CI] | McNemar p | p (Holm) |", "|---|---|---|---|---|---|---|---|---|"]
        for c, pa in zip(cs, adj):
            c["p_holm"] = pa; out[f"{c['a']}->{c['b']}"] = c
            L.append(f"| {c['a']} | {c['b']} | {c['n']} | {c['excluded']} | {100 * c['succ_a'] / max(1, c['n']):.1f}% | {100 * c['succ_b'] / max(1, c['n']):.1f}% | "
                     f"{c['diff_pp']:+.1f} [{c['lo']:+.1f}, {c['hi']:+.1f}] | {c['p']:.2g} | {pa:.2g} |")
        for col, _ in COUNTS:              # v0.14.1 (review M6): count contrasts are Holm-corrected within the family too (protocol §6/§8)
            for c, pa in zip(cs, holm([c["counts"][col]["p"] for c in cs])): c["counts"][col]["p_holm"] = pa
        L += ["", "| A | B | measure | A mean | B mean | B - A [95% CI] | Wilcoxon p | p (Holm) |", "|---|---|---|---|---|---|---|---|"]
        for c in cs:
            for col, label in COUNTS:
                k = c["counts"][col]
                L.append(f"| {c['a']} | {c['b']} | {label} | {k['a']:.2f} | {k['b']:.2f} | {k['diff']:+.2f} [{k['lo']:+.2f}, {k['hi']:+.2f}] | {k['p']:.2g} | {k['p_holm']:.2g} |")
        L.append("")
    if "PEL" in cfgs and "C4" in cfgs:
        L += ["## Per family: PEL vs C4 (descriptive, no correction)", "", "| family | n | C4 success | PEL success | PEL - C4 (pp) [95% CI] |", "|---|---|---|---|---|"]
        for f in FAM_ORDER:
            c = contrast(R, "C4", "PEL", f)
            if c["n"]: L.append(f"| {f} | {c['n']} | {100 * c['succ_a'] / c['n']:.1f}% | {100 * c['succ_b'] / c['n']:.1f}% | {c['diff_pp']:+.1f} [{c['lo']:+.1f}, {c['hi']:+.1f}] |")
        L.append("")
    L += ["Holm correction is applied within each family, separately for success (McNemar) and for each count measure (Wilcoxon). "
          "The 95% intervals are NOT adjusted for multiplicity, so an interval can exclude 0 while the Holm p is above 0.05.",
          "llm_calls for an oracle run counts oracle decisions (planner calls the arm would make), not API calls; there is no token or money cost.",
          "A significant difference in a SYNTHETIC run says how the arms differ under the oracle's assumed error model, not how a real model behaves."]
    return "\n".join(L) + "\n", out

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0]); ap.add_argument("run_dir"); ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    text, out = report(a.run_dir)
    path = a.out or os.path.join(a.run_dir, "PEL_ARMS.md")
    with open(path, "w", encoding="utf-8") as f: f.write(text)
    with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8") as f: json.dump(out, f, indent=1)
    print(text); print("wrote", path)

if __name__ == "__main__": main()
