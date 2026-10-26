"""Paired comparison of two runs on the same task instances (same task_id, configuration and repeat).
  python compare_runs.py results_A results_B --out COMPARE_A_vs_B.md

Use it for every one-switch experiment: --nav-tools (coverage planner tool vs LLM sweeps), --hierarchical, --phrasing paraphrase|bn,
--no-sensor-doc, --no-interp-view, --window N, two models, or two repeats of the same run (run-to-run variance)."""
import argparse, csv, json, math, os, statistics as st
from analyze_spatial import mcnemar_exact, holm, fnum, FAM_ORDER, cfg_key

def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, "trial_results.csv"), encoding="utf-8")))
    meta = json.load(open(os.path.join(d, "run_metadata.json")))
    return {(r["task_id"], r["config"], r.get("rep") or "0"): r for r in rows}, meta

def paired_diff_ci(sa, sb, z=1.96):
    """Paired risk difference B - A in percentage points with a 95% Wald interval after Agresti-Min (0.5 added to each of the
    four cells of the paired 2x2 table). Returns (diff, lo, hi)."""
    n11 = sum(a and b for a, b in zip(sa, sb)); n10 = sum(a and not b for a, b in zip(sa, sb))
    n01 = sum(b and not a for a, b in zip(sa, sb)); n00 = len(sa) - n11 - n10 - n01
    d = (n01 - n10) / len(sa) if sa else 0.0
    m = n11 + n10 + n01 + n00 + 2.0; p10, p01 = (n10 + 0.5) / m, (n01 + 0.5) / m
    da = p01 - p10; se = math.sqrt(max(0.0, (p10 + p01 - da * da) / m))
    return 100 * d, 100 * max(-1.0, da - z * se), 100 * min(1.0, da + z * se)

def main():
    p = argparse.ArgumentParser(); p.add_argument("a"); p.add_argument("b"); p.add_argument("--out", default=None)
    x = p.parse_args()
    A, ma = load(x.a); B, mb = load(x.b); keys = sorted(set(A) & set(B))
    diff = {k: (ma.get(k), mb.get(k)) for k in sorted(set(ma) | set(mb)) if ma.get(k) != mb.get(k) and k not in ("utc", "wall_clock_s_this_invocation", "episodes_written", "model_calls_this_invocation", "python", "platform")}
    L = [f"# Paired comparison: `{x.a}` (A) vs `{x.b}` (B)\n", f"Paired episodes: {len(keys)} (only in A: {len(set(A) - set(B))}, only in B: {len(set(B) - set(A))}).\n",
         "Settings that differ: " + ("; ".join(f"`{k}`: {va!r} -> {vb!r}" for k, (va, vb) in diff.items()) or "none") + "\n"]
    if "OracleBackend" in str(ma.get("model")) or "OracleBackend" in str(mb.get("model")): L.append("> At least one run is SYNTHETIC (oracle): harness validation only.\n")
    cfgs = sorted({k[1] for k in keys}, key=cfg_key); fams = [f for f in FAM_ORDER if any(A[k]["family"] == f for k in keys)]
    tests = []
    L += ["| config | family | n | A success | B success | B - A (pp) [95% CI] | A only ok | B only ok | p | p (Holm) |", "|---|---|---|---|---|---|---|---|---|---|"]
    for c in cfgs:
        for f in fams + ["ALL"]:
            ks = [k for k in keys if k[1] == c and (f == "ALL" or A[k]["family"] == f)]
            if not ks: continue
            sa = [int(A[k]["success"]) for k in ks]; sb = [int(B[k]["success"]) for k in ks]
            ao = sum(a and not b for a, b in zip(sa, sb)); bo = sum(b and not a for a, b in zip(sa, sb))
            tests.append([c, f, len(ks), 100 * sum(sa) / len(ks), 100 * sum(sb) / len(ks), ao, bo, mcnemar_exact(bo, ao), paired_diff_ci(sa, sb)])
    adj = holm([t[7] for t in tests if t[1] != "ALL"]); it = iter(adj)
    for t in tests:
        pa = "–" if t[1] == "ALL" else f"{next(it):.3g}"
        d, lo, hi = t[8]
        L.append(f"| {t[0]} | {t[1]} | {t[2]} | {t[3]:.1f}% | {t[4]:.1f}% | {d:+.1f} [{lo:+.1f}, {hi:+.1f}] | {t[5]} | {t[6]} | {t[7]:.3g} | {pa} |")
    L += ["\nHolm is applied over the per-family rows; ALL rows are summaries. The effect size is the paired risk difference with a 95% "
          "Agresti-Min interval; report it next to the p value, and read an interval that includes 0 as no detected difference.\n", "## Cost (means per episode)\n",
          "| config | model calls A / B | actions A / B | repeated % A / B | collisions A / B | exec time s A / B |", "|---|---|---|---|---|---|"]
    for c in cfgs:
        ks = [k for k in keys if k[1] == c]
        def m(R, col):
            v = [fnum(R[k].get(col)) for k in ks if fnum(R[k].get(col)) is not None]; return f"{st.mean(v):.2f}" if v else "–"
        L.append(f"| {c} | {m(A, 'llm_calls')} / {m(B, 'llm_calls')} | {m(A, 'actions_executed')} / {m(B, 'actions_executed')} | "
                 f"{m(A, 'repeated_call_pct')} / {m(B, 'repeated_call_pct')} | {m(A, 'collisions')} / {m(B, 'collisions')} | {m(A, 'elapsed_s')} / {m(B, 'elapsed_s')} |")
    out = x.out or os.path.join(x.b, "COMPARE_vs_" + os.path.basename(os.path.normpath(x.a)) + ".md")
    open(out, "w", encoding="utf-8").write("\n".join(L) + "\n"); print("wrote", out)

if __name__ == "__main__": main()
