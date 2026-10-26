"""Row-level regression check: are two runs identical on every non-timing column?

    python tools/compare_rows.py results_v013_oracle_all results_regress_m1

Pairs rows by (task_id, config, rep). Timing columns (wall-clock seconds and latencies) are ignored because they differ on every run.
Exit code 0 only if every paired row is identical on every other shared column and both runs contain the same keys; differences are
listed, never suppressed."""
import argparse, csv, os, sys
from collections import Counter

TIMING = {"elapsed_s", "harness_s", "llm_latency_s", "exec_time_s", "latency_s"}

def is_timing(col): return col in TIMING or col.endswith("_time_s")

def load(d):
    with open(os.path.join(d, "trial_results.csv"), encoding="utf-8") as f:
        return {(r["task_id"], r["config"], r.get("rep") or "0"): r for r in csv.DictReader(f)}

def compare(a_dir, b_dir, configs=None):
    A, B = load(a_dir), load(b_dir)
    if configs: A = {k: v for k, v in A.items() if k[1] in configs}; B = {k: v for k, v in B.items() if k[1] in configs}
    keys = sorted(set(A) & set(B))
    cols = sorted(c for c in set(next(iter(A.values()), {})) & set(next(iter(B.values()), {})) if not is_timing(c))
    diff_cols, diff_rows, first = Counter(), 0, []
    for k in keys:
        bad = [c for c in cols if A[k].get(c) != B[k].get(c)]
        if bad:
            diff_rows += 1; diff_cols.update(bad)
            if len(first) < 5: first.append((k, [(c, A[k].get(c), B[k].get(c)) for c in bad[:4]]))
    return {"paired": len(keys), "only_a": len(set(A) - set(B)), "only_b": len(set(B) - set(A)), "columns_compared": len(cols),
            "rows_differing": diff_rows, "columns_differing": dict(diff_cols), "examples": first}

def main(argv=None):
    p = argparse.ArgumentParser(); p.add_argument("a"); p.add_argument("b"); p.add_argument("--configs", nargs="*")
    x = p.parse_args(argv); r = compare(x.a, x.b, x.configs)
    print(f"paired rows {r['paired']}, only in A {r['only_a']}, only in B {r['only_b']}, columns compared {r['columns_compared']}")
    print(f"rows differing {r['rows_differing']}")
    for c, n in sorted(r["columns_differing"].items(), key=lambda t: -t[1]): print(f"  column {c}: {n} rows")
    for k, d in r["examples"]: print(f"  e.g. {k}: {d}")
    ok = r["rows_differing"] == 0 and r["only_a"] == 0 and r["only_b"] == 0 and r["paired"] > 0
    print("RESULT:", "IDENTICAL" if ok else "DIFFERENT"); return 0 if ok else 1

if __name__ == "__main__": sys.exit(main())
