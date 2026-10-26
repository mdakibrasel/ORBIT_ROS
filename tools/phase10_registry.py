r"""Phase 10 experiment registry: records SHA-256 hashes of the historical Phase 8/9 result files so Phase 10 can REFERENCE them
as baselines without recomputing or overwriting them. Writes results_nao_benchmark/phase10_memory_<ts>/historical_registry.json.

Run once at the start of Phase 10, and again at the end with --verify to prove the historical files are byte-for-byte unchanged.

    .venv\Scripts\python.exe tools\phase10_registry.py            # create the registry (also prints the Phase 10 output dir)
    .venv\Scripts\python.exe tools\phase10_registry.py --verify <registry.json>   # re-hash and confirm nothing changed
"""
from __future__ import annotations
import hashlib, json, os, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results_nao_benchmark")

# The historical result files/dirs Phase 10 treats as immutable baselines (Phase 8 reliability + Phase 9 research).
HISTORICAL = [
    "docs/RELIABILITY_REPORT.md", "docs/PHASE9_RESEARCH_REPORT.md",
    "results_nao_benchmark/reliability_20261024T145555Z/reliability.json",
    "results_nao_benchmark/phase9_consolidated_20261024T190219Z/trials.csv",
    "results_nao_benchmark/phase9_consolidated_20261024T190219Z/trials.jsonl",
    "results_nao_benchmark/phase9_consolidated_20261024T190219Z/summary.json",
    "results_nao_benchmark/phase9_consolidated_20261024T190219Z/comparison.md",
]

def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""): h.update(chunk)
    return h.hexdigest()

def hash_all():
    out = {}
    for rel in HISTORICAL:
        p = os.path.join(ROOT, rel)
        out[rel] = {"sha256": sha256(p), "bytes": os.path.getsize(p)} if os.path.exists(p) else {"missing": True}
    # also fold in every phase9_live_*/summary or trace count so the full historical footprint is captured
    live_dirs = sorted(d for d in os.listdir(RES) if d.startswith("phase9_live_"))
    out["_phase9_live_dirs"] = {d: len([f for f in os.listdir(os.path.join(RES, d)) if f.endswith(".trace.json")]) for d in live_dirs}
    return out

def main(argv):
    if argv and argv[0] == "--verify":
        reg = json.load(open(argv[1], encoding="utf-8"))
        now = hash_all(); bad = []
        for rel, rec in reg["files"].items():
            if rel.startswith("_"): continue
            cur = now.get(rel, {})
            if rec.get("sha256") != cur.get("sha256"):
                bad.append(f"{rel}: registry={rec.get('sha256','?')[:12]} now={cur.get('sha256','?')[:12]}")
        if bad:
            print("\n" + "\n".join(bad)); return 1
        print(f"HISTORICAL INTEGRITY: OK ({sum(1 for k in reg['files'] if not k.startswith('_'))} files unchanged)"); return 0
    out = os.path.join(RES, "phase10_memory_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(out, exist_ok=True)
    reg = {"created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "note": "SHA-256 of historical Phase 8/9 result files. Phase 10 references these; it must NOT recompute or overwrite them.",
           "files": hash_all()}
    with open(os.path.join(out, "historical_registry.json"), "w", encoding="utf-8") as f: json.dump(reg, f, indent=2)
    print("PHASE10_OUT_DIR=" + out)
    print(json.dumps(reg, indent=2))
    return 0

if __name__ == "__main__": sys.exit(main(sys.argv[1:]))
