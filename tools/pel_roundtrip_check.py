"""M1 legacy round-trip check over real benchmark plans (synthetic: produced by the OracleBackend).

    python tools/pel_roundtrip_check.py                       # whole suite, 1 seed, every plan-producing config
    python tools/pel_roundtrip_check.py --families goto --seeds 1

Every plan the backend returns (plan, repair, plan_step) is captured inside a normal episode, then checked:
to_legacy(parse_plan(x)) == agent2d.normalize_plan(x). Episodes are run exactly as run_spatial.py runs them (same task grid), but the
wrapper does not change what the backend returns, so the episode results are those of the unwrapped backend."""
import argparse, copy, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orbit_ros.tasks2d import make_task, ALL_FAMILIES
from orbit_ros.agent2d import run_episode, PRESETS, normalize_plan
from orbit_ros.backends2d import OracleBackend
from orbit_ros.pel.schema import parse_plan, to_legacy, errors

CONFIGS = ("S1", "S2", "S3", "S4", "S5", "C2", "C3", "C4", "C5")

class Capture:
    def __init__(self, inner): self.inner, self.plans = inner, []
    def __getattr__(self, n): return getattr(self.inner, n)
    def _keep(self, p): self.plans.append(copy.deepcopy(p)); return p
    def plan(self, ph, ctx): return self._keep(self.inner.plan(ph, ctx))
    def repair(self, ph, ctx, failure): return self._keep(self.inner.repair(ph, ctx, failure))
    def plan_step(self, ph, ctx): return self._keep(self.inner.plan_step(ph, ctx))

def check(families=ALL_FAMILIES, seeds=1, configs=CONFIGS):
    n = bad = with_errors = 0; examples = []
    for fam in families:
        for s in range(seeds):
            for yaw in (0, 90, 180, 270):
                for cfg in configs:
                    w, t = make_task(fam, s, yaw, 3, False, 1)
                    be = Capture(OracleBackend()); run_episode(w, t, PRESETS[cfg], be)
                    for p in be.plans:
                        n += 1
                        plan, issues = parse_plan(copy.deepcopy(p))
                        if errors(issues): with_errors += 1
                        if to_legacy(plan) != normalize_plan(copy.deepcopy(p)):
                            bad += 1
                            if len(examples) < 3: examples.append((fam, cfg, p))
    return n, bad, with_errors, examples

def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("--families", nargs="*", default=list(ALL_FAMILIES)); ap.add_argument("--seeds", type=int, default=1)
    a = ap.parse_args(argv)
    n, bad, werr, ex = check(a.families, a.seeds)
    print(f"plans checked {n}; round-trip mismatches {bad}; plans with schema errors {werr}")
    for e in ex: print("  mismatch:", e)
    print("RESULT:", "PASS" if n and not bad else "FAIL"); return 0 if n and not bad else 1

if __name__ == "__main__": sys.exit(main())
