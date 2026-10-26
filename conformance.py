"""Run the adapter conformance suite (orbit_ros/conformance.py) against one simulator backend.

    python conformance.py --sim world2d
    python conformance.py --sim fake --motion continuous
    python conformance.py --sim webots --motion grid                 # needs a Webots TurtleBot world (removed from this project)
    python conformance.py --sim ros2 --ros2-config configs/ros2_jazzy_tb3.json     # untested: needs ROS 2 + gz sim

Writes conformance_<sim>_<motion>.md into --out and exits 0 only if every check passed."""
import argparse, os, sys
from orbit_ros.contract_api import world2d_adapter, fakesim_adapter, sim_adapter
from orbit_ros.conformance import run_suite, report_md

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", choices=["world2d", "fake", "webots", "ros2"], default="world2d")
    ap.add_argument("--motion", choices=["grid", "continuous"], default="grid")
    ap.add_argument("--ros2-config", default=None); ap.add_argument("--out", default="conformance")
    a = ap.parse_args(argv)
    if a.sim == "world2d": ad = world2d_adapter()
    elif a.sim == "fake": ad = fakesim_adapter(a.motion)
    elif a.sim == "webots":
        from orbit_ros.webots_adapter import WebotsSim; ad = sim_adapter(WebotsSim(), a.motion, "webots")
    else:
        from orbit_ros.ros2_adapter import load_ros2_config, Ros2Sim
        ad = sim_adapter(Ros2Sim(**load_ros2_config(a.ros2_config)), a.motion, "ros2")
    res = run_suite(ad); os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f"conformance_{a.sim}_{a.motion}.md"); open(path, "w", encoding="utf-8").write(report_md(ad, res))
    for r in res: print(f"{'PASS' if r['passed'] else 'FAIL'}  {r['check']:<20} {r['detail'][:100]}")
    print("wrote", path); return 0 if all(r["passed"] for r in res) else 1

if __name__ == "__main__": sys.exit(main())
