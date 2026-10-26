"""Scenario sets A-D (v0.12): named slices of the benchmark, run with `run_spatial.py --scenario A B C D`.

A  Basic navigation        one robot, one target, a few static obstacles, one start heading (0)           -> does the basic task work?
B  Different orientations  the same instructions (goto, motion, clean the left half) from all four start headings -> heading invariance
C  Multiple objects        several coloured boxes: visit them in the stated order (sequence), pick the object by a relation (relative)
D  Long-horizon            three boxes in order and back to the start, a mid-course change of goal, remember-and-return; obstacles that
                           move every 6 robot calls and two injected transient motor faults (2nd and 5th move) -> keeping and repairing a plan

Membership is a predicate on the instance (family, heading, objects, mirrored, level), so an instance may belong to several sets
(A is the heading-0 part of B's goto). Each instance runs once; REPORT.md section 6 summarises every set.
D's realism (moving obstacles, motor faults) applies to D's instances only and is added to any realism switches given on the command line."""
import itertools

SCENARIOS = {
    "A": {"title": "Basic navigation", "yaws": (0,), "mirrored": (False,), "seed_mult": 4,     # one heading only: 4x the seeds
          "families": {"goto": {"levels": (1,), "ns": (3, 5)}}},
    "B": {"title": "Different orientations", "yaws": (0, 90, 180, 270), "mirrored": (False,),
          "families": {"goto": {"levels": (1,), "ns": (3, 5)}, "motion": {"levels": (1, 2, 3), "ns": (1, 3)},
                       "clean_left": {"levels": (1,), "ns": (3, 5)}}},
    "C": {"title": "Multiple objects", "yaws": (0, 90, 180, 270), "mirrored": (False,),
          "families": {"sequence": {"levels": (1, 2), "ns": (3, 5)}, "relative": {"levels": (1, 2, 3), "ns": (3, 5)}}},
    "D": {"title": "Long-horizon", "yaws": (0, 90, 180, 270), "mirrored": (False,),
          "families": {"sequence": {"levels": (4,), "ns": (5,)}, "change": {"levels": (1, 2), "ns": (3, 5)},
                       "remember_return": {"levels": (1,), "ns": (2,)}},
          "realism": {"moving_every": 6, "fail_moves": (2, 5)}},
}

def member(name, fam, s, yaw, n, m, lv, seeds):
    sc = SCENARIOS[name]; f = sc["families"].get(fam)
    return (f is not None and yaw in sc["yaws"] and m in sc["mirrored"] and n in f["ns"] and lv in f["levels"]
            and s < seeds * sc.get("seed_mult", 1))

def scenarios_of(fam, s, yaw, n, m, lv, names, seeds):
    return [k for k in names if member(k, fam, s, yaw, n, m, lv, seeds)]

def instances(names, seeds):
    """Union of the scenarios' instances, each once, in scenario order: (family, seed, yaw, n_objects, mirrored, level)."""
    out, seen = [], set()
    for k in names:
        sc = SCENARIOS[k]
        for fam, f in sc["families"].items():
            for yaw, n, m, lv, s in itertools.product(sc["yaws"], f["ns"], sc["mirrored"], f["levels"], range(seeds * sc.get("seed_mult", 1))):
                i = (fam, s, yaw, n, m, lv)
                if i not in seen: seen.add(i); out.append(i)
    return out

def realism_for(fam, s, yaw, n, m, lv, names, seeds):
    """Extra realism settings of the scenarios this instance belongs to (only D has any)."""
    out = {}
    for k in scenarios_of(fam, s, yaw, n, m, lv, names, seeds): out.update(SCENARIOS[k].get("realism", {}))
    return out
