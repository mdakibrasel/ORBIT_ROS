"""Real-life style benchmark for the NAO in the ORBIT house (v0.16.3).

    python tools/nao_benchmark.py --sim webots            # Webots open first: .\\start_webots.ps1
    python tools/nao_benchmark.py --sim fake              # kinematic stand-in, no Webots (checks the benchmark itself)
    python tools/nao_benchmark.py --sim webots --repeat 3 --only mission_kitchen_bathroom_bedroom_clean

Every task is judged from the SIMULATOR's true pose, not from what the robot reports, so a robot that believes it arrived but did not
is counted as a failure. The metrics are the ones the standard embodied-AI benchmarks use (Habitat/ObjectNav, BEHAVIOR, ALFRED):
success rate (SR; object tasks succeed within 1 m of the object, as in ObjectNav), SPL = success * shortest / max(walked, shortest) against
the shortest collision-free path on the house map, goal-conditioned success (fraction of sub-goals reached in a multi-step job) and
collisions. Also recorded per task: motions, metres really walked, falls recovered, bumps, guard vetoes, final position, simulated and
wall-clock time. Results: results_nao_benchmark/<timestamp>/results.csv, summary.json and report.txt.
"""
from __future__ import annotations
import argparse, csv, json, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros import house
from orbit_ros.humanoid import HumanoidSession

ROOMS = ("kitchen", "bathroom", "bedroom", "living_room")
OBJECTS = ("fridge", "bed", "toilet", "sofa", "desk", "wardrobe")

def item_gap(label, x, y):
    """Distance from (x, y) to the nearest box of the item with this label (0 = touching)."""
    best = 1e9
    for it in house.ITEMS:
        if it["label"] != label: continue
        gx = max(abs(x - it["x"]) - it["sx"] / 2, 0.0); gy = max(abs(y - it["y"]) - it["sy"] / 2, 0.0)
        best = min(best, math.hypot(gx, gy))
    return best

_SHARED = {}                                                        # Webots allows one Supervisor per process: every task reuses it

class Run:
    """One session plus a meter on the backend: true distance walked, true collisions, falls recovered."""
    def __init__(self, sim, start_room):
        backend = _SHARED.get(sim)
        self.s = HumanoidSession(sim=sim, log_dir=False, backend=backend)
        self.start_room = start_room
        self.nao = _SHARED[sim] = self.s.nao
        if not hasattr(self.nao, "_orig_play"): self.nao._orig_play = self.nao.play
        self.nao.play = self.nao._orig_play                         # drop the meter and watchers of the previous task
        self.s.load_scene(start_room)
        self.sim_time = (lambda: self.nao.sup.getTime()) if sim == "webots" else (lambda: 0.0)
        self.reset_meter()
        play, stand = self.nao._orig_play, self.s._t_stand_up
        def metered_play(motion):
            b = self.nao.pose(); play(motion); a = self.nao.pose()
            self.walked += math.hypot(a[0] - b[0], a[1] - b[1])
            if self.touching(a[0], a[1]): self.collisions += 1
        def metered_stand_up():
            self.falls += 1; return stand()
        self.nao.play = metered_play; self.s._t_stand_up = metered_stand_up
    @staticmethod
    def touching(x, y):
        return any(abs(x - o["x"]) < o["sx"] / 2 and abs(y - o["y"]) < o["sy"] / 2 for o in house.obstacles())
    def reset_meter(self):
        self.walked = 0.0; self.collisions = 0; self.falls = 0; self.planned = 0.0; self.motions = 0
        self.t_sim = self.sim_time(); self.t_wall = time.time(); self.vetoes0 = self.s.calls.get("vetoed", 0)
    def call(self, tool, args=None):
        r = self.s.robot_call(tool, args or {})
        self.motions += len(r.get("trail", [])) if "trail" in r else (1 if tool in ("walk_forward", "walk_backward", "turn", "side_step", "wave") else 0)
        wp = r.get("waypoints")
        if wp: self.planned += sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(wp, wp[1:]))
        return r
    def pose(self):
        x, y, yaw, fallen = self.nao.pose(); return x, y, yaw, fallen
    def shortest(self, legs):
        """Length of the shortest collision-free path start -> legs[0] -> legs[1] ... on the house map (the SPL baseline, as in Habitat)."""
        total, here = 0.0, tuple(self.s.start_pose[:2])
        for g in legs:
            p = house.plan_path(here, tuple(g), self.s._boxes(), self.s.CLEAR) or house.plan_path(here, tuple(g), self.s._boxes(), self.s.CLEAR_TIGHT)
            if p: total += sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))
            here = tuple(g)
        return total

# --------------------------------------------------------------------------------------------------------------- tasks
# Each task: (name, start_room, function(run) -> (ok, detail, legs, gc)). The function judges from the true pose.
#   legs = goal points in order (SPL baseline); None = the point where the robot really ended.   gc = goal-conditioned success, the
#   fraction of sub-goals reached (ALFRED/BEHAVIOR style): a 4-step job that does 3 steps scores 0.75.
def go_room(room):
    def f(r):
        res = r.call("go_to_room", {"room": room}); x, y, _, fell = r.pose()
        ok = res["status"] == "ok" and house.room_of(x, y) == room and not fell
        return ok, res["message"], [house.ROOMS[room][0][:2]], float(ok)
    return f

def go_object(name):
    def f(r):
        res = r.call("go_to_object", {"name": name}); x, y, _, fell = r.pose()
        gap = item_gap(name, x, y)
        ok = res["status"] == "ok" and gap <= 1.5 and not fell            # success radius 1.5 m to the box (Habitat uses 1 m; the NAO must stop at the chair of a desk)
        return ok, f"{res['message']} (true gap {gap:.2f} m)", None, float(ok)
    return f

def mission(r):
    """The job from the user's request: kitchen, bathroom, bedroom, then clean."""
    legs, done = [], 0
    for room in ("kitchen", "bathroom", "bedroom"):
        res = r.call("go_to_room", {"room": room}); x, y, _, fell = r.pose(); legs.append(house.ROOMS[room][0][:2])
        if res["status"] != "ok" or house.room_of(x, y) != room or fell: return False, f"stopped at {room}: {res['message']}", legs, done / 4
        done += 1
    res = r.call("clean", {"mode": "vacuum"}); cs = r.s.clean_status()
    ok = res["status"] == "ok" and "bedroom" in cs["rooms"]
    return ok, f"clean: {res['message']}; rooms cleaned {cs['rooms']}", legs, (done + int(ok)) / 4

def clean_house(r):
    res = r.call("clean_house"); cs = r.s.clean_status(); x, y, _, fell = r.pose()
    ok = res["status"] == "ok" and len(cs["rooms"]) == len(house.ROOMS) and not fell and r.s._verify(r.s.last_task)["passed"]   # rooms swept AND floor coverage verified
    return ok, f"{res['message']}; rooms {cs['rooms']}", [house.ROOMS[k][0][:2] for k in house.ROOMS], len(cs["rooms"]) / len(house.ROOMS)

def clean_each_room(r):
    """Go to every room and clean it with the hand: the true cleaned-room list must match the visited rooms."""
    legs, done = [], 0
    for room in ("kitchen", "bathroom", "bedroom"):
        a = r.call("go_to_room", {"room": room}); legs.append(house.ROOMS[room][0][:2]); x, y, _, fell = r.pose()
        b = r.call("clean", {"mode": "mop"}) if a["status"] == "ok" else a
        if a["status"] != "ok" or b["status"] != "ok" or house.room_of(x, y) != room or fell:
            return False, f"stopped in {room}: {a['message']} / {b['message']}", legs, done / 3
        done += 1
    rooms = r.s.clean_status()["rooms"]
    return set(rooms) >= {"kitchen", "bathroom", "bedroom"}, f"cleaned rooms {rooms}", legs, done / 3

def mission_and_return(r):
    ok, detail, legs, gc = mission(r)
    if not ok: return False, detail, legs, gc * 4 / 5
    res = r.call("return_to_start"); x, y, _, fell = r.pose(); sx, sy = r.s.start_pose[:2]
    d = math.hypot(x - sx, y - sy); ok2 = res["status"] == "ok" and d <= 0.6 and not fell
    return ok2, f"return_to_start: {res['message']} (true distance to start {d:.2f} m)", legs + [(sx, sy)], (4 + int(ok2)) / 5

def long_route(r):
    """Corner to corner through the whole house: bathroom, then kitchen, then bedroom, then back to the living room."""
    legs, done = [], 0
    for room in ("bathroom", "kitchen", "bedroom", "living_room"):
        res = r.call("go_to_room", {"room": room}); legs.append(house.ROOMS[room][0][:2]); x, y, _, fell = r.pose()
        if res["status"] != "ok" or house.room_of(x, y) != room or fell: return False, f"stopped at {room}: {res['message']}", legs, done / 4
        done += 1
    return True, "visited all four rooms", legs, 1.0

def fall_recovery(r):
    """The robot is pushed over (simulator reset of the body is not used): stand_up must put it upright, then it must still walk."""
    r.nao.set_pose(4.8, 3.1, 180); res = r.call("stand_up"); _, _, _, fell = r.pose()
    go = r.call("go_to_room", {"room": "kitchen"}); x, y, _, fell2 = r.pose()
    ok = res["status"] == "ok" and not fell and go["status"] == "ok" and house.room_of(x, y) == "kitchen" and not fell2
    return ok, f"stand_up: {res['message']}; then {go['message']}", [house.ROOMS["kitchen"][0][:2]], float(ok)

def no_go_zone(r):
    """A no-go zone beside the way to the kitchen (not on the doorway): the NAO must get there without ever entering it."""
    zx, zy, rad = 4.2, 3.7, 0.4
    r.s.set_no_go_zone(zx, zy, rad)
    inside = {"n": 0}
    play = r.nao.play
    def watch(m):
        play(m); a = r.nao.pose()
        if math.hypot(a[0] - zx, a[1] - zy) < rad: inside["n"] += 1
    r.nao.play = watch
    res = r.call("go_to_room", {"room": "kitchen"}); x, y, _, fell = r.pose()
    ok = res["status"] == "ok" and house.room_of(x, y) == "kitchen" and inside["n"] == 0 and not fell
    return ok, f"{res['message']}; motions that ended inside the zone: {inside['n']}", None, float(ok)

def blocked_doorway(r):
    """A no-go zone on the only way to the kitchen: the right answer is a refusal ('no safe path'), and the NAO must not move."""
    r.s.set_no_go_zone(4.95, 4.4, 0.6)
    res = r.call("go_to_room", {"room": "kitchen"}); x, y, _, _ = r.pose()
    ok = res["status"] == "rejected" and math.hypot(x - 4.8, y - 3.1) < 0.3
    return ok, f"{res['status']}: {res['message']}", None, float(ok)

def refuse_outside(r):
    res = r.call("go_to", {"x": 20, "y": 3}); x, y, _, _ = r.pose()
    ok = res["status"] == "rejected" and math.hypot(x - 4.8, y - 3.1) < 0.3
    return ok, f"{res['status']}: {res['message']}", None, float(ok)

def refuse_wall_walk(r):
    """Walk forward into furniture/wall repeatedly: the guard must stop it before the body touches anything."""
    outcome = "ok"
    for _ in range(6):
        res = r.call("walk_forward", {"steps": 6})
        outcome = res["status"]
        if outcome != "ok": break
    ok = outcome in ("vetoed", "failed") and r.collisions == 0
    return ok, f"stopped with '{outcome}', true collisions {r.collisions}", None, float(ok)

def task_list():
    t = [(f"room_{room}", "living_room", go_room(room)) for room in ROOMS if room != "living_room"]
    t += [(f"object_{o}", "living_room", go_object(o)) for o in OBJECTS]
    t += [("mission_kitchen_bathroom_bedroom_clean", "living_room", mission),
          ("clean_each_room_by_hand", "living_room", clean_each_room),
          ("mission_then_return_to_start", "living_room", mission_and_return),
          ("long_route_all_rooms", "living_room", long_route),
          ("clean_house", "living_room", clean_house),
          ("fall_recovery", "living_room", fall_recovery),
          ("no_go_zone_respected", "living_room", no_go_zone),
          ("blocked_doorway_refused", "living_room", blocked_doorway),
          ("refuse_outside_house", "living_room", refuse_outside),
          ("guard_stops_wall_walk", "living_room", refuse_wall_walk)]
    return t

def run_task(sim, name, start, fn, rep):
    r = Run(sim, start)
    try:
        ok, detail, legs, gc = fn(r)
        err = ""
    except Exception as e:                                       # a controller crash is a failed task, never a skipped one
        ok, detail, legs, gc, err = False, f"EXCEPTION {type(e).__name__}: {e}", None, 0.0, str(e)
    x, y, _, fell = r.pose()
    shortest = r.shortest(legs if legs else [(x, y)])
    spl = round(int(bool(ok)) * shortest / max(r.walked, shortest), 3) if shortest > 0.05 else (1.0 if ok else 0.0)
    m = r.s.metrics()
    return {"task": name, "model": MODEL, "rep": rep, "success": int(bool(ok)), "goal_conditioned": round(gc, 2), "spl": spl, "detail": detail, "motions": r.motions,
            "walked_m": round(r.walked, 2), "shortest_m": round(shortest, 2), "planned_m": round(r.planned, 2),
            "falls_recovered": r.falls, "fell_at_end": int(bool(fell)), "bumps": len(r.s.bumps), "vetoes": r.s.calls.get("vetoed", 0) - r.vetoes0,
            "true_collisions": r.collisions, "replans": m["replans"], "sonar_detections": m["sonar_detections"], "cleaned_area_m2": m["cleaned_area_m2"],
            "coverage_pct": m["coverage_pct"], "turn_error_deg": m["turn_error_deg"], "heading_drift_deg": m["heading_drift_deg"],
            "final_x": round(x, 2), "final_y": round(y, 2),
            "sim_s": round(r.sim_time() - r.t_sim, 1), "wall_s": round(time.time() - r.t_wall, 1), "error": err}

MODEL = "scripted"                                               # label of the model/agent under test (column `model`)

def compare(paths):
    """Mean of the key metrics per (task, model) over several results.csv files: compares scenarios, simulators and models."""
    from collections import defaultdict
    g = defaultdict(list)
    for p in paths:
        with open(p, encoding="utf-8") as f:
            for row in csv.DictReader(f): g[(row["task"], row.get("model") or "scripted")].append(row)
    cols = ("success", "spl", "walked_m", "wall_s", "coverage_pct", "true_collisions", "replans")
    def mean(rs, k):
        v = [float(x[k]) for x in rs if x.get(k) not in (None, "")]; return f"{sum(v) / len(v):.2f}" if v else "-"
    print(f"{'task':40s}{'model':14s}{'n':>3s}" + "".join(f"{c:>16s}" for c in cols))
    for (t, mo), rs in sorted(g.items()): print(f"{t:40s}{mo:14s}{len(rs):3d}" + "".join(f"{mean(rs, c):>16s}" for c in cols))

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", choices=("fake", "webots"), default="webots")
    ap.add_argument("--repeat", type=int, default=1, help="repetitions of every task")
    ap.add_argument("--only", nargs="*", help="run only these task names")
    ap.add_argument("--out", default=os.path.join(ROOT, "results_nao_benchmark"))
    ap.add_argument("--model", default="scripted", help="label for the model/agent under test (column `model`)")
    ap.add_argument("--compare", nargs="+", metavar="CSV", help="compare results.csv files (per task and model) instead of running")
    a = ap.parse_args(argv)
    if a.compare: compare(a.compare); return 0
    global MODEL; MODEL = a.model
    tasks = [t for t in task_list() if not a.only or t[0] in a.only]
    if not tasks: print("no task matches; names:", ", ".join(t[0] for t in task_list())); return 2
    out = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S")); os.makedirs(out, exist_ok=True)
    rows = []
    for rep in range(1, a.repeat + 1):
        for name, start, fn in tasks:
            row = run_task(a.sim, name, start, fn, rep); rows.append(row)
            print(f"[{'PASS' if row['success'] else 'FAIL'}] {name} #{rep}: {row['detail']} | motions {row['motions']}, walked {row['walked_m']} m, "
                  f"falls {row['falls_recovered']}, collisions {row['true_collisions']}, {row['wall_s']} s", flush=True)
    with open(os.path.join(out, "results.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    n = len(rows); ok = sum(r["success"] for r in rows)
    summary = {"simulator": a.sim, "tasks": n, "passed": ok, "success_rate": round(ok / n, 3),
               "goal_conditioned_success": round(sum(r["goal_conditioned"] for r in rows) / n, 3),
               "spl": round(sum(r["spl"] for r in rows) / n, 3), "motions_total": sum(r["motions"] for r in rows),
               "walked_m_total": round(sum(r["walked_m"] for r in rows), 1), "falls_recovered": sum(r["falls_recovered"] for r in rows),
               "ended_fallen": sum(r["fell_at_end"] for r in rows), "true_collisions": sum(r["true_collisions"] for r in rows),
               "wall_s_total": round(sum(r["wall_s"] for r in rows), 1), "failed_tasks": sorted({r["task"] for r in rows if not r["success"]})}
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f: json.dump(summary, f, indent=2)
    lines = [f"NAO house benchmark ({a.sim}): {ok}/{n} passed, success {summary['success_rate'] * 100:.0f}%, "
             f"goal-conditioned {summary['goal_conditioned_success'] * 100:.0f}%, SPL {summary['spl']}",
             f"falls recovered {summary['falls_recovered']}, ended fallen {summary['ended_fallen']}, true collisions {summary['true_collisions']}",
             f"failed: {', '.join(summary['failed_tasks']) or 'none'}", f"results: {out}"]
    open(os.path.join(out, "report.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))
    return 0 if ok == n else 1

if __name__ == "__main__": sys.exit(main())
