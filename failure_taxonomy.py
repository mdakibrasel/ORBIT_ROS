"""Classify failed episodes of a run into error classes (automatic FIRST GUESS; read the event traces before you cite a class).
  python failure_taxonomy.py results_groq120b_pilot          -> failure_taxonomy.csv, FAILURES.md (+ example traces per class if event_log.csv exists)

Classes: runtime_error, format_error (unparseable / invalid calls), looping (repeated calls), budget_exhausted, verifier_abort,
left_right_confusion, sensor_misreading_mirrored_camera, wrong_object, distance_judgement, turn_arithmetic, distance_arithmetic,
frame_of_reference, instruction_forgotten (sequence stage / update ignored), memory_loss, condition_misjudged, constraint_violated,
coverage_incomplete, counting_error, no_report, navigation_incomplete, unclassified.
The world is rebuilt from the task_id (make_task is deterministic), so no simulator state has to be logged."""
import collections, csv, json, os, sys
from orbit_ros.tasks2d import make_task, simulate_ops
from orbit_ros.world2d import to_robot_frame
from orbit_ros.agent2d import config_by_name

def parse_tid(tid):
    fam, y, n, m, l, s, *rest = tid.split("|")
    return fam, int(s[1:]), int(y[1:]), int(n[1:]), bool(int(m[1:])), int(l[1:]), (int(rest[0][1:]) if rest else 1)

def f(x, d=0.0):
    try: return float(x)
    except (TypeError, ValueError): return d

def lateral(world, o):
    sx, sy, syaw = world.start_pose; return to_robot_frame(o.x - sx, o.y - sy, syaw)[1]

def classify(r):
    if r.get("error"): return "runtime_error", r["error"][:80]
    ex = f(r.get("actions_executed"))
    if f(r.get("parse_errors")) > 0 and ex and f(r.get("invalid_calls")) / ex >= 0.3: return "format_error", f"{r.get('parse_errors')} parse errors"
    if ex and f(r.get("invalid_calls")) / ex >= 0.5: return "format_error", f"{r.get('invalid_calls')} invalid calls"
    if f(r.get("aborted")) > 0: return "verifier_abort", ""
    if ex >= 6 and f(r.get("repeated_call_pct")) >= 30: return "looping", f"{r.get('repeated_call_pct')}% repeated"
    fam, s, yaw, n, m, lv, sc = parse_tid(r["task_id"])
    world, task = make_task(fam, s, yaw, n, m, lv, sc)
    fx, fy, fyaw = int(f(r.get("final_x"))), int(f(r.get("final_y"))), int(f(r.get("final_yaw")))
    adj = [o for o in world.objects if abs(o.x - fx) + abs(o.y - fy) == 1]
    grounded = config_by_name(r["config"]).grounded if r["config"][:1] in "SC" else False
    def wrong_object(target_id):
        t = next(o for o in world.objects if o.id == target_id)
        if any(o.id == target_id for o in adj): return None
        if not adj: return ("budget_exhausted", "") if f(r.get("env_tool_calls")) >= f(r.get("tool_call_cap"), 300) - 10 else ("navigation_incomplete", "not next to any object")
        o = adj[0]
        if m and not grounded: return "sensor_misreading_mirrored_camera", f"ended at {o.label}"
        if lateral(world, o) * lateral(world, t) < 0: return "left_right_confusion", f"ended at {o.label} on the other side"
        return "wrong_object", f"ended at {o.label}"
    if fam == "motion":
        exp = task.expected["final_pose"]
        swapped = [(op, {"left": "right", "right": "left"}.get(v, v)) for op, v in task.phases[0].params["ops"]]
        mir, _ = simulate_ops(world.start_pose[:2], world.start_pose[2], swapped)
        if (fx, fy, fyaw) == tuple(mir): return "left_right_confusion", "pose of the mirrored turns"
        if (fx, fy) == tuple(exp[:2]): return "turn_arithmetic", f"heading {fyaw} != {exp[2]}"
        return "distance_arithmetic", f"cell ({fx},{fy}) != ({exp[0]},{exp[1]})"
    if fam in ("goto", "relative"):
        w = wrong_object(task.expected["target_id"])
        if w and fam == "relative" and w[0] == "wrong_object" and lv in (1, 3): return "distance_judgement", w[1]
        return w or ("unclassified", "")
    if fam == "remember_return":
        if any(o.id == "o1" for o in adj): return "memory_loss", "ended at the decoy box"
        return wrong_object("o0") or ("unclassified", "")
    if fam == "ambiguous":
        if lv == 2 and any(o.id == "o0" for o in adj): return "frame_of_reference", "used the robot's left, not the door's"
        return wrong_object(task.expected["target_id"]) or ("unclassified", "")
    if fam == "change":
        if any(o.id == "o0" for o in adj): return "instruction_forgotten", "went to the first target; update ignored"
        if task.expected["target_id"]: return wrong_object(task.expected["target_id"]) or ("unclassified", "")
        return "memory_loss", "did not return to the start"
    if fam == "conditional":
        other = "o1" if task.expected["target_id"] == "o0" else "o0"
        if any(o.id == other for o in adj): return "condition_misjudged", "took the other branch"
        return wrong_object(task.expected["target_id"]) or ("unclassified", "")
    if fam == "sequence":
        prog = f(r.get("progress"))
        if prog > 0: return "instruction_forgotten", f"progress {prog}"
        return "wrong_object" if adj else "navigation_incomplete", "first stage failed"
    if fam in ("clean_left", "clean_side", "prohibit"):
        if f(r.get("violations")) > 0: return "constraint_violated", f"{r.get('violations')} forbidden cell(s)"
        if f(r.get("wrong_side")) > 0.25: return "left_right_confusion" if fam != "clean_side" or lv == 1 else "frame_of_reference", f"wrong_side {r.get('wrong_side')}"
        if f(r.get("env_tool_calls")) >= f(r.get("tool_call_cap"), 300) - 10: return "budget_exhausted", ""
        return "coverage_incomplete", f"coverage {r.get('coverage')}"
    if fam == "report":
        if r.get("answer", "") == "": return "no_report", ""
        return "counting_error", f"answered {r.get('answer')}, truth {task.expected['answer']}"
    return "unclassified", ""

def main(d, examples=2):
    rows = list(csv.DictReader(open(os.path.join(d, "trial_results.csv"), encoding="utf-8")))
    fails = [r for r in rows if str(r.get("success")) != "1"]
    out = []
    for r in fails:
        cls, why = classify(r)
        out.append({"task_id": r["task_id"], "config": r["config"], "rep": r.get("rep", 0), "family": r["family"], "class": cls, "evidence": why})
    with open(os.path.join(d, "failure_taxonomy.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["task_id", "config", "rep", "family", "class", "evidence"]); w.writeheader(); w.writerows(out)
    cfgs = sorted({r["config"] for r in rows}, key=lambda c: (c[0] != "S", c)); classes = [c for c, _ in collections.Counter(o["class"] for o in out).most_common()]
    meta = json.load(open(os.path.join(d, "run_metadata.json"))) if os.path.exists(os.path.join(d, "run_metadata.json")) else {}
    L = ["# Failure classes (automatic first guess)\n", f"Run: `{meta.get('model', '?')}` ({meta.get('NOTE', '')}). Failed episodes: {len(out)} of {len(rows)}.\n",
         "Rules are in `failure_taxonomy.py`. A class is a hypothesis about the cause; check a few traces per class (below) before using it in a paper.\n",
         "| class | " + " | ".join(cfgs) + " | total |", "|---|" + "---|" * (len(cfgs) + 1)]
    for c in classes:
        L.append(f"| {c} | " + " | ".join(str(sum(1 for o in out if o["class"] == c and o["config"] == g)) for g in cfgs) + f" | {sum(1 for o in out if o['class'] == c)} |")
    ev_path = next((p for p in (os.path.join(d, "event_log.csv"), os.path.join(d, "event_log.csv.gz")) if os.path.exists(p)), None)
    if ev_path:
        import gzip
        evs = collections.defaultdict(list)
        fh = gzip.open(ev_path, "rt", encoding="utf-8") if ev_path.endswith(".gz") else open(ev_path, encoding="utf-8")
        for e in csv.DictReader(fh): evs[(e["task_id"], e["config"], str(e.get("rep", 0) or 0))].append(e)
        L.append("\n## Example traces\n")
        for c in classes:
            for o in [o for o in out if o["class"] == c][:examples]:
                L.append(f"**{c}** · `{o['task_id']}` · {o['config']} · {o['evidence']}\n```")
                for e in evs.get((o["task_id"], o["config"], str(o["rep"])), [])[:25]: L.append(f"{e['event']}: {e['details'][:160]}")
                L.append("```")
    open(os.path.join(d, "FAILURES.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("wrote", os.path.join(d, "FAILURES.md"))

if __name__ == "__main__": main(sys.argv[1])
