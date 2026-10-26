"""Adapter conformance suite (v0.13). Every check builds its own small scenario through the contract (`contract_api.RobotAdapter`).

A simulator is listed as supported only for the motion modes whose suite passed. Checks:
  capabilities        the required actions and sensors are declared
  frame_units         1 cell = 1 m; heading 0 = +x, 90 = +y (REP-103, counter-clockwise)
  turn_convention     "left" adds +90 degrees, "right" subtracts 90
  camera_convention   an object on the robot's left has image_x < 0 (unmirrored camera); nearer objects look bigger
  freshness           every observation has a newer step counter and timestamp; no stale frame without latency settings
  invalid_args        a malformed action is rejected with status "invalid" and does not move the robot
  collision_stop      driving into a wall stops the robot inside the room with status "blocked" (or "safety_stop")
  stop                after stop(), motion is refused with status "stopped"
  reset_semantics     the same spec gives the same first observation twice
  ground_truth_split  observations carry no object ids or object cells; state() does
  action_timeout      act(..., timeout_s) reports a status from the declared set
"""
from .contract_api import ScenarioSpec, STATUSES, REQUIRED_ACTIONS

def _spec(objects=(), start=(2, 2), yaw=0, w=7, h=5):
    return ScenarioSpec(w, h, start, yaw, list(objects))

def run_suite(adapter):
    out = []
    def check(name, fn):
        try: ok, detail = fn()
        except Exception as e: ok, detail = False, f"exception: {e!r}"[:200]
        out.append({"check": name, "passed": bool(ok), "detail": detail})
    def caps():
        c = adapter.capabilities(); missing = [a for a in REQUIRED_ACTIONS if a not in c["actions"]]
        return (not missing and "camera" in c["sensors"] and "pose" in c["sensors"]), f"missing actions {missing}" if missing else str(c)
    def frame():
        adapter.reset(_spec()); r = adapter.act("move_forward", {"distance": 1}); p1 = r.pose
        adapter.act("turn", {"direction": "left", "degrees": 90}); r2 = adapter.act("move_forward", {"distance": 1})
        ok = r.status == "ok" and (p1["x"], p1["y"]) == (3, 2) and (r2.pose["x"], r2.pose["y"], r2.pose["yaw_deg"]) == (3, 3, 90)
        return ok, f"after +1 m at yaw 0: {p1}; after left + 1 m: {r2.pose}"
    def turn():
        adapter.reset(_spec()); a = adapter.act("turn", {"direction": "left", "degrees": 90}).pose["yaw_deg"]
        b = adapter.act("turn", {"direction": "right", "degrees": 180}).pose["yaw_deg"]
        return (a, b) == (90, 270), f"left 90 -> {a}, then right 180 -> {b}"
    def camera():
        o = adapter.reset(_spec([("o0", "red box", 5, 3), ("o1", "blue box", 4, 2)]))
        v = {d["label"]: d for d in o.view}
        ok = "red box" in v and "blue box" in v and v["red box"]["image_x"] < 0 and v["blue box"]["apparent_size"] > v["red box"]["apparent_size"]
        return ok, f"view {o.view}"
    def fresh():
        adapter.reset(_spec()); a = adapter.observe(); b = adapter.observe()
        return b.step > a.step and b.stamp_s >= a.stamp_s and not a.stale and not b.stale and a.valid, f"steps {a.step} -> {b.step}"
    def invalid():
        adapter.reset(_spec()); r = adapter.act("move_forward", {"distance": "two"}); p = adapter.observe().pose
        return r.status == "invalid" and (p["x"], p["y"]) == (2, 2), f"status {r.status}, pose {p}"
    def collide():
        adapter.reset(_spec(start=(5, 2))); r = adapter.act("move_forward", {"distance": 4}); p = adapter.state()
        return r.status in ("blocked", "safety_stop") and (p["x"], p["y"]) == (6, 2), f"status {r.status}, true cell ({p['x']},{p['y']})"
    def stop():
        adapter.reset(_spec()); adapter.stop(); r = adapter.act("move_forward", {"distance": 1})
        return r.status == "stopped" and adapter.state()["x"] == 2, f"status after stop: {r.status}"
    def reset():
        spec = _spec([("o0", "red box", 4, 3)], yaw=90)
        a = adapter.reset(spec); adapter.act("move_forward", {"distance": 1}); b = adapter.reset(spec)
        return (a.pose, a.view) == (b.pose, b.view), f"{a.pose} vs {b.pose}"
    def split():
        o = adapter.reset(_spec([("o0", "red box", 5, 3)])); keys = {k for d in o.view for k in d}
        s = adapter.state()
        return not keys & {"id", "x", "y", "cell"} and s["objects"][0]["id"] == "o0", f"view keys {sorted(keys)}"
    def timeout():
        adapter.reset(_spec()); r = adapter.act("turn", {"direction": "left", "degrees": 90}, timeout_s=60.0)
        return r.status in STATUSES and r.status == "ok", f"status {r.status}, {r.duration_s:.4f} s"
    for name, fn in (("capabilities", caps), ("frame_units", frame), ("turn_convention", turn), ("camera_convention", camera),
                     ("freshness", fresh), ("invalid_args", invalid), ("collision_stop", collide), ("stop", stop),
                     ("reset_semantics", reset), ("ground_truth_split", split), ("action_timeout", timeout)):
        check(name, fn)
    return out

def report_md(adapter, results):
    n = sum(r["passed"] for r in results)
    L = [f"# Conformance: `{adapter.name}` ({adapter.motion} motion)", "", f"{n}/{len(results)} checks passed.", "",
         "| check | result | detail |", "|---|---|---|"]
    L += [f"| {r['check']} | {'PASS' if r['passed'] else 'FAIL'} | {r['detail'].replace('|', '/')} |" for r in results]
    return "\n".join(L) + "\n"
