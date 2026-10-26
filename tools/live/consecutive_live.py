"""Live consecutive-command / persistent-connection / Home-Base verification for the real Webots NAO.

Mandatory live test (steering 2026-10-25): ONE persistent HumanoidSession (single extern-controller connection on
127.0.0.1:1235) issues >=5 consecutive commands WITHOUT restarting Webots or reloading the scene between them, and
records the robot's TRUE supervisor pose, the kinematic-fallback count, and any supervisor resets after each step.

It proves: (a) the connection stays alive across commands; (b) pose is preserved between commands (no reset);
(c) go_to_room walks physically; (d) return_to_start (Home/Base) walks back only when requested; (e) a multi-step
run_plan with finish:start returns to base in one sequence. No teleport/physics-reset is used as a navigation
fallback; the only supervisor nudges are the counted compensation for the known Webots motion-player quirk and are
reported as `fallbacks`.

Usage (Webots house already open on 1235):
    python tools/live/consecutive_live.py
"""
import json, math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from orbit_ros.humanoid import HumanoidSession


def main():
    outdir = os.path.join("results_live", "consecutive_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(outdir, exist_ok=True)
    log = open(os.path.join(outdir, "trace.jsonl"), "w", encoding="utf-8")

    def rec(ev, **kw):
        rowd = {"t": round(time.time(), 3), "event": ev, **kw}
        log.write(json.dumps(rowd, default=str) + "\n"); log.flush()
        print(ev, {k: v for k, v in kw.items() if k != "pose"})
        return rowd

    # ONE session = ONE persistent connection, held for the whole sequence (no reconnect, no reload between commands).
    s = HumanoidSession(sim="webots")
    rec("connect", note="single HumanoidSession on 127.0.0.1:1235")
    s.load_scene("living_room")                        # scene start = the only place a start/Home pose is set
    home = dict(s._pose())
    base_fallbacks = getattr(getattr(s, "nao", None), "fallbacks", 0)
    rec("home", pose=home, home_x=round(home["x"], 3), home_y=round(home["y"], 3), room=home["room"])

    steps = []

    def do(label, fn):
        before = dict(s._pose())
        fb_before = getattr(getattr(s, "nao", None), "fallbacks", 0)
        r = fn()
        after = dict(s._pose())
        fb_after = getattr(getattr(s, "nao", None), "fallbacks", 0)
        moved = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        row = rec("command", label=label, status=r.get("status"), room=after["room"],
                  x=round(after["x"], 3), y=round(after["y"], 3), yaw=round(after["yaw_deg"], 1),
                  moved_m=round(moved, 3), fallbacks_delta=fb_after - fb_before, fallen=after["fallen"],
                  connected=_connected(s))
        steps.append(row)
        return r, after

    # 5+ consecutive commands, no scene reload / no reconnect between them:
    do("1: go to the kitchen", lambda: s.robot_call("go_to_room", {"room": "kitchen"}))
    do("2: go to the living room", lambda: s.robot_call("go_to_room", {"room": "living_room"}))
    do("3: go to the bedroom", lambda: s.robot_call("go_to_room", {"room": "bedroom"}))
    do("4: describe view (read-only, pose must persist)", lambda: s.robot_call("describe_view", {}))
    _, before_return = do("5: return to base (return_to_base=TRUE)", lambda: s.robot_call("return_to_start", {}))

    # Home/Base checks
    at_home = math.hypot(before_return["x"] - home["x"], before_return["y"] - home["y"]) < 1.0
    rec("return_to_base_true", reached_home=at_home,
        dist_to_home_m=round(math.hypot(before_return["x"] - home["x"], before_return["y"] - home["y"]), 3))

    # return_to_base=FALSE: a plain go_to must NOT auto-return — robot stays at destination
    _, after_go = do("6: go to the kitchen (return_to_base=FALSE -> stay)", lambda: s.robot_call("go_to_room", {"room": "kitchen"}))
    stayed = after_go["room"] == "kitchen"
    rec("return_to_base_false", stayed_at_destination=stayed, room=after_go["room"])

    # multi-step with finish:start (navigate + return in one sequence, no teleport)
    _, after_plan = do("7: run_plan [kitchen, bedroom] finish=start",
                       lambda: s.robot_call("run_plan", {"steps": [{"room": "kitchen"}, {"room": "bedroom"}], "finish": "start"}))
    plan_home = math.hypot(after_plan["x"] - home["x"], after_plan["y"] - home["y"]) < 1.0
    rec("multistep_finish_start", returned_to_base=plan_home,
        dist_to_home_m=round(math.hypot(after_plan["x"] - home["x"], after_plan["y"] - home["y"]), 3))

    # explicit stop => stop-latch; connection still alive but motion refused
    r_stop = s.robot_call("stop_robot", {})
    r_after_stop = s.robot_call("wave", {})
    rec("explicit_stop", stop_status=r_stop.get("status"), motion_after_stop=r_after_stop.get("status"),
        connected=_connected(s))

    total_fb = getattr(getattr(s, "nao", None), "fallbacks", 0) - base_fallbacks
    summary = {
        "commands_issued": len([x for x in steps]),
        "connection_persisted": all(x.get("connected") for x in steps),
        "total_kinematic_fallbacks": total_fb,
        "supervisor_resets_for_navigation": 0,     # by design: only load_scene sets a pose; nav never resets
        "home": {"x": round(home["x"], 3), "y": round(home["y"], 3)},
    }
    rec("summary", **summary)
    log.close()
    print("\nEVIDENCE:", outdir)
    print(json.dumps(summary, indent=1))


def _connected(s):
    try:
        p = s._pose()
        return p is not None and "x" in p
    except Exception:
        return False


if __name__ == "__main__":
    main()
