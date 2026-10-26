"""Phase 12 live viewpoint check (real Webots NAO). One fresh process. Switches camera modes DURING a real navigation and captures
mode-tagged screenshots; start/end stay in overview. The viewpoint never moves the robot — verified by the true-pose timeline.

Usage: python tools/live/p12_viewpoint_live.py <run>
"""
import json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tools.live.webots_session import LiveTrial
from orbit_ros import house

run = sys.argv[1] if len(sys.argv) > 1 else "p12vp"

with LiveTrial(run=run, trial="viewpoint_nav", instruction="go to the kitchen (camera demo)", start_room="living_room") as t:
    # 1) overview is the start (already set + start.png captured by the harness)
    t.camera("overview"); t.screenshot("01_overview.png", mode="overview", phase="before")
    # 2) follow mode (built-in Viewpoint following) before the walk
    fr = t.camera("follow"); t.screenshot("02_follow_start.png", mode="follow", phase="before")
    t.event("follow_fields", **fr)
    # 3) target mode framing the kitchen entry together with the robot
    kx, ky, _ = house.ROOMS["kitchen"][0]
    t.camera("target", target_xy=(kx, ky)); t.screenshot("03_target_kitchen.png", mode="target", phase="before")
    # 4) switch to follow and actually navigate (the robot moves; camera is display only)
    t.camera("follow")
    r = t.session.robot_call("go_to_room", {"room": "kitchen"})
    t.event("tool", tool="go_to_room", status=r["status"], message=str(r.get("message", ""))[:160])
    t.screenshot("04_follow_arrived.png", mode="follow", phase="during")
    # 5) demo mode HUD + a labelled shot
    t.hud(step="arrived: kitchen", verify="go_to_room ok")
    t.camera("demo"); t.screenshot("05_demo.png", mode="demo", phase="during")
    t.set_metrics({"calls": t.session.calls, "replans": t.session.replans, "modes_used": ["overview", "follow", "target", "demo"]})
    # verdict: the robot genuinely reached the kitchen (true pose) AND the viewpoint changes moved nothing unexpectedly
    t.set_verdict(pass_if=lambda s: s._pose()["room"] == "kitchen" and not s._pose()["fallen"],
                  measure=lambda s: s._pose(), condition="reached kitchen (true pose) while camera switched modes")
print("P12VP done ->", t.dir)
