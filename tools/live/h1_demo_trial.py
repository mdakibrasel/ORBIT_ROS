"""H.1 live acceptance: one real trial to the kitchen, writing a full evidence pack judged from the true pose.
Driven via session.robot_call (guard unchanged). Run as a fresh process with Webots up."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tools.live.webots_session import LiveTrial

run = sys.argv[1] if len(sys.argv) > 1 else "h1"
with LiveTrial(run=run, trial="go_kitchen", instruction="go to the kitchen", start_room="living_room") as t:
    t.hud(step="go_to_room kitchen")
    r = t.session.robot_call("go_to_room", {"room": "kitchen"})
    t.event("tool", tool="go_to_room", args={"room": "kitchen"}, status=r["status"], message=str(r.get("message", ""))[:160])
    t.set_metrics({"calls": t.session.calls, "replans": t.session.replans})
    v = t.set_verdict(pass_if=lambda s: s._pose()["room"] == "kitchen" and not s._pose()["fallen"],
                      measure=lambda s: s._pose(), condition="final room == kitchen and upright")
print("H1_VERDICT", v["result"], "dir", t.dir)
