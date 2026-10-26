r"""Phase-16 LIVE validation: adaptive camera (7 modes + zoom) and the realistic human actor, against the real Webots NAO house.

Run this inside ONE fresh worker process while Webots is running (port 1235 listening), with the NAO controller free. It drives the
one live HumanoidSession, exercises every camera mode and a zoom through the SAME display-only Viewpoint the MCP camera() tool uses,
captures a native Webots screenshot per mode via sup.exportImage, samples the TRUE pose throughout to prove the robot never moved while
the camera changed, and records which world/human is loaded. All evidence lands under results_live/<run>/camera/ and .../human_model/.

    .venv\Scripts\python.exe tools\live\phase16_camera_trial.py --run phase16_visual_clarity_<ts>

This is a LIVE check: no fake backend, no scripted LLM, no teleport (the only pose write is load_scene at the start). It refuses to run
if 1235 is not listening or the controller connection is not ours.
"""
from __future__ import annotations
import argparse, json, os, sys, time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path: sys.path.insert(0, ROOT)
from tools.live.webots_session import LiveTrial, port_listening, RESULTS

CAMERA_MODES = ["overview", "follow", "closeup", "target", "human", "demo", "reset"]


def _pose_tuple(session):
    x, y, yaw, fallen = session.nao.pose()
    return (round(x, 4), round(y, 4), round(yaw, 2), bool(fallen))


def run(run_name, start_room="living_room"):
    if not port_listening():
        print("ABORT: Webots controller port 1235 is not listening. Start Webots (start_webots.ps1) first.", file=sys.stderr)
        return 2

    results = {"run": run_name, "camera": {}, "human": {}, "robot_moved_during_camera": None, "world": {}}
    with LiveTrial(run=run_name, trial="camera", instruction="exercise all 7 camera modes + zoom (display only)",
                   start_room=start_room) as t:
        cs = t.session.connection_status()
        results["world"]["controller_port"] = cs.get("controller_port")
        results["world"]["uses_port_1240"] = cs.get("uses_port_1240")
        # detect the loaded world's human node through the live Supervisor
        human_node = None
        try:
            human_node = t.session.nao.sup.getFromDef("ORBIT_HUMAN")
        except Exception:
            human_node = None
        results["world"]["human_present"] = human_node is not None
        if human_node is not None:
            try:
                p = human_node.getPosition()
                results["world"]["human_position"] = [round(v, 3) for v in p]
            except Exception:
                pass

        pose_before = _pose_tuple(t.session)
        results["camera"]["pose_before_all_modes"] = list(pose_before)

        # ---- every camera mode, one native screenshot each ----
        for mode in CAMERA_MODES:
            t.hud(step=f"camera {mode}")
            if mode == "target":
                # frame the NAO together with the kitchen fridge area (a real furniture point)
                r = t.camera("target", target_xy=(3.9, 6.5))
            else:
                r = t.camera(mode)
            ok = bool(r.get("ok"))
            t.screenshot(f"camera_{mode}.png", mode=mode)
            results["camera"][mode] = {"ok": ok, "detail": r.get("reason") or r.get("builtin") or r.get("mode"),
                                       "pose_after": list(_pose_tuple(t.session))}
            time.sleep(0.2)

        # ---- zoom: closeup then dolly in and out ----
        t.hud(step="camera closeup + zoom in")
        t.camera("closeup")
        t.screenshot("zoom_base.png", mode="closeup")
        zi = t.camera("closeup", zoom="in")
        t.screenshot("zoom_in.png", mode="closeup")
        zo = t.camera("closeup", zoom="out")
        t.screenshot("zoom_out.png", mode="closeup")
        results["camera"]["zoom"] = {"in_ok": bool(zi.get("zoom_result", {}).get("ok", zi.get("ok"))),
                                     "out_ok": bool(zo.get("zoom_result", {}).get("ok", zo.get("ok")))}

        # ---- prove the robot never moved while the camera did ----
        t.camera("overview")
        pose_after = _pose_tuple(t.session)
        results["camera"]["pose_after_all_modes"] = list(pose_after)
        moved = ((pose_after[0] - pose_before[0]) ** 2 + (pose_after[1] - pose_before[1]) ** 2) ** 0.5
        results["robot_moved_during_camera"] = round(moved, 4)

        # ---- human model framing + a dedicated human close-up for the model evidence ----
        if human_node is not None:
            t.hud(step="realistic human close-up")
            hr = t.camera("human")
            t.screenshot("human_model.png", mode="human")
            # a tighter human look via a numeric zoom-in on the human framing
            t.camera("human", zoom=0.6)
            t.screenshot("human_model_closeup.png", mode="human")
            results["human"]["framed_ok"] = bool(hr.get("ok"))
            results["human"]["position"] = results["world"].get("human_position")
        else:
            results["human"]["framed_ok"] = False
            results["human"]["note"] = "no ORBIT_HUMAN node in the loaded world; load orbit_house_human_realistic.wbt to validate the human model"

        # verdict: all seven modes ok AND the robot did not move while the camera changed
        all_modes_ok = all(results["camera"][m]["ok"] for m in CAMERA_MODES if m != "human") and \
            (results["camera"]["human"]["ok"] or human_node is None)

        def verdict(_s):
            return all_modes_ok and results["robot_moved_during_camera"] < 0.02
        t.set_verdict(pass_if=verdict,
                      measure=lambda s: {"modes_ok": all_modes_ok, "robot_moved": results["robot_moved_during_camera"]},
                      condition="all camera modes ok AND robot did not move (<0.02 m) during camera changes")
        t.set_metrics({"camera_modes": len(CAMERA_MODES), "human_present": human_node is not None})

    # write a Phase-16 summary next to the trial evidence
    out_dir = os.path.join(RESULTS, run_name)
    with open(os.path.join(out_dir, "phase16_camera_summary.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, default=str)
    print(json.dumps(results, indent=2, default=str))
    print(f"\nEvidence: {os.path.join(out_dir, 'camera')}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", default="phase16_visual_clarity_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    ap.add_argument("--start-room", default="living_room")
    a = ap.parse_args(argv)
    return run(a.run, a.start_room)


if __name__ == "__main__":
    sys.exit(main())
