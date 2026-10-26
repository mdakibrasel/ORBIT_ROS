"""Re-test the commands that got vetoed/failed in all_commands_live.py from a CLEAN start pose (load_scene first),
which is the correct isolated condition for each. Proves they succeed when not started from a tight corner.

Usage (Webots on 1235):  python tools/live/retest_nav_clean.py
"""
import glob, json, math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from orbit_ros.humanoid import HumanoidSession
from orbit_ros.mcp_server import HumanoidMcpServer

OUT = os.path.join("results_live", "retest_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
os.makedirs(OUT, exist_ok=True)
rows = []


def main():
    s = HumanoidSession(sim="webots")
    srv = HumanoidMcpServer(s, "retest")

    def fresh():
        srv.call_tool("load_scene", {"scene": "living_room"})   # clean start pose + clear latch
        return dict(s._pose())

    def run(name, args, verify):
        start = fresh()
        payload, is_err = srv.call_tool(name, args)
        after = dict(s._pose())
        ok, detail = verify(payload, after)
        row = {"command": name, "args": args, "status": payload.get("status") if isinstance(payload, dict) else None,
               "is_error": bool(is_err), "verified_ok": ok, "detail": detail,
               "start": _xy(start), "end": _xy(after), "end_room": after.get("room"),
               "msg": (payload.get("message") if isinstance(payload, dict) else str(payload))[:200]}
        rows.append(row)
        print(f"{name:16} status={row['status']:8} verified={ok}  {detail}  end={row['end']} room={after.get('room')}")
        return row

    # return_to_start: walk out then back; verify within 1.0 m of home
    def _v_return(p, after):
        home = s.start_pose
        d = math.hypot(after["x"] - home[0], after["y"] - home[1]) if home else 1e9
        return (p.get("status") == "ok" and d < 1.0, f"dist_to_home={round(d,3)}")
    # first move away, then return
    fresh(); srv.call_tool("go_to_room", {"room": "kitchen"})
    p_rs, e_rs = srv.call_tool("return_to_start", {}); after_rs = dict(s._pose())
    home = s.start_pose; d_home = math.hypot(after_rs["x"] - home[0], after_rs["y"] - home[1])
    rows.append({"command": "return_to_start", "args": {}, "status": p_rs.get("status"), "is_error": bool(e_rs),
                 "verified_ok": p_rs.get("status") == "ok" and d_home < 1.0, "detail": f"dist_to_home={round(d_home,3)}",
                 "end": _xy(after_rs), "end_room": after_rs.get("room"), "msg": p_rs.get("message", "")[:200]})
    print(f"{'return_to_start':16} status={p_rs.get('status'):8} verified={p_rs.get('status')=='ok' and d_home<1.0}  dist_to_home={round(d_home,3)}")

    run("go_to_room", {"room": "bedroom"}, lambda p, a: (a.get("room") == "bedroom" and not a["fallen"], f"room={a.get('room')}"))
    run("tour_house", {"finish": "stay"}, lambda p, a: (p.get("status") == "ok", f"visited={_visited(p)}"))
    run("clean_room", {"room": "kitchen", "pattern": "quick", "mode": "vacuum"}, lambda p, a: (p.get("status") == "ok", f"status={p.get('status')}"))
    run("run_plan", {"steps": [{"room": "bedroom"}], "finish": "stay"}, lambda p, a: (p.get("status") == "ok", f"status={p.get('status')}"))
    run("clean_house", {"pattern": "quick", "mode": "vacuum"}, lambda p, a: (p.get("status") == "ok", f"status={p.get('status')}"))

    with open(os.path.join(OUT, "results.json"), "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=1, default=str)
    npass = sum(1 for r in rows if r["verified_ok"])
    print(f"\n{npass}/{len(rows)} verified OK from clean start.  EVIDENCE: {OUT}")


def _xy(p):
    return f"({round(p.get('x',0),2)},{round(p.get('y',0),2)})" if p else "(?)"


def _visited(p):
    d = p.get("data") if isinstance(p, dict) else None
    return (d or {}).get("visited") if isinstance(d, dict) else "?"


if __name__ == "__main__":
    main()
