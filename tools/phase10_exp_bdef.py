r"""Phase 10 experiments B (learn from failure), D (restart persistence), E (stale memory override), F (retrieval/robustness).

Each live trial runs in its own fresh Supervisor process (one per --worker call), direct on tcp://127.0.0.1:1235 (never 1240).
Real Groq LLM. The shared memory db is passed with --mem-db so persistence spans processes.

  .venv\Scripts\python.exe tools\phase10_exp_bdef.py --exp B --mem-db <db> --out <dir> --provider groq
  (internal) --worker <kind> <mem_db> <provider> <out> <tag>     # one trial; kind in: seed_fail, retest, persist_probe, stale

Honesty rules enforced here:
- The FAILURE in B is genuine: a no-go zone is set so the planner truly cannot route to the bedroom (go_to_room -> rejected "no safe path").
- The retest removes the zone; memory is enabled; we record whether the prior failure episode was RETRIEVED and whether the task now
  succeeds (observations override stale memory). We never preload invented successes.
"""
from __future__ import annotations
import argparse, json, os, subprocess, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
def _port_up(port=1235):
    import socket
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1): return True
    except OSError: return False
def ensure_webots():
    if _port_up(): return True
    world = os.path.join(ROOT, "webots", "worlds", "orbit_house.wbt")
    subprocess.Popen([WEBOTS, "--mode=realtime", "--minimize", "--batch", "--port=1235", world])
    for _ in range(90):
        time.sleep(2)
        if _port_up(): time.sleep(3); return True
    return False

# --------------------------------------------------------------------------- worker: one trial, fresh process
def worker(kind, mem_db, provider, out, tag):
    from orbit_ros.humanoid import HumanoidSession
    from orbit_ros.nao_memory import NaoMemory
    tr = {"kind": kind, "tag": tag, "events": [], "statuses": [], "llm_calls": 0, "llm_latency_s": 0.0, "tokens": {}}

    if kind == "seed_fail":
        # Genuine controlled failure: block the bedroom with a no-go zone so the planner truly cannot route there.
        sess = HumanoidSession(sim="webots", log_dir=False, memory=NaoMemory(mem_db))
        sess.load_scene("living_room")
        # no-go zones across the bedroom doorway/entry so A* finds no safe path
        for (zx, zy) in ((6.0, 2.0), (6.5, 2.0), (7.0, 2.0)):
            sess.set_no_go_zone(zx, zy, 0.7)
        r = sess.robot_call("go_to_room", {"room": "bedroom"})
        p = sess._pose()
        tr["statuses"].append(r["status"]); tr["result_msg"] = r["message"][:160]
        tr["final_pose"] = {k: p[k] for k in ("x", "y", "yaw_deg", "room")}
        tr["genuine_failure"] = r["status"] in ("rejected", "failed", "vetoed") and p["room"] != "bedroom"
        # record the failed episode with explicit feedback (provenance = this seed trial)
        wb = sess.record_episode("Go to the bedroom.", {"status": r["status"], "message": r["message"], "_tool": "go_to_room"},
                                 experiment="B", scenario="goto_bedroom_blocked",
                                 feedback="bedroom was unreachable: a no-go zone blocked the only route (go_to_room rejected)",
                                 provenance={"experiment": "B", "tag": tag})
        tr["write_back"] = wb
        print("P10B_JSON " + json.dumps(tr, default=str)); return

    if kind == "retest":
        # Same goal, NO zone now (observations differ). Memory enabled: does the LLM retrieve the prior failure, and does it now succeed?
        _run_llm_goal("Go to the bedroom.", provider, mem_db, tr, want_room="bedroom")
        print("P10B_JSON " + json.dumps(tr, default=str)); return

    if kind == "persist_probe":
        # Experiment D: fresh process reads the shared db and uses a retrieved strategy to drive the task.
        _run_llm_goal("Go to the kitchen.", provider, mem_db, tr, want_room="kitchen")
        print("P10B_JSON " + json.dumps(tr, default=str)); return

    if kind == "stale":
        # Experiment E: seed a (now stale) 'strategy' then set the environment so the live observation differs; verify observations win
        # (the robot still only executes guard-approved motions; memory cannot force an unsafe/invalid action).
        m = NaoMemory(mem_db)
        m.learn_strategy("go:bedroom", tool="go_to", args={"x": 6.0, "y": 2.0}, success=True, sim="webots",
                         note="STALE: a point that is now inside a no-go zone", provenance={"experiment": "E", "tag": tag})
        sess = HumanoidSession(sim="webots", log_dir=False, memory=m)
        sess.load_scene("living_room")
        sess.set_no_go_zone(6.0, 2.0, 0.8)                         # the stale strategy's target is now unsafe
        tb = sess.task_memory(instruction="Go to the bedroom.")
        tr["retrieved_stale"] = any(s.get("args") == {"x": 6.0, "y": 2.0} for s in (tb.get("strategies") or []))
        # execute the stale strategy's exact action -> the guard/planner must refuse or route around it (observation overrides memory)
        r = sess.robot_call("go_to", {"x": 6.0, "y": 2.0})
        p = sess._pose()
        tr["statuses"].append(r["status"]); tr["result_msg"] = r["message"][:160]
        tr["observation_overrode_memory"] = (r["status"] in ("rejected", "vetoed", "failed")) or (abs(p["x"]-6.0) > 0.8 or abs(p["y"]-2.0) > 0.8)
        tr["ended_in_stale_zone"] = (abs(p["x"]-6.0) <= 0.6 and abs(p["y"]-2.0) <= 0.6)
        print("P10B_JSON " + json.dumps(tr, default=str)); return

def _run_llm_goal(instruction, provider, mem_db, tr, want_room=None):
    import urllib.request, urllib.error
    from orbit_ros.humanoid import HumanoidSession
    from orbit_ros.nao_memory import NaoMemory
    from orbit_ros.env_config import load_env
    from orbit_ros.llm import BASE_URLS, KEY_VARS
    from orbit_ros.mcp_server import HumanoidMcpServer
    load_env()
    base = dict(BASE_URLS, gemini="https://generativelanguage.googleapis.com/v1beta/openai")
    keyv = dict(KEY_VARS, gemini="GEMINI_API_KEY")
    key = os.getenv(keyv[provider], ""); model = os.getenv(provider.upper() + "_MODEL", "")
    sess = HumanoidSession(sim="webots", log_dir=False, memory=NaoMemory(mem_db)); sess.load_scene("living_room")
    tr["llm_model"] = f"{provider}:{model}"
    if not key: tr["skipped"] = f"no key for {provider}"; return
    tb = sess.task_memory(instruction=instruction)
    tr["retrieved"] = {"episodes": tb.get("episodes"), "strategies": tb.get("strategies")}
    tr["retrieved_count"] = len(tb.get("episodes") or []) + len(tb.get("strategies") or [])
    tr["retrieved_prior_failure"] = any(e.get("success") is False for e in (tb.get("episodes") or []))
    srv = HumanoidMcpServer(sess, "p10"); tool_defs = srv.tool_defs(); known = {t["name"] for t in tool_defs}
    tools = [{"type": "function", "function": {"name": t["name"], "description": t.get("description", ""),
              "parameters": t.get("inputSchema") or {"type": "object", "properties": {}}}} for t in tool_defs]
    url = base[provider].rstrip("/") + "/chat/completions"
    sysmsg = srv.instructions + "\n"
    user = (tb["brief"] + "\n" + instruction) if tb.get("brief") else instruction
    messages = [{"role": "system", "content": sysmsg}, {"role": "user", "content": user}]
    def ask(mr=5):
        body = json.dumps({"model": model, "messages": messages, "tools": tools, "temperature": 0, "max_tokens": 4096}).encode()
        for att in range(mr + 1):
            req = urllib.request.Request(url, data=body, method="POST", headers={"Authorization": "Bearer " + key, "Content-Type": "application/json", "User-Agent": "orbit-phase10/1"})
            t = time.time()
            try:
                with urllib.request.urlopen(req, timeout=90) as resp: data = json.loads(resp.read().decode())
            except urllib.error.HTTPError as e:
                if e.code in (403, 429, 500, 502, 503, 504) and att < mr: time.sleep(min(30, 2 ** att)); continue
                raise
            tr["llm_calls"] += 1; tr["llm_latency_s"] += time.time() - t
            for k in ("prompt_tokens", "completion_tokens", "total_tokens"):
                if k in (data.get("usage") or {}): tr["tokens"][k] = tr["tokens"].get(k, 0) + data["usage"][k]
            return data["choices"][0]["message"]
    empty = 0
    for _ in range(12):
        try: msg = ask()
        except Exception as e: tr["llm_error"] = f"{type(e).__name__}: {e}"; break
        calls = msg.get("tool_calls") or []; content = msg.get("content") or ""
        messages.append({"role": "assistant", "content": content, **({"tool_calls": calls} if calls else {})})
        if not calls:
            if content.strip(): tr["llm_final_text"] = content[:200]; break
            empty += 1
            if empty > 2: break
            messages.append({"role": "user", "content": "Call one robot tool now, or say you are done."}); continue
        empty = 0
        for tc in calls:
            name = tc["function"]["name"]
            try: args = json.loads(tc["function"].get("arguments") or "{}")
            except json.JSONDecodeError: args = {}
            if not isinstance(args, dict): args = {}
            if name not in known: payload = {"status": "error", "message": "unknown tool"}
            else:
                try: payload, _ = srv.call_tool(name, args)
                except (ValueError, TypeError) as e: payload = {"status": "error", "message": f"bad arguments: {e}"}
            if isinstance(payload, dict) and payload.get("status"):
                tr["statuses"].append(payload["status"]); tr["events"].append({"tool": name, "status": payload["status"]})
            messages.append({"role": "tool", "tool_call_id": tc.get("id", name), "content": json.dumps(payload, default=str)[:2000]})
    p = sess._pose(); tr["final_pose"] = {k: p[k] for k in ("x", "y", "yaw_deg", "room")}
    if want_room is not None: tr["success"] = p["room"] == want_room and not p["fallen"]
    # write the (now real) outcome back
    sess.record_episode(instruction, {"status": tr["statuses"][-1] if tr["statuses"] else "error",
                        "message": tr.get("final_pose"), "_tool": next((e["tool"] for e in tr["events"] if e["tool"].startswith("go")), None)},
                        experiment="B/D", scenario="retest", provenance={"tag": tr.get("tag")})

def run(kind, mem_db, provider, out, tag):
    if not ensure_webots(): return {"kind": kind, "error": "webots down"}
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", kind, mem_db, provider, out, tag],
                       capture_output=True, text=True, cwd=ROOT, timeout=700)
    line = [l for l in p.stdout.splitlines() if l.startswith("P10B_JSON ")]
    tr = json.loads(line[-1][len("P10B_JSON "):]) if line else {"kind": kind, "error": "no json", "stderr": p.stderr[-300:]}
    with open(os.path.join(out, f"expB_{kind}_{tag}.trace.json"), "w", encoding="utf-8") as f: json.dump(tr, f, indent=2, default=str)
    return tr

def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "--worker": return worker(argv[1], argv[2], argv[3], argv[4], argv[5])
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", required=True); ap.add_argument("--mem-db", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--provider", default="groq")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    results = {}
    if a.exp == "B":
        print("=== Experiment B: learn from a genuine failure ===")
        results["seed_fail"] = run("seed_fail", a.mem_db, a.provider, a.out, "b1")
        print("seed_fail: genuine_failure =", results["seed_fail"].get("genuine_failure"), "| msg:", results["seed_fail"].get("result_msg"))
        results["retest"] = run("retest", a.mem_db, a.provider, a.out, "b2")
        rt = results["retest"]
        print("retest: retrieved_prior_failure =", rt.get("retrieved_prior_failure"), "| success =", rt.get("success"),
              "| llm_calls =", rt.get("llm_calls"), "| final_room =", (rt.get("final_pose") or {}).get("room"))
    elif a.exp == "D":
        print("=== Experiment D: restart persistence ===")
        results["persist_probe"] = run("persist_probe", a.mem_db, a.provider, a.out, "d1")
        pr = results["persist_probe"]
        print("persist_probe: retrieved_count =", pr.get("retrieved_count"), "| success =", pr.get("success"),
              "| final_room =", (pr.get("final_pose") or {}).get("room"))
    elif a.exp == "E":
        print("=== Experiment E: stale/conflicting memory overridden by observation ===")
        results["stale"] = run("stale", a.mem_db, a.provider, a.out, "e1")
        st = results["stale"]
        print("stale: retrieved_stale =", st.get("retrieved_stale"), "| observation_overrode_memory =", st.get("observation_overrode_memory"),
              "| ended_in_stale_zone =", st.get("ended_in_stale_zone"), "| status:", st.get("statuses"))
    with open(os.path.join(a.out, f"exp{a.exp}_summary.json"), "w", encoding="utf-8") as f: json.dump(results, f, indent=2, default=str)
    print("SAVED exp", a.exp, "->", a.out)
    return 0

if __name__ == "__main__": sys.exit(main())
