r"""Phase 10 — live evaluation of the persistent MEMORY module with the real LLM in the real Webots house.

Transport: direct Webots extern controller on tcp://127.0.0.1:1235 (never 1240). One Supervisor process per trial.
LLM: real provider (default groq). Memory: orbit_ros/nao_memory.py (episodic + semantic), a SQLite db shared ACROSS trials via --mem-db.

The LLM request REALLY receives retrieved task memory (when enabled): before planning, the runner calls task_memory(instruction)
and prepends the returned brief to the user turn. After a task it writes a verified episode + strategy. Disabled mode retrieves/writes
nothing. Each trial records: condition (mem on/off), retrieved memories (provenance), LLM calls/tokens/latency, true start/final pose,
success (judged from TRUE pose), replans, repeated-error flag.

    .venv\Scripts\python.exe tools\phase10_experiment.py --exp A --provider groq --mem-db <path>
    .venv\Scripts\python.exe tools\phase10_experiment.py --worker <scenario> <memflag on|off> <provider> <mem_db> <experiment>

Experiments: A (disabled vs enabled), B (learn-from-failure), C (reuse-strategy), D (restart persistence), E (stale memory), F (retrieval/robustness).
"""
from __future__ import annotations
import argparse, csv, glob, json, math, os, subprocess, sys, time, datetime
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros import house

def item_gap(label, x, y):
    best = 1e9
    for it in house.ITEMS:
        if it["label"] != label: continue
        best = min(best, math.hypot(max(abs(x-it["x"])-it["sx"]/2, 0.0), max(abs(y-it["y"])-it["sy"]/2, 0.0)))
    return best

# Scenarios used by Phase 10 (navigation tasks with objective, true-pose success tests).
SCN = {
    "goto_kitchen":  {"start": "living_room", "instruction": "Go to the kitchen.",            "success": ("in_room", "kitchen")},
    "goto_bedroom":  {"start": "living_room", "instruction": "Walk to the bedroom.",          "success": ("in_room", "bedroom")},
    "goto_bathroom": {"start": "living_room", "instruction": "Go to the bathroom.",           "success": ("in_room", "bathroom")},
    "goto_fridge":   {"start": "living_room", "instruction": "Go and stand next to the fridge.", "success": ("near_object", ("fridge", 1.5))},
}

def judge(kind, param, sess):
    p = sess._pose(); x, y = p["x"], p["y"]
    if kind == "in_room": return p["room"] == param, {"room": p["room"], "want": param}
    if kind == "near_object":
        name, rad = param; gap = item_gap(name, x, y); return gap <= rad and not p["fallen"], {"gap_m": round(gap, 2), "radius": rad}
    return False, {"error": "unknown"}

# --------------------------------------------------------------------------- worker: ONE trial in its own process
def worker(scenario, memflag, provider, mem_db, experiment):
    sc = SCN[scenario]
    from orbit_ros.humanoid import HumanoidSession
    from orbit_ros.nao_memory import NaoMemory
    mem_on = memflag == "on"
    memory = NaoMemory(mem_db) if mem_on else False
    sess = HumanoidSession(sim="webots", log_dir=False, memory=memory)
    sess.load_scene(sc["start"])
    tr = {"experiment": experiment, "scenario": scenario, "memory": memflag, "provider": provider,
          "instruction": sc["instruction"], "events": [], "statuses": [], "actions": 0, "replans": 0,
          "llm_calls": 0, "llm_latency_s": 0.0, "tokens": {}, "retrieved": None, "retrieved_count": 0,
          "mem_episodes_before": None, "mem_strategies_before": None}
    sp = sess._pose(); tr["start_pose"] = {k: sp[k] for k in ("x", "y", "yaw_deg", "room")}
    t0 = time.time()
    if mem_on:
        tr["mem_episodes_before"] = memory.count_episodes(); tr["mem_strategies_before"] = memory.count_strategies()
    _run_llm(sc, provider, sess, tr, mem_on)
    p = sess._pose(); ok, detail = judge(*sc["success"], sess=sess)
    tr["final_pose"] = {k: p[k] for k in ("x", "y", "yaw_deg", "room")} | {"fallen": p["fallen"]}
    tr["success"] = bool(ok); tr["success_detail"] = detail; tr["wall_s"] = round(time.time() - t0, 2)
    # repeated-error detection: did this trial repeat a tool+status that a retrieved FAILED episode warned about?
    tr["repeated_prior_error"] = _repeated_error(tr)
    # write-back (episodic + semantic) after a verified outcome, when memory + write are enabled
    if mem_on and getattr(memory, "write_after_task", True):
        wb = sess.record_episode(sc["instruction"], {"status": tr["statuses"][-1] if tr["statuses"] else "error",
                                 "message": detail, "_tool": tr.get("chosen_tool")},
                                 experiment=experiment, scenario=scenario,
                                 provenance={"experiment": experiment, "scenario": scenario, "trial_t": tr["start_pose"]})
        tr["write_back"] = wb
        tr["mem_episodes_after"] = memory.count_episodes(); tr["mem_strategies_after"] = memory.count_strategies()
    print("P10_TRIAL_JSON " + json.dumps(tr, default=str))

def _repeated_error(tr):
    ret = tr.get("retrieved") or {}
    failed_tools = {e.get("executed", [{}])[0].get("tool") for e in (ret.get("episodes") or []) if e.get("success") is False and e.get("executed")}
    used_bad = any(ev.get("tool") in failed_tools and ev.get("status") in ("failed", "vetoed", "error") for ev in tr["events"])
    return bool(failed_tools) and used_bad

def _run_llm(sc, provider, sess, tr, mem_on):
    import urllib.request, urllib.error
    from orbit_ros.env_config import load_env
    from orbit_ros.llm import BASE_URLS, KEY_VARS
    from orbit_ros.mcp_server import HumanoidMcpServer
    load_env()
    base = dict(BASE_URLS, gemini="https://generativelanguage.googleapis.com/v1beta/openai")
    keyv = dict(KEY_VARS, gemini="GEMINI_API_KEY")
    key = os.getenv(keyv[provider], ""); model = os.getenv(provider.upper() + "_MODEL", "")
    if not key: tr["skipped"] = f"no API key for {provider}"; return
    tr["llm_model"] = f"{provider}:{model}"
    srv = HumanoidMcpServer(sess, "phase10")
    tool_defs = srv.tool_defs(); known = {t["name"] for t in tool_defs}
    tools = [{"type": "function", "function": {"name": t["name"], "description": t.get("description", ""),
              "parameters": t.get("inputSchema") or {"type": "object", "properties": {}}}} for t in tool_defs]
    url = base[provider].rstrip("/") + "/chat/completions"
    sys_prompt = srv.instructions + ("\n"
                 "When done, reply in words. Keep tool calls minimal.")
    user = sc["instruction"]
    if mem_on:                                                     # REAL retrieval injected into the REAL model request
        tb = sess.task_memory(instruction=sc["instruction"])
        tr["retrieved"] = {"episodes": tb.get("episodes"), "strategies": tb.get("strategies"), "goal": tb.get("goal")}
        tr["retrieved_count"] = len(tb.get("episodes") or []) + len(tb.get("strategies") or [])
        if tb.get("brief"):
            user = tb["brief"] + "\n" + sc["instruction"]
    messages = [{"role": "system", "content": sys_prompt}, {"role": "user", "content": user}]
    def ask(max_retries=5):
        body = json.dumps({"model": model, "messages": messages, "tools": tools, "temperature": 0, "max_tokens": 4096}).encode()
        for attempt in range(max_retries + 1):
            req = urllib.request.Request(url, data=body, method="POST",
                                        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json", "User-Agent": "p10/1"})
            t = time.time()
            try:
                with urllib.request.urlopen(req, timeout=90) as resp: data = json.loads(resp.read().decode())
            except urllib.error.HTTPError as e:
                if e.code in (403, 429, 500, 502, 503, 504) and attempt < max_retries:
                    ra = e.headers.get("retry-after") if hasattr(e, "headers") else None
                    time.sleep(float(ra) if ra and str(ra).replace(".", "", 1).isdigit() else min(30, 2 ** attempt)); continue
                raise
            tr["llm_calls"] += 1; tr["llm_latency_s"] += time.time() - t
            u = data.get("usage") or {}
            for k in ("prompt_tokens", "completion_tokens", "total_tokens"):
                if k in u: tr["tokens"][k] = tr["tokens"].get(k, 0) + u[k]
            return data["choices"][0]["message"]
    def record(name, args, payload):
        st = payload.get("status", "error"); tr["statuses"].append(st); tr["actions"] += 1
        tr["events"].append({"tool": name, "args": args, "status": st, "message": str(payload.get("message", ""))[:160]})
        if name in ("go_to_room", "go_to_object", "go_to", "join_tv_watching"): tr["chosen_tool"] = name
        tr["replans"] = sess.replans
    empty = 0
    for _ in range(12):
        try: msg = ask()
        except Exception as e: tr["llm_error"] = f"{type(e).__name__}: {e}"; break
        calls = msg.get("tool_calls") or []; content = msg.get("content") or ""
        messages.append({"role": "assistant", "content": content, **({"tool_calls": calls} if calls else {})})
        if not calls:
            if content.strip(): tr["llm_final_text"] = content[:300]; break
            empty += 1
            if empty > 2: break
            messages.append({"role": "user", "content": "Call exactly one robot tool now, or state you are done."}); continue
        empty = 0
        for tc in calls:
            name = tc["function"]["name"]
            try: args = json.loads(tc["function"].get("arguments") or "{}")
            except json.JSONDecodeError: args = {}
            if not isinstance(args, dict): args = {}
            if name not in known: payload, _ = {"status": "error", "message": f"unknown tool {name}"}, True
            else:
                try: payload, _ = srv.call_tool(name, args)
                except (ValueError, TypeError) as e: payload = {"status": "error", "message": f"bad arguments: {e}"}
            if isinstance(payload, dict) and payload.get("status"): record(name, args, payload)
            messages.append({"role": "tool", "tool_call_id": tc.get("id", name), "content": json.dumps(payload, default=str)[:2000]})

# --------------------------------------------------------------------------- parent orchestration
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

def run_trial(scenario, memflag, provider, mem_db, experiment, out):
    if not ensure_webots(): return {"scenario": scenario, "memory": memflag, "error": "webots down", "success": None}
    p = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", scenario, memflag, provider, mem_db, experiment],
                       capture_output=True, text=True, cwd=ROOT, timeout=700)
    line = [l for l in p.stdout.splitlines() if l.startswith("P10_TRIAL_JSON ")]
    if not line:
        tr = {"scenario": scenario, "memory": memflag, "experiment": experiment, "success": None, "error": "no trial json",
              "stderr_tail": p.stderr[-300:]}
    else:
        tr = json.loads(line[-1][len("P10_TRIAL_JSON "):])
    with open(os.path.join(out, f"{experiment}__{scenario}__mem_{memflag}.trace.json"), "w", encoding="utf-8") as f:
        json.dump(tr, f, indent=2, default=str)
    s = tr.get("success"); mark = "PASS" if s is True else ("SKIP" if s is None else "FAIL")
    print(f"[{mark}] {experiment} {scenario} mem={memflag}: {tr.get('success_detail') or tr.get('error')} | "
          f"actions={tr.get('actions')} retrieved={tr.get('retrieved_count')} repeated_err={tr.get('repeated_prior_error')} "
          f"llm_calls={tr.get('llm_calls')} {tr.get('wall_s')}s", flush=True)
    return tr

def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "--worker":
        return worker(argv[1], argv[2], argv[3], argv[4], argv[5])
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="A", help="which experiment block to run (A/B/C/E/F or 'all')")
    ap.add_argument("--provider", default="groq")
    ap.add_argument("--mem-db", default=None, help="shared memory db for enabled trials (default: a fresh phase10 db)")
    ap.add_argument("--out", default=None, help="phase10 output dir (default: a new one)")
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "results_nao_benchmark", "phase10_memory_" + time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(out, exist_ok=True)
    mem_db = a.mem_db or os.path.join(out, "phase10_memory.db")
    trials = []
    def do(exp, scenario, memflag): trials.append(run_trial(scenario, memflag, a.provider, mem_db, exp, out))

    if a.exp in ("A", "all"):
        # Experiment A: disabled vs enabled on the SAME task (enabled retrieves whatever the DB holds).
        for sc in ("goto_kitchen", "goto_bedroom"):
            do("A", sc, "off"); do("A", sc, "on")
    if a.exp in ("B", "all"):
        # Experiment B is driven by the dedicated seeding+retest script phase10_exp_B.py (see docs). Here we just note it.
        print("Experiment B: run tools/phase10_exp_B.py (controlled failure -> retest).")
    _finalize(trials, out, mem_db, a.provider)
    return 0

def _finalize(trials, out, mem_db, provider):
    cols = ["experiment", "scenario", "memory", "success", "actions", "replans", "retrieved_count", "repeated_prior_error",
            "llm_calls", "llm_latency_s", "total_tokens", "wall_s", "note"]
    rows = []
    for tr in trials:
        rows.append({"experiment": tr.get("experiment"), "scenario": tr.get("scenario"), "memory": tr.get("memory"),
                     "success": tr.get("success"), "actions": tr.get("actions"), "replans": tr.get("replans"),
                     "retrieved_count": tr.get("retrieved_count"), "repeated_prior_error": tr.get("repeated_prior_error"),
                     "llm_calls": tr.get("llm_calls"), "llm_latency_s": round(tr.get("llm_latency_s", 0) or 0, 2),
                     "total_tokens": (tr.get("tokens") or {}).get("total_tokens"), "wall_s": tr.get("wall_s"),
                     "note": tr.get("skipped") or tr.get("llm_error") or tr.get("error") or ""})
    with open(os.path.join(out, "trials.csv"), "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        if f.tell() == 0: w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out, "trials.jsonl"), "a", encoding="utf-8") as f:
        for tr in trials: f.write(json.dumps(tr, default=str) + "\n")
    print("\n", out, "\n", mem_db)

if __name__ == "__main__": sys.exit(main())
