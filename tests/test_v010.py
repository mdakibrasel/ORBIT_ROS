"""v0.10: paper arms C1-C4, paper metrics, robust LLM parsing, harder instructions, wording variants, runner (resume/estimate), preflight."""
import contextlib, io, json, os, re, shutil, subprocess, sys, tempfile, threading, unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from orbit_ros.tasks2d import make_task, HARD_FAMILIES, ALL_FAMILIES, visible_ids, _visible_from
from orbit_ros.agent2d import run_episode, PRESETS, S_LADDER, C_LADDER, config_by_name
from orbit_ros.backends2d import OracleBackend, LLMBackend, _extract_json
from orbit_ros.llm import ScriptedClient
from orbit_ros.phrasing import rephrase, STYLES
import run_spatial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def motion_plan(task):
    acts = []
    for op, v in task.phases[0].params["ops"]:
        acts.append({"tool": "move_forward", "args": {"distance": v}} if op == "forward" else
                    {"tool": "turn", "args": {"direction": "left", "degrees": 180}} if v == "around" else {"tool": "turn", "args": {"direction": v, "degrees": 90}})
    return acts

def pad_next(prompt):
    """The step the scratchpad marks as next ('->'), or None when all planned steps are done."""
    m = re.search(r"^\s*-> \d+: (.*)$", prompt, re.M)
    return json.loads(m.group(1)) if m else None

def scripted(task, wrong_step=None, garbage_plans=0):
    """A model that knows the right plan for a motion task. wrong_step: what it proposes instead of the planned step (gate/deviation tests)."""
    plan = motion_plan(task); st = {"i": 0, "garbage": garbage_plans}
    def fn(prompt, tag):
        if tag in ("plan", "plan_step", "repair", "plan_retry"):
            if st["garbage"] > 0: st["garbage"] -= 1; return "Sure! Here is my plan: move then turn"
            if tag == "plan_step":       # C2: remaining work only
                return json.dumps({"actions": plan[st["i"]:]})
            return json.dumps({"actions": plan})
        if tag == "step":                # C1 / S0
            if st["i"] >= len(plan): return '{"tool":"done","args":{}}'
            a = plan[st["i"]]; st["i"] += 1; return json.dumps(a)
        if tag == "scratch_step":
            nxt = pad_next(prompt)
            if nxt is None: return '{"tool":"done","args":{}}'
            if wrong_step is not None: return json.dumps(wrong_step)
            return json.dumps(nxt)
        if tag == "assess": return '{"done": true}'
        raise AssertionError(tag)
    return fn

class PaperArmTests(unittest.TestCase):
    def run_llm(self, name, fn, task_args=("motion", 0, 0, 1, False, 2)):
        world, task = make_task(*task_args)
        cfg = PRESETS[name]; client = ScriptedClient(fn(task) if callable(fn) and fn.__name__ == "<lambda>" else fn)
        return run_episode(world, task, cfg, LLMBackend(client, cfg.grounded, cfg.memory), []), client, task

    def test_c_arms_exist_and_differ_only_in_plan_handling(self):
        for n in C_LADDER:
            c = PRESETS[n]; self.assertEqual((c.grounded, c.memory, c.verify, c.repair), (False, False, False, False)); self.assertTrue(c.stepwise)
        self.assertEqual([PRESETS[n].exec_mode for n in C_LADDER[:4]], ["reactive", "transient", "scratchpad", "constrained"])   # paper arms
        self.assertEqual((PRESETS["C5"].exec_mode, PRESETS["C5"].feedback), ("constrained", True))                              # v0.13 extension
        self.assertFalse(PRESETS["C1"].persistent); self.assertFalse(PRESETS["C2"].persistent); self.assertTrue(PRESETS["C3"].persistent)
        self.assertEqual(PRESETS["S1"].exec_mode, "plan"); self.assertEqual(PRESETS["S0"].exec_mode, "reactive")

    def test_every_c_arm_completes_a_motion_task_through_the_llm_prompt_path(self):
        for n in C_LADDER:
            world, task = make_task("motion", 0, 0, 1, False, 2)
            cfg = PRESETS[n]; client = ScriptedClient(scripted(task))
            if n == "C2":         # the scripted C2 model needs to know how far it got: count executed steps via the prompt history length
                plan = motion_plan(task)
                def fn(prompt, tag, plan=plan):
                    done = prompt.count('"ok": true')
                    return json.dumps({"actions": plan[done:]})
                client = ScriptedClient(fn)
            row = run_episode(world, task, cfg, LLMBackend(client, cfg.grounded, cfg.memory), [])
            self.assertEqual(row["success"], 1, (n, row)); self.assertEqual(row["invalid_calls"], 0)
            k = len(motion_plan(task))
            self.assertGreaterEqual(row["llm_calls"], k, n)          # every C arm asks the model at least once per executed tool call
            self.assertTrue(all(t in ("plan", "plan_step", "step", "scratch_step") for t, _ in client.prompts), n)

    def test_c4_gate_rejects_off_plan_calls_and_executes_the_plan(self):
        world, task = make_task("motion", 0, 0, 1, False, 2)
        wrong = {"tool": "turn", "args": {"direction": "left", "degrees": 270}}
        row = run_episode(world, task, PRESETS["C4"], LLMBackend(ScriptedClient(scripted(task, wrong_step=wrong)), False, False), [])
        self.assertEqual(row["success"], 1); self.assertEqual(row["gate_rejections"], len(motion_plan(task))); self.assertEqual(row["plan_deviations"], 0)

    def test_c3_lets_the_model_deviate_and_records_it(self):
        world, task = make_task("motion", 0, 0, 1, False, 2)
        wrong = {"tool": "turn", "args": {"direction": "left", "degrees": 90}}
        row = run_episode(world, task, PRESETS["C3"], LLMBackend(ScriptedClient(scripted(task, wrong_step=wrong)), False, False), [])
        self.assertEqual(row["success"], 0); self.assertGreater(row["plan_deviations"], 0); self.assertEqual(row["gate_rejections"], 0)

    def test_scratchpad_prompt_shows_the_plan_and_the_constraint(self):
        world, task = make_task("motion", 0, 0, 1, False, 1); client = ScriptedClient(scripted(task))
        run_episode(world, task, PRESETS["C4"], LLMBackend(client, False, False), [])
        p = next(p for t, p in client.prompts if t == "scratch_step")
        self.assertIn("SCRATCHPAD", p); self.assertIn("EXECUTION CONSTRAINT", p); self.assertIn("-> 0:", p)
        world, task = make_task("motion", 0, 0, 1, False, 1); client = ScriptedClient(scripted(task))
        run_episode(world, task, PRESETS["C3"], LLMBackend(client, False, False), [])
        self.assertNotIn("EXECUTION CONSTRAINT", next(p for t, p in client.prompts if t == "scratch_step"))

    def test_oracle_runs_every_c_arm_on_every_family(self):
        for fam in ALL_FAMILIES:
            for n in C_LADDER:
                w, t = make_task(fam, 0, 90, 3 if fam not in ("remember_return",) else 2, False, 1)
                row = run_episode(w, t, PRESETS[n], OracleBackend(0.05, 0.3, 0.1)); self.assertIn(row["success"], (0, 1))
        # without deviations the gate never fires, so C3 and C4 make exactly the same calls (same name -> same interpreter noise stream)
        import dataclasses
        for fam in ("motion", "goto", "clean_side", "sequence"):
            w1, t1 = make_task(fam, 1, 180, 3, False, 1); w2, t2 = make_task(fam, 1, 180, 3, False, 1)
            a = run_episode(w1, t1, PRESETS["C3"], OracleBackend(0, 0, 0))
            b = run_episode(w2, t2, dataclasses.replace(PRESETS["C4"], name="C3"), OracleBackend(0, 0, 0))
            self.assertEqual(b["gate_rejections"], 0)
            self.assertEqual((a["success"], a["final_x"], a["final_y"], a["actions_executed"]), (b["success"], b["final_x"], b["final_y"], b["actions_executed"]))

    def test_s4_variants(self):
        self.assertEqual(config_by_name("S4v2").verify_retries, 2); self.assertFalse(config_by_name("S4v2").repair)
        with self.assertRaises(KeyError): config_by_name("S9")

class MetricTests(unittest.TestCase):
    def test_repeated_calls_are_identical_calls_from_an_unchanged_state(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        replies = iter(['{"tool":"move_forward","args":{"distance":12}}'] + ['{"tool":"move_forward","args":{"distance":12}}'] * 3 + ['{"tool":"done","args":{}}'])
        row = run_episode(w, t, PRESETS["S0"], LLMBackend(ScriptedClient(lambda p, tag: next(replies)), False, False), [])
        # call 1 drives to the wall, call 2 is a new attempt from the new cell, calls 3-4 repeat call 2 from the same state
        self.assertEqual(row["actions_executed"], 4); self.assertEqual(row["repeated_calls"], 2); self.assertEqual(row["repeated_call_pct"], 50.0)
    def test_turning_twice_is_not_a_repeat(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        replies = iter(['{"tool":"turn","args":{"direction":"left","degrees":90}}'] * 2 + ['{"tool":"done","args":{}}'])
        row = run_episode(w, t, PRESETS["S0"], LLMBackend(ScriptedClient(lambda p, tag: next(replies)), False, False))
        self.assertEqual(row["repeated_calls"], 0)
    def test_invalid_pct_and_time_columns(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        replies = iter(['{"tool":"fly","args":{}}', '{"tool":"turn","args":{"direction":"up","degrees":90}}', '{"tool":"done","args":{}}'])
        row = run_episode(w, t, PRESETS["S0"], LLMBackend(ScriptedClient(lambda p, tag: next(replies)), False, False))
        self.assertEqual(row["invalid_calls"], 2); self.assertEqual(row["invalid_call_pct"], 100.0)
        for k in ("elapsed_s", "llm_latency_s", "harness_s"): self.assertIn(k, row)

class ParsingTests(unittest.TestCase):
    def test_extract_json_variants(self):
        self.assertEqual(_extract_json('```json\n{"a":1}\n```'), {"a": 1})
        self.assertEqual(_extract_json('Plan: {"actions": []} done.'), {"actions": []})
        with self.assertRaises(ValueError): _extract_json("no json here")
    def test_unparseable_plans_do_not_crash_the_episode(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        row = run_episode(w, t, PRESETS["S1"], LLMBackend(ScriptedClient(scripted(t, garbage_plans=2)), False, False))
        self.assertEqual(row["parse_errors"], 2); self.assertNotIn("error", row)
    def test_unparseable_step_is_an_invalid_call(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        replies = iter(["hmm", "still not json", '{"tool":"done","args":{}}'])
        row = run_episode(w, t, PRESETS["C1"], LLMBackend(ScriptedClient(lambda p, tag: next(replies)), False, False))
        self.assertEqual(row["invalid_calls"], 1); self.assertEqual(row["parse_errors"], 2)
    def test_malformed_plan_shapes(self):
        from orbit_ros.agent2d import normalize_plan
        self.assertEqual(normalize_plan([{"tool": "clean"}])["actions"], [{"tool": "clean", "args": {}}])
        self.assertEqual(normalize_plan({"actions": "go"})["actions"], [])
        self.assertEqual(normalize_plan({"actions": [{"tool": "turn", "args": "left"}]})["actions"][0]["tool"], "__malformed__")

class HardInstructionTests(unittest.TestCase):
    def grid(self): return list(run_spatial.select_instances(list(HARD_FAMILIES), 1, "hard", 0))
    def test_noise_free_full_agent_solves_almost_everything(self):
        ok = tot = 0
        for fam, s, yaw, n, m, lv in self.grid():
            w, t = make_task(fam, s, yaw, n, m, lv); ok += run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0, ask_p=1.0, abstain=True))["success"]; tot += 1
        self.assertGreaterEqual(ok / tot, 0.95, (ok, tot))
    def test_ambiguous_door_frame_swaps_left_and_right(self):
        w, t = make_task("ambiguous", 0, 90, 3, False, 2)
        o0, o1 = w.objects[0], w.objects[1]
        self.assertEqual(t.expected["target_id"], "o1")                   # door is in the wall the robot faces -> door-left = robot-right
        from orbit_ros.world2d import to_robot_frame
        self.assertGreater(to_robot_frame(o0.x - w.x, o0.y - w.y, w.yaw)[1], 0); self.assertLess(to_robot_frame(o1.x - w.x, o1.y - w.y, w.yaw)[1], 0)
    def test_conditional_visibility_matches_the_level(self):
        for yaw in (0, 90, 180, 270):
            for lv in (1, 2):
                w, t = make_task("conditional", 0, yaw, 3, False, lv)
                self.assertEqual("o0" in visible_ids(w), lv == 1)
    def test_report_truth_and_visibility(self):
        for lv in (1, 2):
            w, t = make_task("report", 0, 0, 5, False, lv)
            self.assertEqual(_visible_from(w.w, w.h, (w.x, w.y), w.objects), {o.id for o in w.objects})
            w.execute("report", {"answer": f"There are {t.expected['answer']} of them"})
            from orbit_ros.tasks2d import check_success
            self.assertTrue(check_success(w, t)["success"])
    def test_change_interrupts_after_one_action(self):
        w, t = make_task("change", 0, 0, 3, False, 1); ev = []
        row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0), ev)
        names = [e["event"] for e in ev]
        self.assertIn("instruction_interrupted", names)
        first = names.index("instruction_interrupted"); self.assertEqual(sum(1 for n in names[:first] if n == "action"), 1)
        self.assertEqual(row["success"], 1)
    def test_prohibit_counts_violations(self):
        w, t = make_task("prohibit", 0, 0, 1, False, 2)
        from orbit_ros.tasks2d import check_success
        w.visited.add(tuple(t.expected["forbidden"][0])); self.assertEqual(check_success(w, t)["violations"], 1)

class PhrasingTests(unittest.TestCase):
    def test_every_instruction_has_every_wording_and_the_task_is_unchanged(self):
        for style in STYLES:
            for fam, s, yaw, n, m, lv in run_spatial.select_instances(list(ALL_FAMILIES), 1, "all", 0):
                w, t = make_task(fam, s, yaw, n, m, lv); params = [dict(p.params) for p in t.phases]; texts = [p.text for p in t.phases]
                rephrase(t, style)
                self.assertEqual(params, [p.params for p in t.phases])
                if style != "canonical": self.assertTrue(all(a != b for a, b in zip(texts, [p.text for p in t.phases])), (fam, lv, style))
        w, t = make_task("goto", 0, 0, 1, False, 1); self.assertIn("লাল বাক্স", rephrase(t, "bn").phases[0].text)

class EnvTests(unittest.TestCase):
    def test_empty_value_followed_by_a_comment_is_empty(self):
        from orbit_ros.env_config import parse_env
        e = parse_env("A=   # comment\nB=x # c\nC='#notacomment'\n"); self.assertEqual((e["A"], e["B"], e["C"]), ("", "x", "#notacomment"))
        e = parse_env(open(os.path.join(ROOT, ".env.example"), encoding="utf-8").read())
        self.assertTrue(all(not v.startswith("#") for v in e.values()), e)

class RunnerTests(unittest.TestCase):
    def setUp(self): self.d = tempfile.mkdtemp()
    def tearDown(self): shutil.rmtree(self.d, ignore_errors=True)
    def test_run_resume_estimate_and_report(self):
        out = os.path.join(self.d, "r")
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            run_spatial.main(["--suite", "all", "--seeds", "1", "--limit", "4", "--configs", "S1", "C4", "--out", out, "--events"])
        rows = [json.loads(l) for l in open(os.path.join(out, "trial_results.jsonl"))]; self.assertEqual(len(rows), 8)
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            run_spatial.main(["--suite", "all", "--seeds", "1", "--limit", "4", "--configs", "S1", "C4", "C3", "--out", out, "--resume"])
        rows = [json.loads(l) for l in open(os.path.join(out, "trial_results.jsonl"))]; self.assertEqual(len(rows), 12)   # only C3 was added
        buf = io.StringIO()
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(buf):
            run_spatial.main(["--planner", "groq", "--suite", "all", "--seeds", "1", "--limit", "4", "--configs", "C4", "--estimate", "--out", out])
        self.assertIn("model calls", buf.getvalue())
        import analyze_spatial
        with contextlib.redirect_stdout(io.StringIO()): analyze_spatial.main(out)
        rep = open(os.path.join(out, "REPORT.md"), encoding="utf-8").read()
        self.assertIn("Paper metrics", rep); self.assertIn("Cost per success", rep); self.assertIn("C1 → C4", rep) if "C1" in rep else None
    def test_budget_guard_stops_cleanly(self):
        out = os.path.join(self.d, "b")
        with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
            run_spatial.main(["--suite", "core", "--seeds", "1", "--limit", "10", "--configs", "S0", "--max-llm-calls", "50", "--out", out])
        meta = json.load(open(os.path.join(out, "run_metadata.json"))); self.assertIsNotNone(meta["stopped_early"])
        self.assertLess(meta["episodes_written"], 10)

class Smart(BaseHTTPRequestHandler):
    """Fake OpenAI-compatible endpoint that answers the preflight's prompts sensibly."""
    def do_GET(self):
        self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
        self.wfile.write(json.dumps({"data": [{"id": "openai/gpt-oss-120b"}]}).encode())
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0)))); p = body["messages"][-1]["content"]
        if "2+3" in p: txt = '{"ok": true, "sum": 5}'
        elif "SCRATCHPAD" in p: nxt = pad_next(p); txt = json.dumps(nxt) if nxt else '{"tool":"done","args":{}}'
        elif "fully completed" in p: txt = '{"done": true}'
        else:
            m = re.search(r"Current instruction: Move forward (\d+) meters?, then turn (left|right)\.", p)
            txt = json.dumps({"actions": [{"tool": "move_forward", "args": {"distance": int(m.group(1))}}, {"tool": "turn", "args": {"direction": m.group(2), "degrees": 90}}]})
        self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
        self.wfile.write(json.dumps({"choices": [{"message": {"content": txt}, "finish_reason": "stop"}], "usage": {"prompt_tokens": 100, "completion_tokens": 20}}).encode())
    def log_message(self, *a): pass

class PreflightTests(unittest.TestCase):
    def test_preflight_against_a_fake_groq(self):
        srv = HTTPServer(("127.0.0.1", 0), Smart); threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            env = dict(os.environ, GROQ_API_KEY="TEST", GROQ_BASE_URL=f"http://127.0.0.1:{srv.server_port}/v1", PYTHONPATH=ROOT)
            r = subprocess.run([sys.executable, os.path.join(ROOT, "preflight_llm.py"), "--planner", "groq", "--model", "openai/gpt-oss-120b"],
                               capture_output=True, text=True, env=env, cwd=ROOT, timeout=120)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr); self.assertIn("FOUND", r.stdout); self.assertIn("C4: ", r.stdout)
            self.assertNotIn("TEST", r.stdout.replace("[preflight]", ""))
        finally: srv.shutdown(); srv.server_close()

if __name__ == "__main__": unittest.main()
