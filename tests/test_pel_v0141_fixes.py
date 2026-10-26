"""v0.14.1: one regression test per finding of the v0.14.0 code review (CODE_REVIEW_v0.14.0.md). Each test is built from the matching
probe in review_probes.py and fails on v0.14.0. The robot is the real World2D; the model is a scripted test double, so nothing here is
a real-LLM result."""
import csv, datetime, json, os, sys, tempfile, unittest
from orbit_ros.world2d import World2D, Obj
from orbit_ros.tasks2d import Task, Phase
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import LLMBackend
from orbit_ros.llm import ScriptedClient
from orbit_ros.pel.manager import ExecutionManager, PelFlags
from orbit_ros.pel.registry import RegistryContext
from orbit_ros.pel.state import ExecutionState, accumulated_no_go, pending_failure
from orbit_ros.pel.guard import SafetyGuard
from orbit_ros.pel.schema import parse_plan, errors
from orbit_ros.pel import cost

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class Script:
    def __init__(self, plans, done=True): self.plans, self.done, self.tags = list(plans), done, []
    def __call__(self, prompt, tag):
        self.tags.append(tag)
        if tag.startswith("assess"): return json.dumps({"done": self.done})
        return self.plans.pop(0) if len(self.plans) > 1 else self.plans[0]

class Crash(Exception): pass

def v1(steps, **kw): return json.dumps({"schema": "orbit.plan/v1", "steps": steps, **kw})
def legacy(*acts): return json.dumps({"actions": [{"tool": t, "args": a} for t, a in acts]})
def spy(world):
    calls, real = [], world.execute
    def execute(tool, args): calls.append((tool, json.loads(json.dumps(args, default=str)))); return real(tool, args)
    world.execute = execute; return calls
def robot(calls): return [c for c in calls if c[0] not in ("get_pose", "describe_view")]
def task(text="drive"): return Task("fix|motion", "motion", [Phase(text, "motion", {})], {}, {"final_pose": (0, 0, 0)})
def run(w, plans, cfg="PEL", done=True, **kw):
    sc = Script(plans, done); ev = []
    row = run_episode(w, kw.pop("task", None) or task(), PRESETS[cfg], LLMBackend(ScriptedClient(sc), False, False), ev, **kw)
    return row, ev, sc

def manager(w, sc, d, budget, cfg=PelFlags(), rep=0):
    b = LLMBackend(ScriptedClient(sc), False, False)
    def observe(ps, fresh):
        return {"pose": w.execute("get_pose", {})["data"], "raw": w.execute("describe_view", {})["data"], "interp": [],
                "obstacles_known": [], "landmarks": {}, "cleaned_known": set(w.cleaned), "start_pose": w.start_pose, "phase_start_pose": ps,
                "history": [], "use_memory": False, "reactive": False, "grounded": False, "sensor_spec": None, "embodiment_card": None,
                "fresh_phase": fresh, "reported": [], "clarifications": [], "nav_tools": False, "localizer": "none", "pose_stale": False,
                "lessons": [], "lesson_effects": set()}
    def do(a, ctx):
        if budget["n"] >= budget["cap"]: raise Crash("process killed before the next robot call")
        budget["n"] += 1; return w.execute(a["tool"], a["args"])
    return ExecutionManager(cfg, b, observe, do, lambda *a, **k: None, lambda: True, lambda: RegistryContext(w.stopped),
                            lambda: w.stopped, w.start_pose, episode="fix", config="PEL", state_dir=d, rep=rep)

def crashing(m, ph):
    try: m.run_phase(ph, (1, 1, 0), 0)
    except Crash: pass

def only_state(d):
    names = [n for n in os.listdir(d) if n.endswith(".json")]; assert len(names) == 1, names
    return ExecutionState.load(os.path.join(d, names[0]))

PH = Phase("drive", "motion", {})

class ResumeTests(unittest.TestCase):
    def _nogo_crash(self):
        """Revision 0 declares (4,1) no-go and runs out; revision 1 declares nothing and drives across (4,1). The process dies after
        revision 1's first step."""
        w = World2D(12, 7, (1, 1), 0, []); calls = spy(w); d = tempfile.mkdtemp()
        p_a = v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 180}}], constraints={"no_go_cells": [[4, 1]]})
        p_b = v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 180}}, {"id": 2, "tool": "describe_view", "args": {}},
                  {"id": 3, "tool": "move_forward", "args": {"distance": 4}}])
        crashing(manager(w, Script([p_a, p_b], done=False), d, {"n": 0, "cap": 2}), PH)
        return w, calls, d

    def test_h2_no_go_from_an_earlier_revision_survives_resume(self):
        w, calls, d = self._nogo_crash(); st = only_state(d)
        self.assertEqual(st.no_go_cells, [[4, 1]])                       # stored in the exec state
        m2 = manager(w, Script([legacy()]), d, {"n": 0, "cap": 99}); m2.run_phase(PH, (1, 1, 0), 0, resume=st)
        self.assertIn((4, 1), m2.no_go)
        self.assertNotIn(("move_forward", {"distance": 4}), robot(calls))
        self.assertNotIn((4, 1), w.visited); self.assertGreaterEqual(m2.c["pel_vetoes"], 1)

    def test_h2_v0140_record_without_stored_union_is_rebuilt_from_all_revisions(self):
        w, calls, d = self._nogo_crash(); st = only_state(d)
        old = st.to_dict(); [old.pop(k) for k in ("no_go_cells", "cleaned_at_start", "visited_cells")]   # a v0.14.0 record
        st_old = ExecutionState.from_dict(old)
        self.assertEqual(accumulated_no_go(st_old), {(4, 1)})
        m2 = manager(w, Script([legacy()]), d, {"n": 0, "cap": 99}); m2.run_phase(PH, (1, 1, 0), 0, resume=st_old)
        self.assertNotIn(("move_forward", {"distance": 4}), robot(calls)); self.assertNotIn((4, 1), w.visited)

    def test_m1_failed_step_is_handled_not_resent_after_resume(self):
        w = World2D(12, 7, (1, 1), 0, [Obj("o1", "red box", 5, 1)]); calls = spy(w); d = tempfile.mkdtemp()
        plan = legacy(("move_forward", {"distance": 1}), ("move_forward", {"distance": 5}), ("turn", {"direction": "left", "degrees": 90}))
        class CrashOnRepair(Script):
            def __call__(self, prompt, tag):
                if tag.startswith("repair"): raise Crash("killed while the model writes the repair")
                return super().__call__(prompt, tag)
        try: manager(w, CrashOnRepair([plan]), d, {"n": 0, "cap": 99}).run_phase(PH, (1, 1, 0), 0)
        except Crash: pass
        st = only_state(d); self.assertEqual(st.attempts[-1]["status"], "blocked"); self.assertIsNotNone(pending_failure(st))
        hits = w.collisions
        sc = Script([legacy()]); m2 = manager(w, sc, d, {"n": 0, "cap": 99}); m2.run_phase(PH, (1, 1, 0), 0, resume=st)
        self.assertEqual(robot(calls).count(("move_forward", {"distance": 5})), 1)      # not sent a second time
        self.assertEqual(w.collisions, hits)
        self.assertTrue(any(t.startswith("repair") for t in sc.tags))               # the failure went to a re-plan

    def test_m1_on_failure_stop_is_honoured_on_resume(self):
        w = World2D(12, 7, (1, 1), 0, [Obj("o1", "red box", 4, 1)]); calls = spy(w); d = tempfile.mkdtemp()
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 5}, "on_failure": "stop"}])
        class CrashAfterCall(Script): pass
        m = manager(w, Script([p]), d, {"n": 0, "cap": 99})
        real_save = m._save; n = {"k": 0}
        def save():                                               # die right after the failed attempt was saved
            real_save()
            if m.state.attempts and m.state.attempts[-1]["status"] != "ok": raise Crash("killed")
        m._save = save
        crashing(m, PH); st = only_state(d)
        m2 = manager(w, Script([legacy()]), d, {"n": 0, "cap": 99}); m2.run_phase(PH, (1, 1, 0), 0, resume=st)
        self.assertEqual(m2.state.terminal["code"], "safe_stop"); self.assertIn("on_failure is stop", m2.state.terminal["reason"])
        self.assertEqual(len(robot(calls)), 1)

class PlanGateTests(unittest.TestCase):
    def test_m2_plan_still_invalid_after_fixes_is_not_executed(self):
        w = World2D(9, 7, (1, 1), 0, []); calls = spy(w)
        bad = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 1}}, {"id": 1, "tool": "move_forward", "args": {"distance": 1}}])
        row, ev, sc = run(w, [bad])
        self.assertEqual(robot(calls), [])
        self.assertEqual((row["pel_plan_errors"], row["pel_terminal"]), (4, "safe_stop"))
        self.assertTrue(any(e["event"] == "pel_stop" and "plan invalid" in e["details"] for e in ev))

    def test_m2_malformed_no_go_cells_are_a_plan_error_not_silently_dropped(self):
        plan, issues = parse_plan(json.loads(v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}}],
                                                constraints={"no_go_cells": [[4, "1"]]})))
        self.assertTrue(any(i.where.startswith("constraints.no_go_cells") for i in errors(issues)))

    def test_m2_gate_holds_with_replan_off(self):
        w = World2D(9, 7, (1, 1), 0, []); calls = spy(w)
        bad = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 1}}, {"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}}])
        row, ev, sc = run(w, [bad], cfg="PEL-replan")
        self.assertEqual(robot(calls), []); self.assertEqual(row["pel_terminal"], "safe_stop")

class V1FieldTests(unittest.TestCase):
    def test_m3_pre_pose_unmet_blocks_the_step(self):
        w = World2D(9, 7, (1, 1), 0, []); calls = spy(w)
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 1}, "pre": {"pose": {"x": 7, "y": 5}}}])
        row, ev, sc = run(w, [p, legacy(("turn", {"direction": "left", "degrees": 90}))])
        self.assertNotIn(("move_forward", {"distance": 1}), robot(calls))
        self.assertEqual(row["pel_pre_failures"], 1); self.assertIn("repair", sc.tags[1])

    def test_m3_pre_that_holds_lets_the_step_run(self):
        w = World2D(9, 7, (1, 1), 0, []); calls = spy(w)
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 1}, "pre": {"pose": {"x": 1, "y": 1}, "robot_stopped": False}}])
        row, ev, sc = run(w, [p])
        self.assertEqual(robot(calls), [("move_forward", {"distance": 1})]); self.assertEqual(row["pel_pre_failures"], 0)

    def test_m3_on_failure_stop_ends_the_instruction(self):
        w = World2D(9, 7, (1, 1), 0, [Obj("o1", "red box", 3, 1)]); calls = spy(w)
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 1}, "pre": {"pose": {"x": 7, "y": 5}}, "on_failure": "stop"}])
        row, ev, sc = run(w, [p, legacy(("turn", {"direction": "left", "degrees": 90}))])
        self.assertEqual(robot(calls), []); self.assertEqual(row["pel_terminal"], "safe_stop")
        self.assertEqual([t for t in sc.tags if not t.startswith("assess")], ["plan"])     # no re-plan

    def test_m3_on_failure_stop_after_a_veto(self):
        w = World2D(9, 7, (1, 1), 180, []); calls = spy(w)
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 3}, "on_failure": "stop"}])       # would leave the room
        row, ev, sc = run(w, [p, legacy(("turn", {"direction": "left", "degrees": 90}))])
        self.assertEqual(robot(calls), []); self.assertEqual((row["pel_vetoes"], row["pel_terminal"]), (1, "safe_stop"))

    def test_m3_completion_cleaned_min_is_verified(self):
        w = World2D(9, 7, (1, 1), 0, [])
        row, ev, sc = run(w, [v1([{"id": 1, "tool": "clean", "args": {}}], completion={"cleaned_min": 5})])
        self.assertNotEqual(row["pel_terminal"], "done"); self.assertGreaterEqual(row["pel_verify_failures"], 1)
        w = World2D(9, 7, (1, 1), 0, [])
        row, ev, sc = run(w, [v1([{"id": 1, "tool": "clean", "args": {}}], completion={"cleaned_min": 1})])
        self.assertEqual((row["pel_terminal"], row["pel_verify_failures"]), ("done", 0))

    def test_m3_completion_visited_is_verified(self):
        w = World2D(9, 7, (1, 1), 0, [])
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 3}}], completion={"visited": [[3, 1]]})
        row, ev, sc = run(w, [p]); self.assertEqual((row["pel_terminal"], row["pel_verify_failures"]), ("done", 0))
        w = World2D(9, 7, (1, 1), 0, [])
        p = v1([{"id": 1, "tool": "move_forward", "args": {"distance": 3}}], completion={"visited": [[1, 3]]})
        row, ev, sc = run(w, [p]); self.assertGreaterEqual(row["pel_verify_failures"], 1)

    def test_m3_completion_reported_and_expect_cleaned_cell(self):
        w = World2D(9, 7, (1, 1), 0, [])
        row, ev, sc = run(w, [v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}}], completion={"reported": True})])
        self.assertGreaterEqual(row["pel_verify_failures"], 1)
        w = World2D(9, 7, (1, 1), 0, [])
        p = v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}, "expect": {"cleaned_cell": True}}])
        row, ev, sc = run(w, [p, legacy()]); self.assertGreaterEqual(row["pel_verify_failures"], 1)

    def test_m3_max_steps_and_stay_in_room(self):
        steps = [{"id": i, "tool": "turn", "args": {"direction": "left", "degrees": 90}} for i in (1, 2, 3)]
        _, issues = parse_plan(json.loads(v1(steps, constraints={"max_steps": 2})))
        self.assertTrue(any(i.where == "constraints.max_steps" for i in errors(issues)))
        _, issues = parse_plan(json.loads(v1(steps, constraints={"max_steps": 3, "stay_in_room": False})))
        self.assertEqual(errors(issues), []); self.assertTrue(any(i.where == "constraints.stay_in_room" for i in issues))

    def test_m4_macro_verification_uses_the_no_go_cells(self):
        w = World2D(9, 7, (1, 1), 0, [])
        p = v1([{"id": 1, "tool": "goto_cell", "args": {"x": 4, "y": 1}}], constraints={"no_go_cells": [[2, 1]]})
        row, ev, sc = run(w, [p, legacy()], nav_tools=True)
        self.assertEqual((w.x, w.y), (4, 1)); self.assertNotIn((2, 1), w.visited)
        self.assertEqual((row["pel_verify_failures"], row["pel_replans"]), (0, 0))

class TerminalAndAskTests(unittest.TestCase):
    def test_l1_terminal_codes(self):
        cases = (("unsupported", v1([], status="unsupported", unsupported_reason="unsafe")),
                 ("needs_clarification", v1([], status="needs_clarification", clarification="which box?")),
                 ("abstained", v1([{"id": 1, "tool": "abstain", "args": {"reason": "unsafe"}}])))
        for code, p in cases:
            w = World2D(9, 7, (1, 1), 0, []); row, ev, sc = run(w, [p])
            self.assertEqual(row["pel_terminal"], code, code)

    def test_l4_clarification_goes_through_the_registry_and_is_recorded(self):
        w = World2D(9, 7, (1, 1), 0, []); d = tempfile.mkdtemp()
        row, ev, sc = run(w, [v1([], status="needs_clarification", clarification="which box?")], pel_state_dir=d)
        st = only_state(d)
        asks = [a for a in st.attempts if a["call"]["tool"] == "ask"]
        self.assertTrue(asks)                                                    # recorded in the exec state
        self.assertEqual(asks[-1]["status"], "rejected")                        # the last one was refused by asks_left
        self.assertEqual(st.terminal["code"], "needs_clarification")
        self.assertLessEqual(len([a for a in asks if a["status"] != "rejected"]), 2)

class GuardTests(unittest.TestCase):
    def test_l2_guard_fails_closed_on_malformed_distances(self):
        g = SafetyGuard(); pose = {"x": 4, "y": 3, "yaw_deg": 0, "room_w": 9, "room_h": 7}
        for v in (-3, 3.0, "3", None, True):
            self.assertIsNotNone(g.check({"tool": "move_forward", "args": {"distance": v}}, pose, []), repr(v))
        self.assertIsNone(g.check({"tool": "move_forward", "args": {"distance": 0}}, pose, []))
        self.assertIsNone(g.check({"tool": "move_forward", "args": {"distance": 3}}, pose, []))

    def test_l2_malformed_distance_never_reaches_the_simulator_without_registry(self):
        w = World2D(9, 7, (4, 3), 0, []); calls = spy(w)
        row, ev, sc = run(w, [legacy(("move_forward", {"distance": -2})), legacy(("turn", {"direction": "left", "degrees": 90}))], cfg="PEL-registry")
        self.assertNotIn(("move_forward", {"distance": -2}), robot(calls)); self.assertEqual(row["invalid_calls"], 0)

class MiscTests(unittest.TestCase):
    def test_l5_self_consistency_keys_differ_for_v1_plans(self):
        from orbit_ros.ensemble import _key
        ctx = {"pose": {"x": 1, "y": 1, "yaw_deg": 0, "room_w": 9, "room_h": 7}, "obstacles_known": [], "start_pose": (1, 1, 0)}
        a = json.loads(v1([{"id": 1, "tool": "move_forward", "args": {"distance": 3}}]))
        b = json.loads(v1([{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}}]))
        self.assertNotEqual(_key(a, ctx), _key(b, ctx))
        self.assertEqual(_key({"actions": a["steps"]}, ctx), _key(a, ctx))       # same calls -> same key as the legacy form

    def test_l6_state_files_of_repeats_do_not_overwrite_each_other(self):
        d = tempfile.mkdtemp()
        for rep in (0, 1):
            w = World2D(9, 7, (1, 1), 0, [])
            manager(w, Script([legacy(("turn", {"direction": "left", "degrees": 90}))]), d, {"n": 0, "cap": 99}, rep=rep).run_phase(PH, (1, 1, 0), 0)
        self.assertEqual(sorted(os.listdir(d)), ["fix#PEL#r0#p0.json", "fix#PEL#r1#p0.json"])

    def test_l7_price_validity(self):
        price = {"provider": "p", "model": "m", "input": 1, "output": 1, "source_url": "u", "retrieved": "2026-10-18", "valid_until": "2026-12-31"}
        cost.check_valid(price, datetime.date(2026, 12, 31))
        with self.assertRaises(cost.PricingError): cost.check_valid(price, datetime.date(2027, 1, 1))
        cost.check_valid({k: v for k, v in price.items() if k != "valid_until"}, datetime.date(2030, 1, 1))
        d = tempfile.mkdtemp(); p = os.path.join(d, "pricing.json")
        with open(p, "w") as f: json.dump({"currency": "USD", "models": [dict(price, valid_until="31/12/2026")]}, f)
        with self.assertRaises(cost.PricingError): cost.load_pricing(p)
        shipped = cost.load_pricing(cost.default_pricing_path())
        self.assertTrue(all("valid_until" in e for e in shipped["entries"].values()))

    def test_m6_safety_contrasts_are_holm_corrected(self):
        sys.path.insert(0, os.path.join(ROOT, "tools")); sys.path.insert(0, ROOT)
        import pel_arms
        d = tempfile.mkdtemp(); cols = ["task_id", "config", "rep", "family", "success", "outcome_code", "safety_violations", "invalid_calls", "llm_calls"]
        rows = []
        for i in range(40):
            for cfg, sv in (("C4", 2 if i % 2 else 1), ("C5", 1 if i % 3 else 0), ("PEL", 0)):
                rows.append({"task_id": f"t{i}", "config": cfg, "rep": "0", "family": "goto", "success": 1, "outcome_code": "success",
                             "safety_violations": sv, "invalid_calls": 0, "llm_calls": 3})
        with open(os.path.join(d, "trial_results.csv"), "w", newline="") as f:
            wr = csv.DictWriter(f, cols); wr.writeheader(); wr.writerows(rows)
        text, out = pel_arms.report(d)
        k4, k5 = out["C4->PEL"]["counts"]["safety_violations"], out["C5->PEL"]["counts"]["safety_violations"]
        ps = sorted([k4["p"], k5["p"]])
        self.assertAlmostEqual(min(k4["p_holm"], k5["p_holm"]), min(1.0, 2 * ps[0]))
        self.assertIn("p (Holm) |", text.split("Wilcoxon p")[1].splitlines()[0])

if __name__ == "__main__": unittest.main()
