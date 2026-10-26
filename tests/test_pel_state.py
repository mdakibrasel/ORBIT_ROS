"""ORBIT-PEL M3: persistent execution state (schema, update rules, atomic save, stale plans, safe resume after an interruption)."""
import json, os, tempfile, unittest
from orbit_ros.world2d import World2D, Obj, Realism
from orbit_ros.tasks2d import make_task
from orbit_ros.navtools import bfs, path_actions
from orbit_ros.pel.schema import from_legacy
from orbit_ros.pel.state import ExecutionState, fingerprint, stale_reason, resume_blocker, progress_key, SCHEMA

def observe(w, known=()):
    """What the robot reports: pose estimate and camera view (two robot calls), never w.x / w.y."""
    pose = w.execute("get_pose", {})["data"]; view = w.execute("describe_view", {})["data"]
    return fingerprint(pose, view, known, set(), w.stopped)

def drive_plan(w, goal):
    p = w.pose(); path = bfs((p["x"], p["y"]), {goal}, {(o.x, o.y) for o in w.objects}, w.w, w.h)
    acts, _ = path_actions(p["yaw_deg"], path, frozenset({goal}))
    return from_legacy({"actions": acts, "expect": {"x": goal[0], "y": goal[1]}}, plan_id="t").to_dict()

def run(w, plan, path=None, stop_after=None, st=None):
    """Execute plan steps one at a time, saving the state after each step. stop_after=k simulates a crash after k steps."""
    if st is None:
        st = ExecutionState(episode="e", config="PEL"); st.start_plan(plan, "initial", observe(w))
    while st.current_step() is not None:
        if stop_after is not None and st.cursor >= stop_after: return st
        s = st.current_step(); r = w.execute(s["tool"], s["args"])
        st.record_attempt(s["id"], {"tool": s["tool"], "args": s["args"]}, "ok" if r["ok"] else "blocked", r["message"], 0.0, observe(w)); st.advance()
        if path: st.save(path)
    st.terminate("done"); return st

class SchemaAndRulesTests(unittest.TestCase):
    def test_round_trip_and_version(self):
        w, _ = make_task("goto", 0, 90, 3, False, 1); st = run(w, drive_plan(w, (1, 1)))
        d = json.loads(json.dumps(st.to_dict())); self.assertEqual(d["schema"], SCHEMA)
        self.assertEqual(ExecutionState.from_dict(d).to_dict(), st.to_dict())
        with self.assertRaises(ValueError): ExecutionState.from_dict({**d, "schema": "orbit.exec_state/v0"})
        with self.assertRaises(ValueError): ExecutionState.from_dict({**d, "surprise": 1})
    def test_update_rules(self):
        st = ExecutionState(); st.start_plan({"steps": [{"id": 1, "tool": "clean", "args": {}}, {"id": 2, "tool": "clean", "args": {}}]}, "initial", None)
        fp = fingerprint({"x": 1, "y": 1, "yaw_deg": 0})
        st.record_attempt(1, {"tool": "clean", "args": {}}, "ok", "cleaned", 0.01, fp); st.advance()
        self.assertEqual((st.cursor, st.completed_steps, st.budgets["tool_calls"]), (1, [1], 1))
        st.record_attempt(2, {"tool": "move_forward", "args": {"distance": 99}}, "rejected", "bad_value", 0.0, None)
        self.assertEqual((st.budgets["tool_calls"], st.budgets["rejections"]), (1, 1))
        st.start_plan({"steps": []}, "step 2 rejected", None)
        self.assertEqual((st.revision, st.cursor, st.completed_steps, st.budgets["replans"]), (1, 0, [], 1))
        st.terminate("done"); st.terminate("error")
        self.assertEqual(st.terminal["code"], "done")
        with self.assertRaises(RuntimeError): st.start_plan({"steps": []}, "x", None)
        with self.assertRaises(ValueError): ExecutionState().terminate("finished-ish")
    def test_no_progress_and_duplicates(self):
        st = ExecutionState(); st.start_plan({"steps": []}, "initial", None)
        fp = fingerprint({"x": 1, "y": 1, "yaw_deg": 0}); call = {"tool": "move_forward", "args": {"distance": 0}}
        st.fingerprint = fp                                       # the state the first call is issued from (set by the manager)
        st.record_attempt(1, call, "ok", "", 0, fp); self.assertTrue(st.is_duplicate(call))
        for _ in range(3): st.record_attempt(1, call, "ok", "", 0, fp)
        self.assertEqual(st.no_progress_steps, 4)             # all four calls left the state unchanged
        st.record_attempt(1, call, "ok", "", 0, fingerprint({"x": 2, "y": 1, "yaw_deg": 0}))
        self.assertEqual(st.no_progress_steps, 0); self.assertFalse(st.is_duplicate({"tool": "turn", "args": {}}))
    def test_duplicates_are_keyed_on_the_state_before_the_call(self):
        """A turn from yaw 0 to yaw 90 followed by the same turn from yaw 90 is progress, not a duplicate (v0.14 M4 fix)."""
        st = ExecutionState(); st.start_plan({"steps": []}, "initial", None)
        turn = {"tool": "turn", "args": {"direction": "left", "degrees": 90}}
        st.fingerprint = fingerprint({"x": 1, "y": 1, "yaw_deg": 0})
        st.record_attempt(1, turn, "ok", "", 0, fingerprint({"x": 1, "y": 1, "yaw_deg": 90}))
        self.assertFalse(st.is_duplicate(turn))
        st2 = ExecutionState(); st2.start_plan({"steps": []}, "initial", None)          # no recorded pre-call state: nothing is keyed
        st2.record_attempt(1, turn, "ok", "", 0, fingerprint({"x": 1, "y": 1, "yaw_deg": 90})); self.assertEqual(st2.seen_calls, [])
    def test_fingerprint_uses_reported_pose_only(self):
        w = World2D(9, 3, (0, 1), 0, []); w.set_realism(Realism(odom_p=1.0, seed="x")); w.execute("move_forward", {"distance": 3})
        fp = observe(w); self.assertEqual(fp["pose"][:2], [w.pose()["x"], w.pose()["y"]]); self.assertNotEqual(fp["pose"][0], w.x)

class AtomicSaveTests(unittest.TestCase):
    def test_atomic_write_leaves_no_partial_file(self):
        d = tempfile.mkdtemp(); p = os.path.join(d, "s.json")
        st = ExecutionState(episode="a"); st.save(p)
        with open(p) as f: old = f.read()
        class Boom:
            def __repr__(self): raise RuntimeError("crash while writing")
        st.attempts.append({"bad": Boom()})
        with self.assertRaises(Exception): st.save(p)
        with open(p) as f: self.assertEqual(f.read(), old)
        self.assertEqual([f for f in os.listdir(d) if f.startswith(".state_")], [])

class StaleAndResumeTests(unittest.TestCase):
    def test_stale_plan_detection(self):
        w = World2D(9, 5, (1, 1), 0, [Obj("o0", "red box", 6, 1)]); fp0 = observe(w, [(6, 1)])
        self.assertIsNone(stale_reason(fp0, observe(w, [(6, 1)])))
        w.execute("move_forward", {"distance": 1}); self.assertIn("pose", stale_reason(fp0, observe(w, [(6, 1)])))
        self.assertIn("obstacle map", stale_reason(fp0, {**fp0, "obstacles_hash": "x"}))
    def test_interrupted_run_equals_uninterrupted_run(self):
        for seed in range(4):
            for yaw in (0, 90, 180, 270):
                w1, _ = make_task("goto", seed, yaw, 3, False, 1); goal = next((x, y) for x in range(w1.w) for y in range(w1.h) if w1.free(x, y) and (x, y) != (w1.x, w1.y))
                plan = drive_plan(w1, goal); full = run(w1, plan)
                w2, _ = make_task("goto", seed, yaw, 3, False, 1); d = tempfile.mkdtemp(); p = os.path.join(d, "s.json")
                k = max(1, len(plan["steps"]) // 2)
                st = run(w2, plan, p, stop_after=k); st.save(p); del st                         # the agent process dies here
                st2 = ExecutionState.load(p)
                self.assertIsNone(resume_blocker(st2, observe(w2)))
                done = run(w2, plan, p, st=st2)
                self.assertEqual((w1.x, w1.y, w1.yaw, sorted(w1.cleaned), w1.collisions), (w2.x, w2.y, w2.yaw, sorted(w2.cleaned), w2.collisions))
                self.assertEqual(done.completed_steps, full.completed_steps); self.assertEqual(done.terminal, full.terminal)
    def test_unsafe_resume_is_rejected(self):
        w, _ = make_task("goto", 1, 0, 3, False, 1); goal = next((x, y) for x in range(w.w) for y in range(w.h) if w.free(x, y) and (x, y) != (w.x, w.y))
        plan = drive_plan(w, goal); d = tempfile.mkdtemp(); p = os.path.join(d, "s.json")
        st = run(w, plan, p, stop_after=1); st.save(p); st2 = ExecutionState.load(p)
        w.yaw = (w.yaw + 90) % 360                                                              # someone turned the robot meanwhile
        self.assertIn("pose", resume_blocker(st2, observe(w)))
        w.yaw = (w.yaw - 90) % 360; w.objects[0].x, w.objects[0].y = w.objects[0].x, w.objects[0].y
        stopped = observe(w); stopped["stopped"] = True; self.assertIn("stopped", resume_blocker(st2, stopped))
        st2.terminate("safe_stop"); self.assertIn("already ended", resume_blocker(st2, observe(w)))
        self.assertEqual(resume_blocker(ExecutionState(), observe(w)), "no recorded observation to compare with")
    def test_moved_object_blocks_resume(self):
        w = World2D(9, 5, (1, 2), 0, [Obj("o0", "red box", 5, 2)]); st = ExecutionState(); st.start_plan({"steps": []}, "initial", None)
        st.record_attempt(0, {"tool": "get_pose", "args": {}}, "ok", "", 0, observe(w))
        w.objects[0].y = 3
        self.assertIn("view", resume_blocker(st, observe(w)))

if __name__ == "__main__": unittest.main()
