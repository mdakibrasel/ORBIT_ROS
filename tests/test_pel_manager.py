"""ORBIT-PEL M4: guarded execution manager. Invalid or unsafe model output never reaches the robot; retries, duplicates, no-progress,
budgets, stale plans and resume behave as documented in orbit_ros/pel/manager.py. Every test drives the real runner (agent2d.run_episode)
or the manager with real World2D calls; the model is a scripted test double, so nothing here is a real-LLM result."""
import json, os, re, tempfile, unittest
from orbit_ros.world2d import World2D, Obj, Realism
from orbit_ros.tasks2d import Task, Phase, make_task, ALL_FAMILIES
from orbit_ros.agent2d import run_episode, PRESETS, PEL_LADDER, S_LADDER, C_LADDER
from orbit_ros.backends2d import LLMBackend, OracleBackend
from orbit_ros.llm import ScriptedClient
from orbit_ros.pel.manager import ExecutionManager, PelFlags, MAX_RETRIES, NO_PROGRESS_LIMIT
from orbit_ros.pel.registry import RegistryContext
from orbit_ros.pel.state import ExecutionState

def legacy(*acts, **expect):
    d = {"actions": [{"tool": t, "args": a} for t, a in acts]}
    if expect: d["expect"] = expect
    return json.dumps(d)

class Script:
    """Replies to plan/repair calls from a queue (the last reply repeats); assess answers `done`."""
    def __init__(self, plans, done=True): self.plans, self.done, self.tags = list(plans), done, []
    def __call__(self, prompt, tag):
        self.tags.append(tag)
        if tag.startswith("assess"): return json.dumps({"done": self.done(prompt) if callable(self.done) else self.done})
        return self.plans.pop(0) if len(self.plans) > 1 else self.plans[0]

def spy(world):
    """Record every call that reaches the simulator (the adapter boundary)."""
    calls, real = [], world.execute
    def execute(tool, args):
        calls.append((tool, json.loads(json.dumps(args, default=str)))); return real(tool, args)
    world.execute = execute; return calls

def motion_task(text="Move forward 1 meter, then turn right."):
    return Task("unit|motion", "motion", [Phase(text, "motion", {"ops": [("forward", 1), ("turn", "right")]})], {}, {"final_pose": (5, 3, 270)})

def run(world, plans, cfg="PEL", done=True, **kw):
    sc = Script(plans, done); b = LLMBackend(ScriptedClient(sc), False, False)
    ev = []; row = run_episode(world, kw.pop("task", None) or motion_task(), PRESETS[cfg], b, ev, **kw)
    return row, ev, sc

def robot_calls(calls): return [c for c in calls if c[0] not in ("get_pose", "describe_view")]

class NoBypassTests(unittest.TestCase):
    def test_invalid_and_unsafe_calls_never_reach_the_simulator(self):
        w = World2D(9, 7, (4, 3), 0, []); calls = spy(w)
        plans = [legacy(("move_forward", {"distance": 13})),                 # out of range       -> plan rejected, repair
                 legacy(("turn", {"direction": "left", "degrees": 45})),     # not 90|180|270     -> plan rejected, repair
                 legacy(("fly", {"height": 2})),                             # unknown tool       -> plan rejected, repair
                 legacy(("move_forward", {"distance": 5})),                  # leaves the room    -> guard veto, re-plan
                 legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}), x=5, y=3)]
        row, ev, _ = run(w, plans)
        sent = robot_calls(calls)
        self.assertEqual(sent, [("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90})])
        self.assertEqual((row["success"], row["invalid_calls"], row["wall_bumps"], row["object_hits"]), (1, 0, 0, 0))
        self.assertEqual((row["pel_plan_errors"], row["pel_vetoes"], row["pel_terminal"]), (3, 1, "done"))
        self.assertTrue(any(e["event"] == "pel_vetoed" and "leave the room" in e["details"] for e in ev))

    def test_per_call_preconditions_are_enforced(self):
        """`localize` is a valid tool, but this robot has no localiser: rejected before the simulator, then re-planned."""
        w = World2D(9, 7, (4, 3), 0, []); calls = spy(w)
        row, ev, _ = run(w, [legacy(("localize", {})), legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}))])
        self.assertNotIn("localize", [c[0] for c in calls]); self.assertEqual(row["pel_rejections"], 1); self.assertEqual(row["success"], 1)

    def test_believed_obstacles_are_never_entered(self):
        w = World2D(9, 7, (2, 3), 0, [Obj("o1", "red box", 4, 3)]); calls = spy(w)
        row, ev, _ = run(w, [legacy(("move_forward", {"distance": 3})), legacy(("turn", {"direction": "left", "degrees": 90}))], done=True)
        self.assertNotIn(("move_forward", {"distance": 3}), robot_calls(calls))
        self.assertEqual((row["object_hits"], row["pel_vetoes"]), (0, 1))

    def test_declared_no_go_cells_are_enforced(self):
        w = World2D(9, 7, (2, 3), 0, []); calls = spy(w)
        v1 = json.dumps({"schema": "orbit.plan/v1", "steps": [{"id": 1, "tool": "move_forward", "args": {"distance": 3}}],
                         "constraints": {"no_go_cells": [[4, 3]]}})
        row, ev, _ = run(w, [v1, legacy(("turn", {"direction": "left", "degrees": 90}))])
        self.assertNotIn(("move_forward", {"distance": 3}), robot_calls(calls))
        self.assertTrue(any(e["event"] == "pel_vetoed" and "no-go" in e["details"] for e in ev))
        self.assertEqual((w.x, w.y), (2, 3))

    def test_ablations_switch_exactly_one_component_off(self):
        w = World2D(9, 7, (4, 3), 0, []); row, _, _ = run(w, [legacy(("turn", {"direction": "left", "degrees": 45}))], cfg="PEL-registry", done=True)
        self.assertEqual(row["invalid_calls"], 1)                     # without the registry the invalid call reaches the simulator
        w = World2D(9, 7, (4, 3), 0, []); row, _, _ = run(w, [legacy(("move_forward", {"distance": 5}))], cfg="PEL-guard", done=True)
        self.assertGreater(row["wall_bumps"], 0)                      # without the guard the unsafe move reaches it
        w = World2D(9, 7, (4, 3), 0, []); row, ev, sc = run(w, [legacy(("move_forward", {"distance": 5}))], cfg="PEL-replan", done=True)
        self.assertEqual((row["wall_bumps"], row["pel_replans"], row["pel_unrecovered"]), (0, 0, 1))    # vetoed, no re-plan: safe stop
        self.assertEqual(row["pel_terminal"], "safe_stop"); self.assertNotIn("repair", sc.tags)
        self.assertEqual({PRESETS[n].pel_off for n in PEL_LADDER}, {"", "registry", "guard", "verify", "replan"})

class RecoveryTests(unittest.TestCase):
    def test_transient_fault_is_retried_only_when_provably_safe(self):
        w = World2D(9, 7, (4, 3), 0, []); w.set_realism(Realism(fail_moves=(1,), seed="t")); calls = spy(w)
        row, _, _ = run(w, [legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}))])
        self.assertEqual((row["pel_retries"], row["success"]), (1, 1))      # motor fault, pose unchanged -> one retry
        self.assertEqual([c for c in robot_calls(calls) if c[0] == "move_forward"], [("move_forward", {"distance": 1})] * 2)

    def test_retries_are_bounded(self):
        w = World2D(9, 7, (4, 3), 0, []); w.set_realism(Realism(fail_moves=(1, 2, 3, 4, 5), seed="t")); calls = spy(w)
        row, _, _ = run(w, [legacy(("move_forward", {"distance": 1}))], done=False, max_actions=20)
        per_plan = MAX_RETRIES + 1
        self.assertLessEqual(row["pel_retries"], MAX_RETRIES * (row["pel_replans"] + 1))
        self.assertGreaterEqual(row["pel_retries"], MAX_RETRIES); self.assertLessEqual(len([c for c in robot_calls(calls) if c[0] == "move_forward"]), per_plan * (row["pel_replans"] + 1))

    def test_blocked_motion_is_not_retried_and_duplicates_are_blocked(self):
        """Guard off (PEL-guard) so the move reaches the simulator and is blocked. A blocked move is not retry-safe: it is re-planned;
        the identical call from the identical state is then refused as a duplicate instead of being sent again."""
        w = World2D(9, 7, (2, 3), 0, [Obj("o1", "red box", 3, 3)]); calls = spy(w)
        row, ev, _ = run(w, [legacy(("move_forward", {"distance": 1}))], cfg="PEL-guard", done=False)
        self.assertEqual([c for c in robot_calls(calls) if c[0] == "move_forward"], [("move_forward", {"distance": 1})])
        self.assertEqual(row["pel_retries"], 0); self.assertGreaterEqual(row["pel_duplicates_blocked"], 1)

    def test_no_progress_is_detected(self):
        w = World2D(9, 7, (4, 3), 0, [])
        row, ev, _ = run(w, [legacy(*[("get_pose", {})] * 6), legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}))])
        self.assertEqual(row["pel_no_progress"], 1); self.assertEqual(row["success"], 1)
        self.assertEqual(sum(1 for e in ev if e["event"] == "action" and '"get_pose"' in e["details"]), NO_PROGRESS_LIMIT)

    def test_verification_mismatch_triggers_a_replan(self):
        """The plan's completion says x=6; after its steps the robot reports x=5: re-planned, never declared done on the model's word."""
        w = World2D(9, 7, (4, 3), 0, [])
        row, ev, sc = run(w, [legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}), x=6, y=3),
                              legacy(), legacy()], done=True)
        self.assertGreaterEqual(row["pel_verify_failures"], 1); self.assertIn("repair", sc.tags)
        w = World2D(9, 7, (4, 3), 0, [])
        row2, _, _ = run(w, [legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}), x=6, y=3)], cfg="PEL-verify")
        self.assertEqual(row2["pel_verify_failures"], 0)

class BudgetTests(unittest.TestCase):
    def test_tool_call_budget_ends_the_episode(self):
        w = World2D(9, 7, (4, 3), 0, [])
        row, _, _ = run(w, [legacy(*[("turn", {"direction": "left", "degrees": 90})] * 40)], done=False, max_actions=5)
        self.assertEqual(row["pel_terminal"], "budget_exhausted"); self.assertLessEqual(row["env_tool_calls"], row["tool_call_cap"] + 3)

    def test_clarification_and_unsupported_statuses(self):
        t = Task("unit|amb", "goto", [Phase("Go to the box.", "goto", {"label": "box", "clarify_answer": "the red box"})], {}, {"target_id": "o1"})
        w = World2D(9, 7, (4, 3), 0, [Obj("o1", "box", 7, 3)])
        ask = json.dumps({"schema": "orbit.plan/v1", "status": "needs_clarification", "clarification": "Which box?", "steps": []})
        row, ev, _ = run(w, [ask, legacy()], task=t)
        self.assertEqual((row["asks"], row["pel_clarifications"]), (1, 1))
        w = World2D(9, 7, (4, 3), 0, [])
        w = World2D(9, 7, (4, 3), 0, [Obj("o1", "box", 7, 3)])
        uns = json.dumps({"schema": "orbit.plan/v1", "status": "unsupported", "unsupported_reason": "cannot fly", "steps": []})
        row, _, _ = run(w, [uns], task=t)
        self.assertEqual(row["abstained"], 1)

class Crash(Exception): pass

class ResumeTests(unittest.TestCase):
    """Interrupted and uninterrupted runs on the same world; resume only when a fresh observation matches the saved state."""
    PLAN = legacy(("move_forward", {"distance": 2}), ("turn", {"direction": "left", "degrees": 90}), ("move_forward", {"distance": 2}))

    def manager(self, w, sc, d, limit=None):
        b = LLMBackend(ScriptedClient(sc), False, False); n = {"do": 0}
        def observe(ps, fresh):
            p = w.execute("get_pose", {})["data"]; v = w.execute("describe_view", {})["data"]
            return {"pose": p, "raw": v, "interp": [], "obstacles_known": [], "landmarks": {}, "cleaned_known": set(), "start_pose": w.start_pose,
                    "phase_start_pose": ps, "history": [], "use_memory": False, "reactive": False, "grounded": False, "sensor_spec": None,
                    "embodiment_card": None, "fresh_phase": fresh, "reported": [], "clarifications": [], "nav_tools": False, "localizer": "none",
                    "pose_stale": False, "lessons": [], "lesson_effects": set()}
        def do(a, ctx):
            if limit is not None and n["do"] >= limit: raise Crash("process killed")      # simulated crash before the next robot call
            n["do"] += 1; return w.execute(a["tool"], a["args"])
        return ExecutionManager(PelFlags(), b, observe, do, lambda *a, **k: None, lambda: True, lambda: RegistryContext(w.stopped), lambda: w.stopped,
                                w.start_pose, episode="unit|resume", config="PEL", state_dir=d)

    def test_interrupted_equals_uninterrupted(self):
        ph = Phase("drive", "motion", {}); d = tempfile.mkdtemp()
        w0 = World2D(9, 7, (1, 1), 0, []); self.manager(w0, Script([self.PLAN]), None).run_phase(ph, (1, 1, 0), 0)
        for k in (1, 2):
            w = World2D(9, 7, (1, 1), 0, []); m = self.manager(w, Script([self.PLAN]), d, limit=k)
            with self.assertRaises(Crash): m.run_phase(ph, (1, 1, 0), 0)
            path = os.path.join(d, os.listdir(d)[0]); st = ExecutionState.load(path)
            self.assertIsNone(st.terminal); self.assertEqual(st.cursor, k)
            sc = Script([legacy()]); m2 = self.manager(w, sc, d); m2.run_phase(ph, (1, 1, 0), 0, resume=st)
            self.assertEqual((w.x, w.y, w.yaw), (w0.x, w0.y, w0.yaw)); self.assertNotIn("plan", sc.tags); self.assertNotIn("repair", sc.tags)
            self.assertEqual(m2.c["pel_stale_plans"], 0); self.assertEqual(m2.state.terminal["code"], "done")
            for f in os.listdir(d): os.remove(os.path.join(d, f))

    def test_changed_world_blocks_resume(self):
        ph = Phase("drive", "motion", {}); d = tempfile.mkdtemp()
        w = World2D(9, 7, (1, 1), 0, [])
        with self.assertRaises(Crash): self.manager(w, Script([self.PLAN]), d, limit=1).run_phase(ph, (1, 1, 0), 0)
        st = ExecutionState.load(os.path.join(d, os.listdir(d)[0]))
        w.execute("turn", {"direction": "right", "degrees": 90})                 # someone turned the robot while the run was down
        sc = Script([legacy()]); m2 = self.manager(w, sc, d); m2.run_phase(ph, (1, 1, 0), 0, resume=st)
        self.assertEqual(m2.c["pel_stale_plans"], 1); self.assertIn("repair", sc.tags)
        self.assertEqual((w.x, w.y), (3, 1))                                      # the stored step 2 was not executed from the wrong heading

    def test_state_files_are_written_through_the_runner(self):
        d = tempfile.mkdtemp(); w = World2D(9, 7, (4, 3), 0, [])
        run(w, [legacy(("move_forward", {"distance": 1}), ("turn", {"direction": "right", "degrees": 90}))], pel_state_dir=d)
        files = os.listdir(d); self.assertEqual(len(files), 1)
        st = ExecutionState.load(os.path.join(d, files[0])); self.assertEqual(st.terminal["code"], "done")
        self.assertEqual([a["status"] for a in st.attempts], ["ok", "ok"])

class IsolationTests(unittest.TestCase):
    def test_prompts_carry_no_evaluator_data(self):
        """Same leak check as tests/test_v013.py, for the PEL configurations."""
        EVAL_KEYS = ("target_id", "alt_target_id", "condition_true", "interpretations", "final_pose", "forbidden")
        for fam in ALL_FAMILIES:
            for cfg in ("PEL", "PEL-guard"):
                w, t = make_task(fam, 1, 180, 3, False, 1); w.set_realism(Realism(odom_p=1.0, seed=t.task_id)); seen = []
                def fn(p, tag, w=w):
                    seen.append((p, (w.x, w.y), (w.pose()["x"], w.pose()["y"])))
                    return '{"done": false}' if tag.startswith("assess") else '{"actions":[{"tool":"move_forward","args":{"distance":1}}]}'
                run_episode(w, t, PRESETS[cfg], LLMBackend(ScriptedClient(fn), False, False))
                self.assertTrue(seen)
                for p, true_xy, est_xy in seen:
                    self.assertIsNone(re.search(r"\bo\d\b", p)); [self.assertNotIn(k, p) for k in EVAL_KEYS]
                    if true_xy != est_xy: self.assertNotIn(f'"x": {true_xy[0]}, "y": {true_xy[1]}, "yaw_deg"', p)

    def test_pel_configs_are_not_default_and_runner_accepts_them(self):
        import run_spatial
        self.assertFalse(set(PEL_LADDER) & (set(S_LADDER) | set(C_LADDER) | {"S6"}))
        for n in PEL_LADDER:
            w, t = make_task("goto", 0, 90, 3, False, 1)
            row = run_episode(w, t, PRESETS[n], OracleBackend(0.05, 0.3, 0.1, abstain=True)); self.assertIn(row["success"], (0, 1))
            self.assertIn("pel_terminal", row)
        w, t = make_task("goto", 0, 90, 3, False, 1)
        self.assertNotIn("pel_terminal", run_episode(w, t, PRESETS["C4"], OracleBackend(0.05, 0.3, 0.1)))

if __name__ == "__main__": unittest.main()
