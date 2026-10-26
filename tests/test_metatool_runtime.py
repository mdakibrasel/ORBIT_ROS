"""Code checks for the meta-tool runtime pieces that need neither a robot nor a network (NO LLM call, NO Webots).
The executor's full plan->execute->verify->repair path is exercised LIVE in M.10 (that is the honest place for it)."""
import json, unittest
from orbit_ros.humanoid import HUMANOID_TOOLS
from orbit_ros.metatool import prompts, guard as g
from orbit_ros.metatool.trace import Trace


class _Pose:
    """A tiny stand-in session exposing only _pose()/stopped/memory for the loop-guard digest (no robot, no motion)."""
    def __init__(self, x=4.8, y=3.1, yaw=180.0, fallen=False, stopped=False):
        self._p = {"x": x, "y": y, "yaw_deg": yaw, "fallen": fallen, "room": "living_room"}
        self.stopped = stopped; self.memory = None
    def _pose(self): return dict(self._p)
    def move(self, dx, dy): self._p["x"] += dx; self._p["y"] += dy


class TestPrompts(unittest.TestCase):
    def test_catalogue_and_system_prompt(self):
        cat = prompts.tool_catalogue(HUMANOID_TOOLS)
        self.assertIn("go_to_room(", cat); self.assertIn("[", cat)   # risk tag present
        sysp = prompts.system_prompt(HUMANOID_TOOLS)
        self.assertIn("ONE complete plan", sysp); self.assertIn("feasible", sysp)

    def test_system_prompt_names_no_phantom_tools(self):
        # the prompt must not instruct the model to use tools that are not in the registry
        sysp = prompts.system_prompt(HUMANOID_TOOLS)
        for phantom in ("remember_place", "find_object"):
            self.assertNotIn(phantom, sysp, f"prompt references non-existent tool {phantom!r}")

    def test_refine_prompt_lists_issues(self):
        class I:  # a plain issue-like object
            code, step, msg, hint = "ARG_RANGE", "s1", "walk_forward.steps must be in 1..6", "use 1..6"
        txt = prompts.refine_prompt([I()])
        self.assertIn("ARG_RANGE", txt); self.assertIn("use 1..6", txt)

    def test_user_prompt_has_memory_and_facts(self):
        u = prompts.user_prompt("go to the kitchen", {"room": "living_room"}, "MEMORY: nothing", {"upright", "not_stopped"})
        self.assertIn("go to the kitchen", u); self.assertIn("MEMORY", u); self.assertIn("upright", u)


class TestLoopGuard(unittest.TestCase):
    def test_blocks_third_identical_no_state_change(self):
        s = _Pose(); lg = g.LoopGuard()
        for _ in range(2):
            ok, _ = lg.check("get_pose", {}, s); self.assertTrue(ok); lg.record("get_pose", {}, s)
        ok, why = lg.check("get_pose", {}, s)
        self.assertFalse(ok); self.assertIn("loop", why)

    def test_state_change_resets_the_block(self):
        s = _Pose(); lg = g.LoopGuard()
        for _ in range(2):
            lg.check("walk_forward", {"steps": 1}, s); lg.record("walk_forward", {"steps": 1}, s); s.move(0.3, 0)
        ok, _ = lg.check("walk_forward", {"steps": 1}, s)   # state changed each time -> not the same fingerprint
        self.assertTrue(ok)

    def test_ababa_oscillation_blocked(self):
        # Distinct args each call so the identical-fingerprint rule does NOT fire; isolate the A-B-A-B-A tool-identity pattern.
        s = _Pose(); lg = g.LoopGuard()
        seq = [("turn", {"degrees": 40}), ("walk_forward", {"steps": 1}), ("turn", {"degrees": 60}),
               ("walk_forward", {"steps": 2})]
        for t, a in seq:
            ok, _ = lg.check(t, a, s); self.assertTrue(ok); lg.record(t, a, s); s.move(0.05, 0.0)
        ok, why = lg.check("turn", {"degrees": 180}, s)   # completes turn-walk-turn-walk-turn
        self.assertFalse(ok); self.assertIn("oscillat", why)

    def test_budgets(self):
        lg = g.LoopGuard(budget=g.Budget(tool_calls=1, llm_calls=1, repairs=1, seconds=1000))
        lg.tool_calls = 2
        self.assertIsNotNone(lg.over_budget(0, 0, 0))


class TestTraceMetrics(unittest.TestCase):
    def test_counts_and_metrics(self):
        tr = Trace()
        tr.event("tool", status="ok"); tr.event("tool", status="rejected"); tr.event("guard", blocked=True)
        tr.event("llm"); tr.event("verify", ok=True)
        m = tr.metrics()
        self.assertEqual(m["tool_calls"], 2); self.assertEqual(m["blocked_calls"], 1)
        self.assertEqual(m["invalid_pct"], 50.0); self.assertEqual(m["verified_steps"], 1)


class TestLlmModeBookkeeping(unittest.TestCase):
    def test_mode_order_and_rejection_memory(self):
        from orbit_ros.metatool import llm_io
        llm_io._REJECTED_MODE.clear()
        c = llm_io.LlmClient.__new__(llm_io.LlmClient)   # no network, no env
        c.provider, c.model = "groq", "m"
        self.assertEqual(c._modes({"x": 1}), ["json_schema", "json_object", "text"])
        llm_io._REJECTED_MODE[("groq", "m")] = {"json_schema"}
        self.assertEqual(c._modes({"x": 1}), ["json_object", "text"])
        self.assertEqual(c._modes(None), ["json_object", "text"])   # no schema -> skip json_schema


if __name__ == "__main__":
    unittest.main()
