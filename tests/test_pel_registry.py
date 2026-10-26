"""ORBIT-PEL M2: tool registry. The registry must accept and reject exactly what the simulator and the runtime accept and reject."""
import itertools, re, unittest
from orbit_ros.world2d import World2D, ENV_TOOLS
from orbit_ros.cognition import AGENT_TOOLS
from orbit_ros.navtools import MACROS, expand
from orbit_ros import backends2d, navtools
from orbit_ros.pel.registry import REGISTRY, ROBOT_TOOLS, RegistryContext, validate_call, admissible, prompt_text, validate_plan_calls
from orbit_ros.pel.schema import parse_plan

VALUES = [None, True, False, 0, 1, 3, 12, 13, -1, 90, 180, 270, 45, 90.0, 2.5, "3", "left", "right", "up", "", [], [1, 2], [[1, 2]], {}, {"a": 1}]

def world_accepts(tool, args):
    """True if World2D.execute does NOT count the call as invalid (a blocked move or a motor fault is a valid call that failed)."""
    w = World2D(30, 30, (15, 15), 0, []); w.localizer = True
    n = w.invalid_calls; w.execute(tool, dict(args)); return w.invalid_calls == n

def arg_sets(tool):
    names = list(REGISTRY[tool].args) or ["x"]
    for combo in itertools.product(VALUES + ["__absent__"], repeat=len(names)):
        yield {k: v for k, v in zip(names, combo) if v != "__absent__"}

class AgreementTests(unittest.TestCase):
    def test_robot_tools_match_world2d_exactly(self):
        n = 0
        for tool in sorted(ENV_TOOLS):
            for args in arg_sets(tool):
                n += 1
                self.assertEqual(admissible({"tool": tool, "args": args}), world_accepts(tool, args), (tool, args))
            self.assertEqual(admissible({"tool": tool, "args": {"extra_key": 1, **({"distance": 1} if tool == "move_forward" else {})}}),
                             world_accepts(tool, {"extra_key": 1, **({"distance": 1} if tool == "move_forward" else {})}))
        self.assertGreater(n, 800)            # 858 generated argument sets over the 8 robot tools
    def test_documented_quirks_are_mirrored(self):
        self.assertTrue(admissible({"tool": "turn", "args": {"direction": "left", "degrees": 90.0}}))
        self.assertTrue(world_accepts("turn", {"direction": "left", "degrees": 90.0}))
        iss = validate_call({"tool": "clean", "args": {"speed": 3}})
        self.assertEqual([(i.code, i.severity) for i in iss], [("unknown_field", "warning")])
    def test_macros_match_navtools_argument_checks(self):
        pose = {"x": 2, "y": 2, "yaw_deg": 0, "room_w": 30, "room_h": 30}
        arg_msgs = ("needs integer", "needs side", "avoid must be")
        for tool in sorted(MACROS):
            for args in arg_sets(tool):
                _, err = expand(tool, args, pose, (2, 2, 0), [])
                nav_rejects_args = bool(err) and any(m in err for m in arg_msgs)
                self.assertEqual(not admissible({"tool": tool, "args": args}), nav_rejects_args, (tool, args, err))
    def test_registry_covers_every_runtime_tool(self):
        self.assertTrue(ENV_TOOLS <= ROBOT_TOOLS); self.assertTrue((AGENT_TOOLS | MACROS | {"done", "replan"}) <= set(REGISTRY))
        self.assertEqual(ROBOT_TOOLS, ENV_TOOLS)

class RejectionTests(unittest.TestCase):
    def test_unknown_and_malformed(self):
        self.assertEqual(validate_call({"tool": "fly", "args": {}})[0].code, "unknown_tool")
        for bad in (None, "turn", {"args": {}}, {"tool": 5}):
            self.assertEqual(validate_call(bad)[0].code, "malformed_action")
        self.assertEqual(validate_call({"tool": "turn", "args": [1]})[0].code, "bad_type")
    def test_ranges_types_and_missing(self):
        codes = lambda c: [i.code for i in validate_call(c) if i.severity == "error"]
        self.assertEqual(codes({"tool": "move_forward", "args": {"distance": 13}}), ["bad_value"])
        self.assertEqual(codes({"tool": "move_forward", "args": {"distance": True}}), ["bad_value"])
        self.assertEqual(codes({"tool": "move_forward", "args": {}}), ["missing_field"])
        self.assertEqual(codes({"tool": "turn", "args": {"direction": "up", "degrees": 45}}), ["bad_value", "bad_value"])
        self.assertEqual(codes({"tool": "report", "args": {"answer": None}}), ["bad_value"])
    def test_preconditions(self):
        c = {"tool": "move_forward", "args": {"distance": 1}}
        self.assertEqual([i.code for i in validate_call(c, RegistryContext(stopped=True))], ["precondition"])
        self.assertTrue(admissible({"tool": "stop_robot", "args": {}}, RegistryContext(stopped=True)))
        self.assertFalse(admissible({"tool": "localize", "args": {}}, RegistryContext()))
        self.assertFalse(admissible({"tool": "remember", "args": {"label": "x"}}, RegistryContext()))
        self.assertFalse(admissible({"tool": "ask", "args": {"question": "?"}}, RegistryContext(asks_left=0)))
        self.assertFalse(admissible({"tool": "goto_cell", "args": {"x": 1, "y": 1}}, RegistryContext()))
        self.assertTrue(admissible({"tool": "goto_cell", "args": {"x": 1, "y": 1}}, RegistryContext(nav_tools=True)))
    def test_world_preconditions_match(self):
        w = World2D(5, 5, (2, 2), 0, []); w.execute("stop_robot", {})
        for t in ("move_forward", "turn", "get_pose", "describe_view", "clean"):
            n = w.invalid_calls; w.execute(t, {"distance": 1} if t == "move_forward" else {"direction": "left", "degrees": 90})
            self.assertEqual(w.invalid_calls, n + 1); self.assertIn("not_stopped", REGISTRY[t].pre)
    def test_retry_safety_and_categories(self):
        self.assertFalse(REGISTRY["move_forward"].retry_safe); self.assertFalse(REGISTRY["turn"].retry_safe)
        self.assertTrue(REGISTRY["get_pose"].retry_safe); self.assertTrue(REGISTRY["describe_view"].retry_safe)
        for s in REGISTRY.values():
            self.assertIn(s.category, ("motion", "sensing", "effector", "answer", "safety", "agent", "macro", "control"))
            self.assertGreater(s.timeout_s, 0)

class PromptTests(unittest.TestCase):
    def test_existing_prompt_agrees_with_registry(self):
        """The prompt text the existing configs use states the same tools and ranges as the registry (prompts themselves unchanged)."""
        text = backends2d.TOOLS + backends2d.LOCALIZE_TOOL + backends2d.INTERACTIVE_TOOLS + navtools.TOOL_TEXT
        for name in re.findall(r'"tool":"(\w+)"', text): self.assertIn(name, REGISTRY)
        self.assertIn('"distance":int 0..12', text); self.assertEqual((REGISTRY["move_forward"].args["distance"].min, REGISTRY["move_forward"].args["distance"].max), (0, 12))
        self.assertIn('"degrees":90|180|270', text); self.assertEqual(REGISTRY["turn"].args["degrees"].values, (90, 180, 270))
        self.assertIn('"direction":"left|right"', text); self.assertEqual(REGISTRY["turn"].args["direction"].values, ("left", "right"))
        self.assertIn("left|right|front|back|all", text); self.assertEqual(REGISTRY["cover_region"].args["side"].values, ("left", "right", "front", "back", "all"))
    def test_generated_prompt(self):
        t = prompt_text(["move_forward", "turn", "clean"])
        self.assertTrue(t.startswith("Tools (JSON): ")); self.assertIn("int 0..12", t); self.assertEqual(t.count(" | "), 2)

class PlanCallTests(unittest.TestCase):
    def test_plan_steps_are_checked(self):
        plan, _ = parse_plan({"actions": [{"tool": "move_forward", "args": {"distance": 2}}, {"tool": "turn", "args": {"direction": "up", "degrees": 90}}]})
        iss = validate_plan_calls(plan)
        self.assertEqual([(i.code, i.where) for i in iss], [("bad_value", "steps[1].args.direction")])

if __name__ == "__main__": unittest.main()
