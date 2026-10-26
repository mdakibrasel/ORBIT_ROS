"""ORBIT-PEL M1: plan schema orbit.plan/v1 (parsing, checking, legacy round trip). No robot behaviour is involved."""
import copy, json, random, unittest
from orbit_ros.agent2d import normalize_plan
from orbit_ros.pel.schema import (SCHEMA, PlanV1, Step, parse_plan, parse_plan_json, check_plan, to_legacy, from_legacy, errors,
                                  default_known_tools)
from tools.pel_roundtrip_check import check as roundtrip_check

def v1(**kw):
    d = {"schema": SCHEMA, "plan_id": "t", "revision": 0, "goal": "go to the red box", "status": "ok",
         "steps": [{"id": 1, "tool": "turn", "args": {"direction": "left", "degrees": 90}, "expect": {"pose": {"yaw_deg": 180}}},
                   {"id": 2, "tool": "move_forward", "args": {"distance": 2}, "pre": {"robot_stopped": False},
                    "expect": {"pose": {"x": 2, "y": 3}}, "on_failure": "replan"}],
         "completion": {"pose": {"x": 2, "y": 3}}, "constraints": {"no_go_cells": [[1, 4], [2, 3]], "max_steps": 20}}
    d.update(kw); return d

def codes(issues, severity="error"): return sorted({i.code for i in issues if i.severity == severity})

class ValidPlanTests(unittest.TestCase):
    def test_full_v1_plan_is_clean(self):
        plan, issues = parse_plan(v1())
        self.assertEqual(errors(issues), []); self.assertEqual(plan.source, SCHEMA)
        self.assertEqual([s.id for s in plan.steps], [1, 2]); self.assertEqual(plan.steps[1].expect, {"pose": {"x": 2, "y": 3}})
        self.assertEqual(plan.constraints["no_go_cells"], [[1, 4], [2, 3]])
    def test_v1_json_round_trip(self):
        plan, _ = parse_plan(v1(revision=2, parent_revision=1, revision_reason="step 2 blocked"))
        again, issues = parse_plan(json.loads(json.dumps(plan.to_dict())))
        self.assertEqual(errors(issues), []); self.assertEqual(again.to_dict(), plan.to_dict())
    def test_v1_to_legacy_is_executable_by_the_old_runner(self):
        plan, _ = parse_plan(v1())
        self.assertEqual(to_legacy(plan), {"actions": [{"tool": "turn", "args": {"direction": "left", "degrees": 90}},
                                                       {"tool": "move_forward", "args": {"distance": 2}}], "expect": {"x": 2, "y": 3}})
        self.assertEqual(normalize_plan(to_legacy(plan)), to_legacy(plan))

class MalformedTests(unittest.TestCase):
    def test_never_raises_on_odd_inputs(self):
        for x in (None, 3, "move", [], [1, 2], {}, {"actions": "x"}, {"actions": [None, 5, "turn"]}, {"actions": [{"tool": 5}]},
                  {"actions": [{"tool": "turn", "args": [1]}]}, {"expect": 5}, {"schema": SCHEMA}, {"schema": SCHEMA, "steps": "x"},
                  {"schema": SCHEMA, "steps": [None, {"id": 1}]}, {"regions": "x"}, {"regions": [5, {"name": "a"}]}):
            plan, issues = parse_plan(copy.deepcopy(x))
            self.assertIsInstance(plan, PlanV1)
            self.assertEqual(to_legacy(plan), normalize_plan(copy.deepcopy(x)) if "schema" not in (x if isinstance(x, dict) else {}) else to_legacy(plan))
    def test_non_object_and_bad_json(self):
        self.assertIn("schema_error", codes(parse_plan("go left")[1]))
        self.assertIn("schema_error", codes(parse_plan_json("{not json")[1]))
        self.assertEqual(parse_plan_json('{"actions": []}')[0].steps, [])
    def test_malformed_legacy_actions_are_flagged_and_kept(self):
        plan, issues = parse_plan({"actions": [{"tool": "turn"}, "oops", {"args": {}}]})
        self.assertEqual([s.tool for s in plan.steps], ["turn", "__malformed__", "__malformed__"])
        self.assertEqual(codes(issues), ["malformed_action"])
    def test_wrong_schema_version(self):
        plan, issues = parse_plan({"schema": "orbit.plan/v2", "steps": []})
        self.assertEqual(codes(issues), ["schema_error"]); self.assertEqual(plan.steps, [])

class MissingAndDuplicateTests(unittest.TestCase):
    def test_missing_fields(self):
        d = v1(); del d["steps"]
        self.assertIn("missing_field", codes(parse_plan(d)[1]))
        d = v1(); del d["steps"][0]["tool"]
        self.assertIn("missing_field", codes(parse_plan(d)[1]))
        d = v1(); del d["steps"][1]["id"]
        plan, issues = parse_plan(d); self.assertEqual(codes(issues), ["missing_field"])
    def test_duplicate_and_bad_ids(self):
        d = v1(); d["steps"][1]["id"] = 1
        self.assertEqual(codes(parse_plan(d)[1]), ["duplicate_id"])
        for bad in (0, -1, "2", True, 1.5):
            d = v1(); d["steps"][1]["id"] = bad
            self.assertEqual(codes(parse_plan(d)[1]), ["bad_type"], bad)
    def test_bad_types_in_nested_fields(self):
        for path, val in ((("steps", 0, "expect"), {"pose": {"x": "2"}}), (("steps", 0, "pre"), {"robot_stopped": "no"}),
                          (("constraints",), {"no_go_cells": [[1, 2, 3]]}), (("completion",), {"cleaned_min": True}),
                          (("steps", 0, "args"), [1]), (("revision",), -1), (("goal",), 5)):
            d = v1(); tgt = d
            for k in path[:-1]: tgt = tgt[k]
            tgt[path[-1]] = val
            self.assertIn("bad_type", codes(parse_plan(d)[1]), path)
    def test_bad_values(self):
        self.assertIn("bad_value", codes(parse_plan(v1(status="maybe"))[1]))
        d = v1(); d["steps"][0]["on_failure"] = "panic"; self.assertIn("bad_value", codes(parse_plan(d)[1]))
        d = v1(); d["steps"][0]["expect"] = {"pose": {"yaw_deg": 45}}; self.assertIn("bad_value", codes(parse_plan(d)[1]))
        self.assertIn("bad_value", codes(parse_plan(v1(revision=1, parent_revision=1))[1]))
    def test_unknown_fields_are_warnings(self):
        d = v1(colour="red"); d["steps"][0]["speed"] = 3
        plan, issues = parse_plan(d)
        self.assertEqual(errors(issues), []); self.assertEqual(codes(issues, "warning"), ["unknown_field"])

class ToolNameTests(unittest.TestCase):
    def test_unsupported_tools(self):
        d = v1(); d["steps"][0]["tool"] = "fly_to"
        self.assertEqual(codes(parse_plan(d)[1]), ["unknown_tool"])
        self.assertEqual(codes(parse_plan({"actions": [{"tool": "teleport", "args": {}}]})[1]), ["unknown_tool"])
        self.assertEqual(errors(parse_plan(d, known_tools=())[1]), [])            # name check can be switched off
    def test_known_tools_cover_the_runtime(self):
        k = default_known_tools()
        for t in ("move_forward", "turn", "clean", "describe_view", "get_pose", "stop_robot", "report", "localize", "remember", "ask",
                  "abstain", "goto_cell", "cover_region", "done"):
            self.assertIn(t, k)

class StatusTests(unittest.TestCase):
    def test_clarification(self):
        plan, issues = parse_plan(v1(status="needs_clarification", clarification="Which red box, the left or the right one?", steps=[]))
        self.assertEqual(errors(issues), []); self.assertEqual(plan.status, "needs_clarification")
        self.assertIn("missing_field", codes(parse_plan(v1(status="needs_clarification", steps=[]))[1]))
        self.assertIn("bad_value", codes(parse_plan(v1(status="needs_clarification", clarification="which?"))[1]))
    def test_unsupported(self):
        self.assertEqual(errors(parse_plan(v1(status="unsupported", unsupported_reason="the robot cannot climb stairs", steps=[]))[1]), [])
        self.assertIn("missing_field", codes(parse_plan(v1(status="unsupported", steps=[]))[1]))
    def test_empty_ok_plan_is_only_a_warning(self):
        plan, issues = parse_plan(v1(steps=[]))
        self.assertEqual(errors(issues), []); self.assertEqual(codes(issues, "warning"), ["empty_plan"])

class LegacyTests(unittest.TestCase):
    def test_legacy_plan_and_expect(self):
        x = {"actions": [{"tool": "move_forward", "args": {"distance": 2}, "purpose": "approach"}], "expect": {"x": 3, "y": 2, "cleaned_min": 0}}
        plan, issues = parse_plan(copy.deepcopy(x))
        self.assertEqual(errors(issues), []); self.assertEqual(plan.source, "legacy")
        self.assertEqual(plan.completion, {"pose": {"x": 3, "y": 2}, "cleaned_min": 0})
        self.assertEqual(to_legacy(plan), normalize_plan(copy.deepcopy(x)))
    def test_list_and_null_args(self):
        x = [{"tool": "clean", "args": None}, {"tool": "turn", "args": {"direction": "right", "degrees": 90}}]
        self.assertEqual(to_legacy(parse_plan(copy.deepcopy(x))[0]), normalize_plan(copy.deepcopy(x)))
    def test_hierarchical_regions(self):
        x = {"regions": [{"name": "left half", "actions": [{"tool": "clean"}, {"tool": "move_forward", "args": {"distance": 1}}]},
                         {"name": "right half", "steps": [{"tool": "clean", "args": {}}]}], "expect": {"x": 1, "y": 1}}
        plan, issues = parse_plan(copy.deepcopy(x))
        self.assertEqual(plan.source, "legacy_hierarchical"); self.assertEqual(errors(issues), [])
        self.assertEqual([s.region for s in plan.steps], ["left half", "left half", "right half"])
        self.assertEqual(to_legacy(plan), normalize_plan(copy.deepcopy(x)))
        native = from_legacy(copy.deepcopy(x), plan_id="h", goal="clean both halves")
        self.assertEqual(errors(check_plan(native)), []); self.assertEqual(native.to_dict()["steps"][2]["region"], "right half")
    def test_random_legacy_inputs_round_trip(self):
        rng = random.Random(7); tools = ["move_forward", "turn", "clean", "ask", "fly", None, 3]
        def val():
            return rng.choice([None, 1, -2, True, "x", [], {}, {"distance": rng.randint(-2, 15)}, [1, 2]])
        for _ in range(500):
            acts = []
            for _ in range(rng.randint(0, 5)):
                r = rng.random()
                if r < 0.15: acts.append(val())
                else:
                    a = {"tool": rng.choice(tools)}
                    if rng.random() < 0.8: a["args"] = val()
                    if rng.random() < 0.2: a["_region"] = "r"
                    acts.append(a)
            x = rng.choice([acts, {"actions": acts}, {"actions": acts, "expect": val()}, {"regions": [{"name": "r", "actions": acts}]}, val()])
            plan, _ = parse_plan(copy.deepcopy(x))
            self.assertEqual(to_legacy(plan), normalize_plan(copy.deepcopy(x)), x)

class BenchmarkRoundTripTests(unittest.TestCase):
    def test_every_oracle_plan_round_trips(self):
        """SYNTHETIC plans: every plan the oracle produces in three task families under nine plan-writing configs."""
        n, bad, with_errors, ex = roundtrip_check(("goto", "clean_left", "sequence"), 1)
        self.assertGreater(n, 100); self.assertEqual(bad, 0, ex); self.assertEqual(with_errors, 0)

class NoBehaviourChangeTests(unittest.TestCase):
    def test_existing_modules_do_not_import_pel(self):
        """M1-M3: no existing module referred to pel. From M4 on, agent2d is the only one, and it imports pel lazily inside the branch
        of the PEL configurations, so no existing configuration loads or runs PEL code."""
        import os, re
        from orbit_ros.agent2d import PRESETS, S_LADDER, C_LADDER
        root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "orbit_ros")
        new_v015 = ("mcp_bridge.py", "mcp_server.py",       # v0.15 MCP server: built ON the PEL registry and guard; no benchmark module imports it
                    "humanoid.py", "house.py")             # v0.16 NAO humanoid in the house (MCP only), same rule
        for f in os.listdir(root):
            if f.endswith(".py") and f != "agent2d.py" and f not in new_v015:
                with open(os.path.join(root, f), encoding="utf-8") as fh:
                    src_f = fh.read()
                    self.assertIsNone(re.search(r"\bpel\b", src_f), f); self.assertIsNone(re.search(r"\bmcp_(bridge|server)\b|from \.(humanoid|house)\b|from \. import (humanoid|house)\b", src_f), f)
        with open(os.path.join(root, "agent2d.py"), encoding="utf-8") as fh: src = fh.read()
        self.assertIsNone(re.search(r"^(from|import)\s+\S*pel", src, re.M))                       # no module-level import
        self.assertEqual(len(re.findall(r"from \.pel\.", src)), 2)                                 # the two lazy imports in the PEL branch
        for n in list(S_LADDER) + ["S6"] + list(C_LADDER): self.assertNotEqual(PRESETS[n].exec_mode, "pel")

if __name__ == "__main__": unittest.main()
