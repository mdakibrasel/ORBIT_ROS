import json, math, unittest
from orbit_ros.humanoid import HumanoidSession, FakeNao, validate_humanoid_call
from orbit_ros.mcp_server import HumanoidMcpServer


def session():
    s = HumanoidSession(sim="fake", log_dir=False, memory=False, backend=FakeNao())
    s.load_scene("living_room")
    return s


class TestPlanning(unittest.TestCase):
    def test_ordering_saves_walking_and_does_not_move(self):
        s = session(); before = s._pose()
        r = s.robot_call("plan_tasks", {"steps": [{"room": "bedroom"}, {"room": "kitchen"}, {"object": "desk"}]})
        d = r["data"]
        self.assertEqual(r["status"], "ok"); self.assertEqual(s._pose(), before)
        self.assertLess(d["walk_m"], d["as_given_m"]); self.assertGreater(d["saved_m"], 0)
        names = [x["do"] for x in d["steps"]]
        self.assertLess(abs(names.index("go to the bedroom") - names.index("go to the desk")), 2)     # the desk is in the bedroom: those two stay together
        for x in d["steps"]: self.assertTrue(x["done_when"] and x["stand_at"])

    def test_given_order_is_kept(self):
        d = session().robot_call("plan_tasks", {"steps": [{"room": "bedroom"}, {"room": "kitchen"}], "order": "given"})["data"]
        self.assertEqual([x["request"] for x in d["steps"]], [1, 2]); self.assertEqual(d["saved_m"], 0)

    def test_repeats_and_redundant_visits_are_merged(self):
        d = session().robot_call("plan_tasks", {"steps": [{"room": "kitchen", "do": "clean"}, {"room": "kitchen"}, {"room": "kitchen", "do": "clean"}]})["data"]
        self.assertEqual(len(d["steps"]), 1); self.assertEqual(len(d["skipped"]), 2)

    def test_many_steps_use_the_heuristic(self):
        steps = [{"object": o} for o in ("sofa", "fridge", "bed", "toilet", "desk", "television", "stove", "bathtub", "armchair", "wardrobe")]
        d = session().robot_call("plan_tasks", {"steps": steps})["data"]
        self.assertEqual(len(d["steps"]), 10); self.assertLessEqual(d["walk_m"], d["as_given_m"])

    def test_argument_checks(self):
        bad = [[], "kitchen", [{"room": "garage"}], [{"room": "kitchen", "object": "sofa"}], [{"do": "visit"}], [{"x": 1}], [{"x": 99, "y": 1}],
               [{"object": "unicorn"}], [{"room": "kitchen", "do": "fly"}], [{"room": "kitchen"}] * 13, [{"room": "kitchen", "speed": 3}]]
        for steps in bad: self.assertTrue(validate_humanoid_call("run_plan", {"steps": steps}, False, False), steps)
        self.assertEqual(validate_humanoid_call("run_plan", {"steps": [{"x": 2, "y": 2, "do": "wave"}], "finish": "start"}, False, False), [])
        self.assertEqual(session().robot_call("run_plan", {"steps": [{"room": "garage"}]})["status"], "rejected")

    def test_a_point_in_a_no_go_zone_is_reported_not_forced(self):
        s = session(); s.set_no_go_zone(2.0, 2.0, 0.5)
        r = s.robot_call("run_plan", {"steps": [{"x": 2.0, "y": 2.0}], "order": "given"})
        self.assertEqual(r["status"], "rejected"); self.assertIn("no safe path", r["message"])
        r = s.robot_call("run_plan", {"steps": [{"x": 2.0, "y": 2.0}, {"room": "kitchen"}]})
        self.assertEqual(r["status"], "failed"); self.assertIn("left out", r["message"]); self.assertEqual(len(r["data"]["steps"]), 1); self.assertTrue(r["data"]["steps"][0]["met"])


class TestRunPlan(unittest.TestCase):
    def test_visit_clean_wave_and_return(self):
        s = session()
        r = s.robot_call("run_plan", {"steps": [{"object": "fridge", "do": "clean"}, {"room": "bedroom"}, {"x": 3.0, "y": 3.0, "do": "wave"}], "finish": "start"})
        self.assertEqual(r["status"], "ok", r["message"])
        steps = r["data"]["steps"]; self.assertEqual(len(steps), 4); self.assertTrue(all(x["met"] for x in steps))
        v = s.robot_call("verify_task")["data"]; self.assertTrue(v["passed"], v)
        self.assertTrue(any("planned step" in c["check"] for c in v["checks"]))
        sx, sy, _ = s.start_pose; p = s._pose(); self.assertLess(math.hypot(p["x"] - sx, p["y"] - sy), 0.6)
        self.assertGreaterEqual(s.metrics()["tasks"], 1)

    def test_sweeping_a_room_in_a_plan_checks_coverage(self):
        s = session()
        r = s.robot_call("run_plan", {"steps": [{"room": "bathroom", "do": "clean"}]})
        self.assertEqual(r["status"], "ok", r["message"]); self.assertIn("covered", r["data"]["steps"][0]["detail"])
        self.assertIn("bathroom", s.clean_status()["swept"])

    def test_debug_view_shows_the_plan_and_guard_stays_active(self):
        s = session(); s.robot_call("run_plan", {"steps": [{"room": "kitchen"}, {"object": "bed"}]})
        v = s.debug_view(); self.assertEqual([x["state"] for x in v["plan"]], ["done", "done"]); self.assertIn("plan: 2/2 done", v["summary"][1])
        s.nao.set_pose(7.75, 1.9, 0)
        self.assertEqual(s.robot_call("walk_forward", {"steps": 1})["status"], "vetoed")

    def test_a_stopped_robot_runs_nothing(self):
        s = session(); s.robot_call("stop_robot")
        self.assertEqual(s.robot_call("run_plan", {"steps": [{"room": "kitchen"}]})["status"], "rejected")

    def test_mcp_tools(self):
        srv = HumanoidMcpServer(session(), "t"); defs = {t["name"]: t for t in srv.tool_defs()}
        self.assertIn("run_plan", defs); self.assertIn("plan_tasks", defs)
        sch = defs["run_plan"]["inputSchema"]["properties"]["steps"]; self.assertEqual(sch["type"], "array"); self.assertEqual(sch["items"]["type"], "object")
        reply = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "plan_tasks", "arguments": {"steps": [{"room": "kitchen"}]}}})
        out = json.loads(reply["result"]["content"][0]["text"]); self.assertEqual(out["status"], "ok")


if __name__ == "__main__":
    unittest.main()
