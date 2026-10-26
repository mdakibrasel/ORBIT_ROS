import json, tempfile, unittest
from orbit_ros.humanoid import HumanoidSession, FakeNao
from orbit_ros.mcp_server import HumanoidMcpServer


def session(log_dir=False, extra=()):
    s = HumanoidSession(sim="fake", log_dir=log_dir, memory=False, backend=FakeNao(extra_obstacles=extra))
    s.load_scene("living_room")
    return s


class TestDebugView(unittest.TestCase):
    @staticmethod
    def events(s):
        with open(s.log_path, encoding="utf-8") as f: return [json.loads(l) for l in f]

    def test_empty_before_any_task(self):
        v = session().debug_view()
        self.assertIsNone(v["target"]); self.assertEqual(v["route"], [])
        self.assertIn("target: none", v["summary"][1])
        self.assertNotIn("T", v["map"])

    def test_target_route_and_status_after_a_walk(self):
        s = session()
        r = s.robot_call("go_to_room", {"room": "kitchen"})
        v = s.debug_view()
        self.assertEqual(r["status"], "ok"); self.assertEqual(v["task"]["tool"], "go_to_room"); self.assertEqual(v["task"]["status"], "ok")
        self.assertTrue(v["target"]["label"]); self.assertGreaterEqual(len(v["route"]), 2)
        self.assertIn("*", v["map"])
        s.nao.set_pose(1.0, 1.0, 0); self.assertIn("T", s.debug_view()["map"])      # the robot arrow hides T only while it stands on the target
        self.assertEqual(len(v["map"].splitlines()), len(s.believed_map()["map"].splitlines()))
        self.assertIn("legend", v)

    def test_unmapped_obstacle_is_shown_and_logged(self):
        with tempfile.TemporaryDirectory() as d:
            s = session(d)
            s.bumps.append({"x": 3.0, "y": 3.0, "sx": 0.4, "sy": 0.4})
            v = s.debug_view()
            self.assertEqual(v["obstacles"]["bumped"], [{"x": 3.0, "y": 3.0}]); self.assertIn("?", v["map"])
            s.robot_call("turn", {"direction": "left", "degrees": 40})
            ev = self.events(s)
            dbg = [e for e in ev if e.get("event") == "debug"]
            self.assertTrue(dbg); self.assertEqual(dbg[-1]["tool"], "turn"); self.assertEqual(dbg[-1]["obstacles"]["bumped"][0]["x"], 3.0)
            self.assertIn("pose", dbg[-1])

    def test_route_is_logged(self):
        with tempfile.TemporaryDirectory() as d:
            s = session(d); s.robot_call("go_to_room", {"room": "kitchen"})
            ev = self.events(s)
            routes = [e for e in ev if e.get("event") == "route"]
            self.assertTrue(routes and routes[0]["route"]); self.assertTrue(routes[0]["target"])

    def test_debug_view_does_not_move_and_guard_still_vetoes(self):
        s = session(); before = s._pose(); s.debug_view(); self.assertEqual(s._pose(), before)
        r = s.robot_call("walk_forward", {"steps": 40})
        self.assertIn(r["status"], ("vetoed", "rejected", "failed"))

    def test_display_failure_never_stops_the_robot(self):
        s = session()
        class Boom(FakeNao):
            def show(self, lines): raise RuntimeError("no display")
        s._backend = Boom(); s.load_scene("living_room")
        self.assertEqual(s.robot_call("turn", {"direction": "left", "degrees": 40})["status"], "ok")

    def test_mcp_tool(self):
        srv = HumanoidMcpServer(session(), "t")
        self.assertIn("debug_view", [t["name"] for t in srv.tool_defs()])
        reply = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "debug_view", "arguments": {}}})
        out = json.loads(reply["result"]["content"][0]["text"])
        self.assertIn("map", out); self.assertIn("summary", out)

if __name__ == "__main__":
    unittest.main()
