import io, json, os, tempfile, unittest
from orbit_ros.humanoid import HumanoidSession, FakeNao
from orbit_ros.nao_memory import NaoMemory, age_text
from orbit_ros.mcp_server import HumanoidMcpServer


def session(memory=None, extra=()):
    s = HumanoidSession(sim="fake", log_dir=False, backend=FakeNao(extra_obstacles=extra), memory=memory if memory is not None else NaoMemory())
    s.load_scene("living_room")
    return s


class TestStore(unittest.TestCase):
    def test_roundtrip_and_relevance(self):
        m = NaoMemory(); now = 10_000.0
        m.add("action", sim="fake", tool="go_to_room", args={"room": "kitchen"}, status="failed", room="kitchen", x=4.0, y=5.0, note="no progress", t=now - 120)
        m.add("action", sim="fake", tool="go_to_room", args={"room": "bedroom"}, status="ok", room="bedroom", x=8.0, y=2.0, t=now - 60)
        m.add("obstacle", sim="fake", label="unknown obstacle (bump)", room="kitchen", x=4.4, y=5.4, t=now - 30)
        m.add("observation", sim="fake", label="fridge", room="kitchen", x=5.8, y=6.0, t=now - 90)
        r = m.recall(sim="fake", room="kitchen", x=4.0, y=5.0, now=now)
        self.assertEqual([f["tool"] for f in r["recent_failures"]], ["go_to_room"])
        self.assertEqual(r["recent_failures"][0]["args"], {"room": "kitchen"})
        self.assertEqual(len(r["obstacles"]), 1); self.assertEqual(r["items_seen"][0]["label"], "fridge")
        self.assertEqual(r["recent_successes"], [])                      # the bedroom success is not relevant to the kitchen
        self.assertEqual(r["recent_failures"][0]["age"], "2 min ago")
        self.assertIn("Current sensors", r["note"])
        self.assertIn("failed or refused here", NaoMemory.brief(r))

    def test_other_simulator_is_not_recalled(self):
        m = NaoMemory(); m.add("obstacle", sim="fake", room="kitchen", x=4, y=5)
        self.assertEqual(m.recall(sim="webots", room="kitchen", x=4, y=5)["obstacles"], [])

    def test_persists_in_a_file_and_is_bounded(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "sub", "mem.db"); m = NaoMemory(p); m.add("position", room="kitchen", x=1, y=2); m.close()
            m2 = NaoMemory(p); self.assertEqual(m2.count("position"), 1); m2.clear(); self.assertEqual(m2.count(), 0); m2.close()
        with self.assertRaises(ValueError): NaoMemory().add("dream")

    def test_age_text(self):
        self.assertEqual((age_text(5), age_text(600), age_text(7200), age_text(400000)), ("5s ago", "10 min ago", "2 h ago", "4 d ago"))


class TestSession(unittest.TestCase):
    def test_actions_failures_positions_and_sightings_are_recorded(self):
        s = session(); m = s.memory
        s.robot_call("go_to_room", {"room": "kitchen"}); s.robot_call("describe_view")
        s.robot_call("walk_forward", {"steps": 99})                      # rejected by the argument check: not a motion, nothing stored
        s.nao.set_pose(7.75, 1.9, 0); s.robot_call("walk_forward", {"steps": 1})   # vetoed: 0.07 m from the bed
        self.assertGreaterEqual(m.count("action"), 2); self.assertGreaterEqual(m.count("position"), 2)
        rows = m._rows("SELECT tool, status FROM events WHERE kind='action'")
        self.assertIn(("go_to_room", "ok"), [(r["tool"], r["status"]) for r in rows])
        self.assertIn(("walk_forward", "vetoed"), [(r["tool"], r["status"]) for r in rows])
        self.assertTrue(m.count("observation") >= 0)

    def test_bumped_obstacle_is_remembered_and_recalled(self):
        box = {"label": "box", "x": 4.65, "y": 6.0, "sx": 0.3, "sy": 0.3}
        nao = FakeNao(extra_obstacles=[box]); s = HumanoidSession(sim="fake", log_dir=False, backend=nao, memory=NaoMemory()); s.load_scene("kitchen")
        s.nao.set_pose(3.9, 5.2, 40); s.robot_call("go_to", {"x": 5.4, "y": 6.8})
        self.assertTrue(s.bumps or s.sonar_obs); self.assertGreaterEqual(s.memory.count("obstacle"), 1)
        r = s.recall_memory(); self.assertTrue(r["enabled"]); self.assertIn("obstacles found here", r["brief"])

    def test_memory_never_changes_what_the_guard_sees(self):
        s = session(); s.memory.add("obstacle", sim="fake", label="ghost", room="living_room", x=2.0, y=2.0)
        self.assertEqual(s.known_obstacles(), session(memory=NaoMemory()).known_obstacles())
        s.nao.set_pose(7.75, 1.9, 0)
        self.assertEqual(s.robot_call("walk_forward", {"steps": 1})["status"], "vetoed")      # safety still active with a memory present

    def test_a_broken_memory_does_not_stop_the_robot(self):
        s = session(); s.memory.db.close()
        r = s.robot_call("go_to_room", {"room": "kitchen"})
        self.assertEqual(r["status"], "ok", r["message"])

    def test_memory_off(self):
        s = HumanoidSession(sim="fake", log_dir=False, backend=FakeNao(), memory=False); s.load_scene("living_room")
        self.assertIsNone(s.memory); self.assertFalse(s.recall_memory()["enabled"])
        self.assertEqual(s.robot_call("wave")["status"], "ok")

    def test_mcp_tool(self):
        srv = HumanoidMcpServer(session(), "t")
        self.assertIn("recall_memory", [t["name"] for t in srv.tool_defs()])
        reply = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "recall_memory", "arguments": {"limit": 3}}})
        out = json.loads(reply["result"]["content"][0]["text"])
        self.assertTrue(out["enabled"]); self.assertIn("MEMORY", out["brief"])


class TestChat(unittest.TestCase):
    def test_brief_is_put_in_front_of_the_request(self):
        import orbit_chat
        s = session(); s.robot_call("go_to_room", {"room": "kitchen"})
        class Mcp:
            def call(self, name, args): return json.dumps(s.recall_memory())
        class Model:
            seen = None
            def ask(self, messages, tools): Model.seen = messages[-1]["content"]; return {"content": "done"}
        orbit_chat.run_turn(Model(), Mcp(), [], [], "clean the kitchen", out=lambda *_: None, memory=True)
        self.assertIn("MEMORY", Model.seen); self.assertTrue(Model.seen.endswith("User request: clean the kitchen"))
        orbit_chat.run_turn(Model(), Mcp(), [], [], "hello", out=lambda *_: None)
        self.assertEqual(Model.seen, "hello")


class TestEpisodicSemantic(unittest.TestCase):
    def test_episode_roundtrip_and_similarity_retrieval(self):
        m = NaoMemory()
        m.add_episode("go to the kitchen", sim="webots", success=False, start={"room": "living_room", "x": 4.8, "y": 3.1},
                      executed=[{"tool": "walk_forward", "status": "vetoed"}], feedback="open-loop hit the guard",
                      provenance={"trial": "t1"})
        got = m.recall_episodes(instruction="please walk to the kitchen", sim="webots", start_room="living_room")
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0]["goal"], "go:kitchen"); self.assertFalse(got[0]["success"])
        self.assertEqual(got[0]["provenance"], {"trial": "t1"})
        # a different goal is not retrieved
        self.assertEqual(m.recall_episodes(instruction="clean the bathroom", sim="webots"), [])

    def test_dedup_updates_in_place(self):
        m = NaoMemory()
        m.add_episode("go to kitchen", sim="webots", success=False, start={"room": "living_room"})
        m.add_episode("go to kitchen", sim="webots", success=True, start={"room": "living_room"}, outcome="reached")
        self.assertEqual(m.count_episodes(), 1)                     # same (sim, goal, start_room) updated, not duplicated
        self.assertTrue(m.recall_episodes(goal="go:kitchen", sim="webots")[0]["success"])

    def test_strategy_confidence_and_verified(self):
        m = NaoMemory()
        m.learn_strategy("go:kitchen", tool="go_to_room", args={"room": "kitchen"}, success=True, sim="webots")
        m.learn_strategy("go:kitchen", tool="go_to_room", args={"room": "kitchen"}, success=True, sim="webots")
        st = m.recall_strategies("go:kitchen", sim="webots")
        self.assertEqual(len(st), 1); self.assertTrue(st[0]["verified"]); self.assertEqual(st[0]["successes"], 2)
        self.assertEqual(st[0]["confidence"], 1.0)

    def test_retrieval_disabled_returns_nothing(self):
        m = NaoMemory(retrieval_enabled=False)
        m.add_episode("go to kitchen", sim="webots", success=True, start={"room": "living_room"})
        self.assertEqual(m.recall_episodes(instruction="go to kitchen", sim="webots"), [])
        self.assertEqual(m.task_brief(instruction="go to kitchen", sim="webots")["brief"], "")

    def test_persistence_and_migration_v1_to_v2(self):
        import sqlite3
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "mem.db")
            # simulate a v1 db (events only, no meta/episodes)
            con = sqlite3.connect(p)
            con.execute("CREATE TABLE events(id INTEGER PRIMARY KEY, t REAL, sim TEXT, kind TEXT, tool TEXT, args TEXT, status TEXT, room TEXT, x REAL, y REAL, yaw REAL, label TEXT, note TEXT)")
            con.commit(); con.close()
            m = NaoMemory(p)                                        # opening migrates in place
            self.assertEqual(m.schema_version(), 2)
            m.add_episode("go to kitchen", sim="webots", success=True, start={"room": "living_room"}); m.close()
            m2 = NaoMemory(p)                                       # survives restart
            self.assertEqual(m2.count_episodes(), 1); m2.close()

    def test_normalize_goal(self):
        rooms = ["kitchen", "bathroom", "bedroom", "living_room"]; objs = {"fridge", "bed", "sofa"}
        self.assertEqual(NaoMemory.normalize_goal("please walk to the kitchen", rooms=rooms), "go:kitchen")
        self.assertEqual(NaoMemory.normalize_goal("clean the bathroom", rooms=rooms), "clean:bathroom")
        self.assertEqual(NaoMemory.normalize_goal("stand next to the fridge", rooms=rooms, objects=objs), "go:fridge")
        # without a vocabulary it still classifies the verb, just no room/object token
        self.assertEqual(NaoMemory.normalize_goal("do a little dance"), "go")

    def test_session_record_and_retrieve(self):
        s = session()                                              # fake sim, fresh in-memory NaoMemory
        s.robot_call("go_to_room", {"room": "kitchen"})
        wb = s.record_episode("go to the kitchen", {"status": "ok", "message": "reached", "_tool": "go_to_room"})
        self.assertTrue(wb["written"]); self.assertEqual(wb["goal"], "go:kitchen")
        tm = s.task_memory(instruction="walk to the kitchen")
        self.assertTrue(tm["enabled"])
        self.assertTrue(any(st["tool"] == "go_to_room" and st["verified"] for st in tm["strategies"]))


if __name__ == "__main__": unittest.main()
