"""v0.16 NAO humanoid in the furnished house (orbit_ros/house.py, orbit_ros/humanoid.py, HumanoidMcpServer, orbit_mcp_server.py
--robot nao). Everything here runs without Webots, with the kinematic FakeNao."""
import json, math, os, subprocess, sys, tempfile, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros import house
from orbit_ros.humanoid import (HUMANOID_TOOLS, REACH, FakeNao, HumanoidGuard, HumanoidSession, WebotsNao, validate_humanoid_call)
from orbit_ros.mcp_server import HumanoidMcpServer

SERVER = os.path.join(ROOT, "orbit_mcp_server.py")

def session(scene="living_room", **k):
    k.setdefault("log_dir", False); s = HumanoidSession(**k); s.load_scene(scene); return s

class SpyNao(FakeNao):
    """Every walking motion that reaches the robot must pass the guard at that moment, from the pose the session reports."""
    def __init__(self, test, *a, **k):
        super().__init__(*a, **k); self.test, self.session = test, None
    def play(self, motion):
        s = self.session
        if s is not None and motion in REACH:
            self.test.assertIsNone(s.guard.check(motion, (self.x, self.y, self.yaw), s.room, s.known_obstacles(), s.zones),
                                   f"guard-vetoed {motion} reached the robot at {(self.x, self.y, self.yaw)}")
        super().play(motion)

class TestHouse(unittest.TestCase):
    def test_rooms_and_items(self):
        self.assertEqual(set(house.ROOMS), {"living_room", "kitchen", "bedroom", "bathroom"})
        labels = {it["label"] for it in house.ITEMS}
        for need in ("sofa", "TV remote", "television", "bed", "fridge", "dining table", "toilet", "bathtub", "wardrobe", "kitchen counter"): self.assertIn(need, labels)
        self.assertNotIn("resident", labels)
        for it in house.ITEMS:
            self.assertEqual(house.room_of(it["x"], it["y"]) is not None, True, it["label"])
            self.assertTrue(0 < it["x"] < house.W and 0 < it["y"] < house.H, it["label"])

    def test_room_start_points_are_free(self):
        for name, (start, _, _) in house.ROOMS.items():
            self.assertEqual(house.room_of(*start[:2]), name)
            for o in house.obstacles():
                self.assertFalse(abs(start[0] - o["x"]) < o["sx"] / 2 + 0.3 and abs(start[1] - o["y"]) < o["sy"] / 2 + 0.3, (name, o["label"]))

    def test_planner_connects_every_pair_of_rooms(self):
        for a, (sa, _, _) in house.ROOMS.items():
            for b, (sb, _, _) in house.ROOMS.items():
                self.assertIsNotNone(house.plan_path(sa[:2], sb[:2], house.obstacles(), HumanoidSession.CLEAR), (a, b))

    def test_planner_refuses_goal_in_furniture(self):
        bed = [i for i in house.ITEMS if i["label"] == "bed"][0]
        self.assertIsNone(house.plan_path((2.5, 2.5), (bed["x"], bed["y"]), house.obstacles(), HumanoidSession.CLEAR))

    def test_world_file_matches_house(self):
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import make_house_world
        text = make_house_world.house_world()
        with open(make_house_world.OUT, encoding="utf-8") as f: shipped = f.read()   # Webots may re-save the file with its own layout: compare content, not text
        self.assertNotIn("nan", shipped.lower().replace("nao", "").replace("ornament", ""))
        for part in (text, shipped):
            self.assertIn('controller "<extern>"', part); self.assertIn("DEF ORBIT_VIEW Viewpoint", part)
            self.assertIn(f"position {house.W / 2:.3f} {house.H / 2:.3f} 30.000", part)
            self.assertEqual(part.count("Nao {"), 1)
            self.assertNotIn("ResidentSitting", part)
            self.assertIn('EXTERNPROTO "../protos/CleaningBrush.proto"', part)
            self.assertNotIn('name "resident_', part)
            self.assertIn('mode "vacuum"', part)
            self.assertIn('DEF NAO_CLEANING_BRUSH CleaningBrush', part)
            for k, it in enumerate(house.ITEMS): self.assertIn(f'name "{it["label"].replace(" ", "_")}_{k}"', part)
            for p in {it["proto"] for it in house.ITEMS if it["proto"]}: self.assertIn(f'EXTERNPROTO "{house.PROTOS[p]}"', part)

class TestValidationAndGuard(unittest.TestCase):
    def test_argument_check(self):
        ok = lambda t, a: validate_humanoid_call(t, a, False, False)
        self.assertEqual(ok("walk_forward", {"steps": 3}), [])
        for bad in ({"steps": 0}, {"steps": 7}, {"steps": 2.5}, {"steps": True}, {}, {"steps": "2"}): self.assertNotEqual(ok("walk_forward", bad), [], bad)
        self.assertNotEqual(ok("turn", {"direction": "left", "degrees": 90}), [])
        self.assertNotEqual(ok("go_to_room", {"room": "garage"}), [])
        self.assertNotEqual(ok("go_to_object", {"name": "piano"}), [])
        self.assertNotEqual(ok("go_to", {"x": "1", "y": 2}), []); self.assertNotEqual(ok("go_to", {"x": float("nan"), "y": 2}), [])
        self.assertNotEqual(ok("fly", {}), [])
        self.assertIn("stopped", " ".join(validate_humanoid_call("wave", {}, True, False)))
        self.assertIn("fallen", " ".join(validate_humanoid_call("walk_forward", {"steps": 1}, False, True)))
        self.assertEqual(validate_humanoid_call("stand_up", {}, False, True), [])

    def test_guard(self):
        g, room, obs = HumanoidGuard(), (house.W, house.H), house.obstacles()
        self.assertIsNone(g.check("forward", (4.8, 3.1, 180), room, obs, []))
        self.assertIn("bed", g.check("forward", (7.4, 1.3, 0), room, obs, []))                 # bed edge at x 7.82
        self.assertIn("wall", g.check("forward", (5.5, 3.3, 0), room, obs, []))
        self.assertIn("no-go", g.check("forward", (4.8, 3.1, 180), room, obs, [{"x": 4.4, "y": 3.1, "r": 0.2}]))
        self.assertIsNone(g.check("turn_left_60", (7.4, 1.3, 0), room, obs, []))              # turning on the spot is not a walk

    def test_turn_motions_reach_every_20_degrees(self):
        for target in range(-180, 181, 20):
            yaw = 0
            for m in HumanoidSession.turn_motions(target): yaw += FakeNao.TURN[m]
            self.assertEqual((yaw - target) % 360, 0, target)

class TestSession(unittest.TestCase):
    def test_webots_motion_parser_reuses_starred_joint_angles(self):
        class Supervisor:
            def getDevice(self, name): return name
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "HandWave.motion")
            with open(path, "w", encoding="utf-8") as f:
                f.write("#WEBOTS_MOTION,V1.0,JointA,JointB\n"
                        "00:00:000,Pose1,-1.5,0.2\n"
                        "00:00:040,Pose2,*,0.3\n"
                        "00:00:080,Pose3,-1.4,*\n")
            nao = object.__new__(WebotsNao); nao.motion_dir = d; nao.sup = Supervisor()
            motors, rows = nao._load_motion("wave")
        self.assertEqual(motors, ["JointA", "JointB"])
        self.assertEqual(rows, [(0, [-1.5, 0.2]), (40, [-1.5, 0.3]), (80, [-1.4, 0.3])])

    def test_webots_motion_parser_rejects_star_before_first_joint_value(self):
        class Supervisor:
            def getDevice(self, name): return name
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(d, "HandWave.motion"), "w", encoding="utf-8") as f:
                f.write("#WEBOTS_MOTION,V1.0,JointA\n00:00:000,Pose1,*\n")
            nao = object.__new__(WebotsNao); nao.motion_dir = d; nao.sup = Supervisor()
            with self.assertRaisesRegex(RuntimeError, "repeats an angle before defining one"):
                nao._load_motion("wave")

    def test_webots_brush_mode_refreshes_proto_node_and_keeps_it_with_nao(self):
        class VectorField:
            def __init__(self, value): self.value = value
            def setSFVec3f(self, value): self.value = value
            def setSFRotation(self, value): self.value = value

        class Brush:
            def __init__(self, supervisor, mode):
                self.fields = {"mode": ModeField(supervisor), "translation": VectorField([0, 0, 0]),
                               "rotation": VectorField([0, 0, 1, 0])}
                self.fields["mode"].value = mode
            def getField(self, name): return self.fields.get(name)
            def getPosition(self): return self.fields["translation"].value

        class ModeField:
            def __init__(self, supervisor): self.supervisor, self.value = supervisor, "vacuum"
            def getSFString(self): return self.value
            def setSFString(self, value):
                self.supervisor.brush = Brush(self.supervisor, value)
                self.value = value

        class Supervisor:
            def __init__(self): self.brush = Brush(self, "vacuum")
            def getFromDef(self, name): return self.brush
            def step(self, timestep): return 0

        class Robot:
            def getPosition(self): return [4.8, 3.1, 0.334]
            def getOrientation(self): return [-1, 0, 0, 0, -1, 0, 0, 0, 1]

        nao = object.__new__(WebotsNao)
        nao.sup, nao.node, nao.dt = Supervisor(), Robot(), 20
        nao.brush = nao.sup.brush
        nao.brush_mode_field = nao.brush.getField("mode")
        nao.brush_mode = "vacuum"
        nao.set_brush_mode("mop")
        self.assertIs(nao.brush, nao.sup.brush)
        self.assertEqual(nao.brush_mode, "mop")
        position = nao.brush.getPosition()
        self.assertAlmostEqual(position[0], 4.48)
        self.assertAlmostEqual(position[1], 3.1)
        self.assertEqual(position[2], 0.0005)

    def test_go_to_every_room_and_object_never_bypasses_guard(self):
        targets = [("go_to_room", {"room": r}) for r in house.ROOMS] + [("go_to_object", {"name": n}) for n in sorted({i["label"] for i in house.ITEMS})]
        for start in house.ROOMS:
            for tool, args in targets:
                nao = SpyNao(self); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene(start); nao.session = s
                r = s.robot_call(tool, args)
                self.assertEqual(r["status"], "ok", (start, tool, args, r["message"]))
                if tool == "go_to_room": self.assertEqual(s._pose()["room"], args["room"])

    def test_go_to_object_ends_next_to_and_facing_it(self):
        s = session("living_room"); r = s.robot_call("go_to_object", {"name": "fridge"})
        self.assertEqual(r["status"], "ok"); p = r["data"]
        fridge = [i for i in house.ITEMS if i["label"] == "fridge"][0]
        import math
        self.assertLess(math.hypot(p["x"] - fridge["x"], p["y"] - fridge["y"]), 1.6)
        b = (math.degrees(math.atan2(fridge["y"] - p["y"], fridge["x"] - p["x"])) - p["yaw_deg"] + 180) % 360 - 180
        self.assertLessEqual(abs(b), 10.5)

    def test_greet_uses_guarded_hand_wave(self):
        nao = SpyNao(self); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene("living_room"); nao.session = s
        r = s.robot_call("greet", {})
        self.assertEqual(r["status"], "ok")
        self.assertEqual(nao.played, ["wave"])

    def test_turn_to_object_faces_item_without_walking(self):
        nao = SpyNao(self); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene("living_room"); nao.session = s
        start = (nao.x, nao.y)
        tv = next(item for item in house.ITEMS if item["label"] == "television")
        r = s.robot_call("turn_to_object", {"name": "television"})
        bearing = (math.degrees(math.atan2(tv["y"] - nao.y, tv["x"] - nao.x)) - nao.yaw + 180) % 360 - 180
        self.assertEqual(r["status"], "ok", r["message"])
        self.assertEqual((nao.x, nao.y), start)
        self.assertLessEqual(abs(bearing), 18)
        self.assertTrue(s._verify(s.last_task)["passed"])

    def test_join_tv_watching_safely_positions_nao_and_faces_the_screen(self):
        nao = SpyNao(self); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene("living_room"); nao.session = s
        r = s.robot_call("join_tv_watching", {})
        tv = next(item for item in house.ITEMS if item["label"] == "television")
        self.assertEqual(r["status"], "ok", r["message"])
        self.assertLess(math.hypot(nao.x - 1.6, nao.y - 1.65), 0.3)
        bearing = (math.degrees(math.atan2(tv["y"] - nao.y, tv["x"] - nao.x)) - nao.yaw + 180) % 360 - 180
        self.assertLessEqual(abs(bearing), 10.5)

    def test_tour_house_visits_all_rooms_and_returns_to_start(self):
        nao = SpyNao(self); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene("living_room"); nao.session = s
        r = s.robot_call("tour_house", {"finish": "start"})
        self.assertEqual(r["status"], "ok", r["message"])
        self.assertEqual(set(r["data"]["visited"]), set(house.ROOMS))
        self.assertEqual(r["data"]["pose"]["room"], "living_room")
        self.assertLess(math.hypot(nao.x - s.start_pose[0], nao.y - s.start_pose[1]), 0.6)
        self.assertTrue(s.robot_call("verify_task", {})["data"]["passed"])

    def test_clean_room_navigates_sweeps_and_reports_measured_coverage(self):
        s = session("living_room")
        r = s.robot_call("clean_room", {"room": "kitchen", "pattern": "quick", "mode": "mop"})
        self.assertEqual(r["status"], "ok", r["message"])
        self.assertEqual(s._pose()["room"], "kitchen")
        self.assertEqual(s.nao.brush_mode, "mop")
        self.assertEqual(s.cleaning["mode"], "mop")
        self.assertIn("kitchen", s.clean_status()["rooms"])
        self.assertIn("coverage", r["data"])
        self.assertTrue(s._verify(s.last_task)["passed"])

    def test_clean_switches_between_physical_vacuum_and_mop_modes(self):
        s = session()
        vacuum = s.robot_call("clean", {"mode": "vacuum"})
        self.assertEqual(vacuum["status"], "ok")
        self.assertIn("vacuum brush", vacuum["message"])
        self.assertEqual(s.nao.brush_mode, "vacuum")
        self.assertEqual(s.nao.played, [])
        mop = s.robot_call("clean", {"mode": "mop"})
        self.assertEqual(mop["status"], "ok")
        self.assertIn("mop brush", mop["message"])
        self.assertEqual(s.nao.brush_mode, "mop")
        self.assertEqual(s.clean_status()["mode"], "mop")
        self.assertEqual(s.nao.played, [])

    def test_run_plan_keeps_cleaning_inactive_when_room_is_already_swept(self):
        s = session()
        s.cleaning["swept"] = ["living_room"]
        node = s._node(1, {"room": "living_room", "do": "clean"}, "quick", s._pose())
        result = s._do_step(node)
        self.assertEqual(result["status"], "ok")
        self.assertFalse(s.clean_status()["active"])

    def test_clean_room_refuses_while_paused_without_moving(self):
        s = session("living_room")
        s.robot_call("pause_cleaning", {})
        before = s.nao.pose()
        r = s.robot_call("clean_room", {"room": "kitchen"})
        self.assertEqual(r["status"], "rejected")
        self.assertEqual(s.nao.pose(), before)

    def test_clean_room_repeats_quick_sweep_when_full_coverage_is_requested(self):
        s = session("kitchen")
        s.cleaning.setdefault("swept", []).append("kitchen")
        calls = []
        s._t_go_to_room = lambda room: {"status": "ok", "message": "already in room"}
        s._sweep_room = lambda room, pattern: calls.append((room, pattern))
        r = s.robot_call("clean_room", {"room": "kitchen", "pattern": "full"})
        self.assertEqual(r["status"], "failed")
        self.assertEqual(calls, [("kitchen", "full")])

    def test_rejected_and_vetoed_do_not_move(self):
        s = session("bedroom"); s.nao.set_pose(7.0, 1.9, 0)                     # facing the bed
        statuses = [s.robot_call("walk_forward", {"steps": 1})["status"] for _ in range(6)]
        self.assertIn("vetoed", statuses); self.assertEqual(statuses[-1], "vetoed")
        p0, n0 = s.nao.pose(), len(s.nao.played)
        self.assertEqual(s.robot_call("walk_forward", {"steps": 9})["status"], "rejected")
        self.assertEqual(s.robot_call("go_to", {"x": 20, "y": 3})["status"], "rejected")
        r = s.robot_call("walk_forward", {"steps": 3}); self.assertEqual(r["status"], "vetoed"); self.assertIn("bed", r["message"])
        self.assertEqual((s.nao.pose(), len(s.nao.played)), (p0, n0))
        bed = [i for i in house.ITEMS if i["label"] == "bed"][0]
        self.assertGreaterEqual(bed["x"] - bed["sx"] / 2 - p0[0], 0.3 - 1e-9)     # never closer than the guard's clearance

    def test_guard_lets_a_robot_that_is_too_close_back_away(self):
        g, room, obs = HumanoidGuard(), (house.W, house.H), house.obstacles()
        self.assertIsNotNone(g.check("forward", (7.75, 1.9, 0), room, obs, []))   # 0.07 m from the bed: closer is refused
        self.assertIsNone(g.check("backward", (7.75, 1.9, 0), room, obs, []))     # getting clear is allowed
        self.assertIsNone(g.check("forward", (7.75, 1.9, 180), room, obs, []))

    def test_no_go_zone_is_respected(self):
        s = session("living_room"); s.set_no_go_zone(5.0, 4.1, 0.5)          # in front of the kitchen door
        r = s.robot_call("go_to_room", {"room": "kitchen"})
        self.assertEqual(r["status"], "rejected"); self.assertIn("no safe path", r["message"])
        with self.assertRaises(ValueError): s.set_no_go_zone(1, 1, 10)

    def test_unknown_obstacle_is_bumped_and_avoided(self):
        box = {"label": "box", "x": 4.65, "y": 6.0, "sx": 0.3, "sy": 0.3}         # in the kitchen, not on the map
        nao = SpyNao(self, extra_obstacles=[box]); s = HumanoidSession(log_dir=False, backend=nao); s.load_scene("kitchen"); nao.session = s
        s.nao.set_pose(3.9, 5.2, 40)
        r = s.robot_call("go_to", {"x": 5.4, "y": 6.8})
        self.assertEqual(r["status"], "ok", r["message"]); self.assertTrue(s.bumps or s.sonar_obs)
        self.assertTrue(all(abs(b["x"] - 4.65) < 0.5 and abs(b["y"] - 6.0) < 0.5 for b in s.bumps + s.sonar_obs))

    def test_fall_and_stand_up(self):
        s = session(); s.nao.fall_on = {"forward"}
        r = s.robot_call("walk_forward", {"steps": 1}); self.assertEqual(r["status"], "failed"); self.assertIn("fell", r["message"])
        self.assertEqual(s.robot_call("wave", {})["status"], "rejected")
        s.nao.fall_on = set()
        self.assertEqual(s.robot_call("stand_up", {})["status"], "ok"); self.assertEqual(s.robot_call("wave", {})["status"], "ok")

    def test_stop_latches_until_load_scene(self):
        s = session(); s.robot_call("stop_robot", {})
        for t, a in (("walk_forward", {"steps": 1}), ("go_to_room", {"room": "kitchen"}), ("wave", {})): self.assertEqual(s.robot_call(t, a)["status"], "rejected")
        s.load_scene("kitchen"); self.assertEqual(s.robot_call("wave", {})["status"], "ok")

    def test_call_budget(self):
        s = session(max_calls=3)
        for _ in range(3): self.assertEqual(s.robot_call("get_pose", {})["status"], "ok")
        self.assertEqual(s.robot_call("get_pose", {})["status"], "rejected")

    def test_describe_view_hides_items_behind_walls(self):
        s = session("living_room"); s.nao.set_pose(3.0, 2.8, 90)                # facing north, the kitchen behind the wall
        seen = {o["label"] for o in s.robot_call("describe_view", {})["data"]}
        self.assertIn("TV stand", seen); self.assertNotIn("kitchen counter", seen)

    def test_map_shows_walls_robot_and_zones(self):
        s = session(); s.set_no_go_zone(1.5, 3.5, 0.4); m = s.believed_map()["map"].splitlines()
        self.assertEqual(len(m), 32); self.assertTrue(all(len(r) == 40 for r in m))
        self.assertEqual(set(m[0]), {"|"}); self.assertIn("<", "".join(m)); self.assertIn("X", "".join(m)); self.assertIn("#", "".join(m))

    def test_look_without_webots_is_a_clear_failure(self):
        r = session().robot_call("look", {}); self.assertEqual(r["status"], "failed"); self.assertIn("Webots", r["message"])

class TestMcp(unittest.TestCase):
    def call(self, srv, name, args=None):
        return srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": name, "arguments": args or {}}})["result"]

    def test_tool_list_and_schemas(self):
        srv = HumanoidMcpServer(session(), "test")
        tools = {t["name"]: t for t in srv.tool_defs()}
        for t in HUMANOID_TOOLS: self.assertIn(t, tools)
        for t in ("house_info", "load_scene", "list_scenes", "set_no_go_zone", "get_map"): self.assertIn(t, tools)
        self.assertEqual(set(tools["go_to_room"]["inputSchema"]["properties"]["room"]["enum"]), set(house.ROOMS))
        self.assertIn("fridge", tools["go_to_object"]["inputSchema"]["properties"]["name"]["enum"])
        self.assertNotIn("resident", tools["go_to_object"]["inputSchema"]["properties"]["name"]["enum"])
        self.assertIn("turn_to_object", tools)
        self.assertEqual(tools["load_scene"]["inputSchema"]["properties"]["scene"]["enum"], sorted(house.ROOMS))
        self.assertEqual(set(tools["tour_house"]["inputSchema"]["properties"]["finish"]["enum"]), {"stay", "start"})
        self.assertEqual(set(tools["clean_room"]["inputSchema"]["properties"]["room"]["enum"]), set(house.ROOMS))
        self.assertIn("greet", tools)
        self.assertNotIn("greet_resident", tools)
        self.assertIn("join_tv_watching", tools)

    def test_calls_and_errors(self):
        srv = HumanoidMcpServer(session(), "test")
        r = self.call(srv, "go_to_room", {"room": "bathroom"}); self.assertFalse(r["isError"])
        self.assertEqual(json.loads(r["content"][0]["text"])["data"]["room"], "bathroom")
        self.assertTrue(self.call(srv, "walk_forward", {"steps": 0})["isError"])
        self.assertTrue(self.call(srv, "load_scene", {"scene": "garage"})["isError"])
        info = json.loads(self.call(srv, "house_info")["content"][0]["text"]); self.assertEqual(len(info["items"]), len(house.ITEMS))

    def test_image_content(self):
        class CamNao(FakeNao):
            def camera_png(self): return b"\x89PNG\r\n\x1a\nfake"
        s = HumanoidSession(log_dir=False, backend=CamNao()); s.load_scene("living_room")
        r = self.call(HumanoidMcpServer(s, "test"), "look")
        self.assertFalse(r["isError"]); self.assertEqual(r["content"][1]["type"], "image"); self.assertEqual(r["content"][1]["mimeType"], "image/png")
        self.assertNotIn("_image", r["content"][0]["text"])

    def test_selftest_subprocess(self):
        p = subprocess.run([sys.executable, SERVER, "--selftest", "--robot", "nao", "--sim", "fake", "--log-dir", tempfile.mkdtemp(prefix="orbit_nao_")],
                           capture_output=True, text=True, timeout=300, cwd=ROOT)
        self.assertEqual(p.returncode, 0, p.stdout[-3000:] + p.stderr[-3000:]); self.assertIn("SELF-TEST PASSED", p.stdout)

    def test_bad_robot_sim_combination(self):
        p = subprocess.run([sys.executable, SERVER, "--robot", "nao", "--sim", "world2d", "--print-config"], capture_output=True, text=True, timeout=60, cwd=ROOT)
        self.assertNotEqual(p.returncode, 0); self.assertIn("--sim fake or --sim webots", p.stderr)

    def test_print_config_nao(self):
        p = subprocess.run([sys.executable, SERVER, "--robot", "nao", "--print-config"], capture_output=True, text=True, timeout=60, cwd=ROOT)
        self.assertEqual(p.returncode, 0, p.stderr); self.assertIn("orbit-nao-webots", json.loads(p.stdout)["mcpServers"])

    def test_vscode_config_has_nao(self):
        with open(os.path.join(ROOT, ".vscode", "mcp.json"), encoding="utf-8") as f: servers = json.load(f)["servers"]
        self.assertEqual(set(servers), {"orbit-nao-webots"})
        self.assertIn("--robot", servers["orbit-nao-webots"]["args"]); self.assertIn("webots", servers["orbit-nao-webots"]["args"])


class TestConnectionStatus(unittest.TestCase):
    """v0.16.5 diagnostic tool: reports the direct-1235 transport and simulator health, read-only."""
    def test_reports_direct_transport_and_no_1240(self):
        s = HumanoidSession(sim="fake", log_dir=False)
        cs = s.connection_status()
        self.assertEqual(cs["transport"], "in-process (direct)")
        self.assertFalse(cs["uses_port_1240"])
        self.assertEqual(cs["simulator"], "fake")
        self.assertEqual(cs["call_budget"]["remaining"], cs["call_budget"]["max"])

    def test_not_connected_before_first_call_then_connected(self):
        s = HumanoidSession(sim="fake", log_dir=False)
        self.assertFalse(s.connection_status()["connected"])    # no backend built yet; must not force a connection
        self.assertIsNone(s._backend)
        s.load_scene("kitchen")                                  # builds the backend
        cs = s.connection_status()
        self.assertTrue(cs["connected"]); self.assertEqual(cs["backend"], "fake"); self.assertTrue(cs["ready"])

    def test_webots_url_reported_without_connecting(self):
        # sim=webots but we never call the robot, so no Webots is contacted; the controller endpoint is still reported.
        s = HumanoidSession(sim="webots", log_dir=False)
        cs = s.connection_status()
        self.assertEqual(cs["controller_port"], 1235)
        self.assertIn("1235", cs["controller_url"]); self.assertFalse(cs["uses_port_1240"])
        self.assertFalse(cs["connected"])                        # backend not built, so not connected
        self.assertIsNone(s._backend)                            # a status read must never trigger a Webots connection

    def test_advertised_as_a_session_tool_over_mcp(self):
        s = session("living_room", sim="fake")
        defs = HumanoidMcpServer(s, "test").tool_defs()
        names = {d["name"] for d in defs}
        self.assertIn("connection_status", names)
        payload, is_error = HumanoidMcpServer(s, "test").call_tool("connection_status", {})
        self.assertFalse(is_error)
        self.assertEqual(payload["transport"], "in-process (direct)")


class TestConnectionLoss(unittest.TestCase):
    """v0.16.5 reliability: a lost backend connection becomes a clean, logged {status:error}, never an exception out of robot_call,
    and must not break unrelated read-only MCP functionality. Reproduces the Webots 'simulation ended' disconnect without a real Webots."""
    @staticmethod
    def _kill_backend(s):
        b = s.nao
        def dead(*a, **k): raise RuntimeError("Webots simulation ended")
        b.step = dead; b.pose = dead     # a dropped extern controller: even pose reads fail
        return b

    def test_motion_call_during_loss_returns_error_not_raise(self):
        s = session("living_room", sim="fake")
        self._kill_backend(s)
        r = s.robot_call("walk_forward", {"steps": 1})     # must not raise
        self.assertEqual(r["status"], "error")
        self.assertIn("connection was lost", r["message"])
        self.assertEqual(s.calls["error"], 1)              # the failure was counted via _finish, not skipped

    def test_read_call_during_loss_returns_error_not_raise(self):
        s = session("living_room", sim="fake")
        self._kill_backend(s)
        self.assertEqual(s.robot_call("get_pose", {})["status"], "error")

    def test_connection_status_during_loss_is_graceful_and_never_1240(self):
        # The FAKE backend has no real link to lose, so it stays connected=True by design; the Webots-only disconnect probe
        # (sup.getSelf()) reporting connected=False is covered by the live Webots fault test (see docs/RELIABILITY_REPORT.md).
        # What must hold for every backend: connection_status never raises and never claims port 1240.
        s = session("living_room", sim="fake")
        self._kill_backend(s)
        cs = s.connection_status()                         # must not raise
        self.assertFalse(cs["uses_port_1240"])
        self.assertEqual(cs["transport"], "in-process (direct)")

    def test_unrelated_readonly_mcp_survives_loss(self):
        s = session("living_room", sim="fake")
        self._kill_backend(s)
        self.assertIn("rooms", s.house_info())             # static session info keeps working

    def test_stop_latch_survives_a_loss(self):
        # Safety invariant: stop_robot latches; a connection loss must not silently clear it.
        s = session("living_room", sim="fake")
        s.robot_call("stop_robot", {})
        self._kill_backend(s)
        r = s.robot_call("walk_forward", {"steps": 1})
        self.assertEqual(r["status"], "rejected")          # stop-latch rejects before any backend access
        self.assertIn("stopped", r["message"])


class TestMalformedToolArgs(unittest.TestCase):
    """Phase 9 regression: a tool call with malformed arguments (e.g. an LLM emitting an extra/empty key) must be turned
    into a clean {status:error} tool result by the MCP dispatch, never an uncaught TypeError. Mirrors the production
    HumanoidMcpServer._dispatch path that orbit_chat and the Phase 9 runner rely on."""
    def _server(self):
        s = HumanoidSession(sim="fake", log_dir=False, memory=False); s.load_scene("living_room")
        return HumanoidMcpServer(s, "test")

    def test_extra_empty_key_becomes_tool_error_not_exception(self):
        srv = self._server()
        resp = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                           "params": {"name": "join_tv_watching", "arguments": {"": "x"}}})
        self.assertIn("result", resp)                       # dispatch returned a normal result, did not raise
        self.assertTrue(resp["result"]["isError"])
        payload = json.loads(resp["result"]["content"][0]["text"])
        self.assertEqual(payload["status"], "error")

    def test_wrong_type_argument_becomes_tool_error(self):
        srv = self._server()
        resp = srv.handle({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                           "params": {"name": "go_to", "arguments": {"x": "not-a-number", "y": 3}}})
        self.assertIn("result", resp)
        self.assertTrue(resp["result"]["isError"])          # rejected by argument validation, no crash

if __name__ == "__main__": unittest.main()
