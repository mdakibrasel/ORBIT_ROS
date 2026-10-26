import unittest
from orbit_ros.world2d import World2D, Obj
from orbit_ros.tasks2d import make_task, left_region, check_success, FAMILIES
from orbit_ros.cognition import SensorInterpreter, verify_plan
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend, LLMBackend
from orbit_ros.llm import FakeClient

class WorldTests(unittest.TestCase):
    def test_turn_and_move_conventions(self):
        w = World2D(9, 7, (4, 3), 0, [])
        w.execute("move_forward", {"distance": 2}); self.assertEqual((w.x, w.y), (6, 3))
        w.execute("turn", {"direction": "left", "degrees": 90}); self.assertEqual(w.yaw, 90)
        w.execute("move_forward", {"distance": 1}); self.assertEqual((w.x, w.y), (6, 4))      # left of +x is +y
        w.execute("turn", {"direction": "right", "degrees": 180}); self.assertEqual(w.yaw, 270)
    def test_collision_and_bounds(self):
        w = World2D(5, 5, (2, 2), 0, [Obj("a", "box", 4, 2)])
        r = w.execute("move_forward", {"distance": 5}); self.assertFalse(r["ok"]); self.assertEqual(w.x, 3); self.assertEqual(w.collisions, 1)
    def test_invalid_calls_counted(self):
        w = World2D(5, 5, (2, 2), 0, [])
        self.assertFalse(w.execute("fly", {})["ok"]); self.assertFalse(w.execute("move_forward", {"distance": 1.5})["ok"])
        self.assertEqual(w.invalid_calls, 2)
    def test_camera_sign_and_mirroring(self):
        obj = [Obj("a", "red box", 6, 5)]                                    # forward 2, left 2 from (4,3) yaw 0 -> bearing +45 (left)
        self.assertLess(World2D(9, 7, (4, 3), 0, obj).describe_view()[0]["image_x"], 0)     # left -> image left (negative)
        self.assertGreater(World2D(9, 7, (4, 3), 0, obj, mirrored_camera=True).describe_view()[0]["image_x"], 0)
    def test_grounded_interpreter_recovers_world_cell(self):
        for mir in (False, True):
            for yaw in (0, 90, 180, 270):
                w = World2D(9, 7, (4, 3), yaw, [], mirrored_camera=mir)
                from orbit_ros.tasks2d import world_from_robot_frame
                cell = world_from_robot_frame((4, 3), yaw, 3, 1); w.objects = [Obj("t", "t", *cell)]
                it = SensorInterpreter(90, mir, True).interpret(w.describe_view(), w.pose())
                self.assertEqual(tuple(it[0]["world_xy"]), cell)
    def test_ungrounded_mirrored_flips_side(self):
        w = World2D(9, 7, (4, 3), 0, [Obj("t", "t", 6, 5)], mirrored_camera=True)
        g = SensorInterpreter(90, True, True).interpret(w.describe_view(), w.pose())[0]["bearing_deg"]
        u = SensorInterpreter(90, True, False).interpret(w.describe_view(), w.pose())[0]["bearing_deg"]
        self.assertGreater(g, 0); self.assertLess(u, 0)

class TaskTests(unittest.TestCase):
    def test_object_count_factor_matches_world_for_motion_and_cleaning(self):
        for family in ("motion", "clean_left"):
            for n in (1, 3, 5):
                w, _ = make_task(family, 23, 0, n, False, 1)
                self.assertEqual(len(w.objects), n, (family, n, len(w.objects)))

    def test_paired_determinism(self):
        a, _ = make_task("goto", 3, 90, 3, True); b, _ = make_task("goto", 3, 90, 3, True)
        self.assertEqual([(o.label, o.x, o.y) for o in a.objects], [(o.label, o.x, o.y) for o in b.objects])
    def test_left_region_depends_on_heading(self):
        self.assertTrue(all(y >= 2 for _, y in left_region(6, 4, (3, 2, 0))))      # facing +x, left = +y
        self.assertTrue(all(x <= 2 for x, _ in left_region(6, 4, (3, 2, 90))))     # facing +y, left = -x
    def test_clean_left_world_free_cells_are_connected(self):
        from orbit_ros.tasks2d import _connected
        for n in (1, 3, 5):
            w, _ = make_task("clean_left", 17, 90, n, False, 1)
            free = {(x, y) for x in range(w.w) for y in range(w.h) if w.free(x, y)}
            self.assertTrue(_connected(w, free))

    def test_all_families_solvable_by_noise_free_oracle(self):
        b = OracleBackend(0, 0)
        for fam in FAMILIES:
            for yaw in (0, 90, 180, 270):
                for m in (False, True):
                    w, t = make_task(fam, 1, yaw, 3, m, 2)
                    self.assertTrue(run_episode(w, t, PRESETS["S5"], b)["success"], t.task_id)

class AgentTests(unittest.TestCase):
    def test_verifier_catches_collision_and_expect_mismatch(self):
        pose = {"x": 1, "y": 1, "yaw_deg": 0, "room_w": 5, "room_h": 5}
        plan = {"actions": [{"tool": "move_forward", "args": {"distance": 3}}], "expect": {"x": 4, "y": 1}}
        self.assertEqual(verify_plan(plan, pose, []), [])
        self.assertTrue(verify_plan(plan, pose, [(3, 1)]))                          # predicted collision
        plan["expect"] = {"x": 3, "y": 1}; self.assertTrue(verify_plan(plan, pose, []))  # declared != simulated
    def test_memory_matters_only_with_memory(self):
        b = OracleBackend(0, 0); ok = {}
        for name in ("S2", "S3"):
            ok[name] = sum(run_episode(*make_task("remember_return", s, 0, 2, False), PRESETS[name], b)["success"] for s in range(8))
        self.assertGreater(ok["S3"], ok["S2"])
    def test_s4_aborts_bad_plan_without_using_s5_repair(self):
        class BadPlanBackend:
            calls = 0
            usage_events = []
            repair_calls = 0
            def begin_episode(self, key): pass
            def plan(self, ph, ctx):
                self.calls += 1
                return {"actions": [], "expect": {"x": -99, "y": -99}}
            def repair(self, ph, ctx, failure):
                self.repair_calls += 1
                self.calls += 1
                return {"actions": []}
        backend = BadPlanBackend()
        w, t = make_task("motion", 0, 0, 1, False, 1)
        row = run_episode(w, t, PRESETS["S4"], backend)
        self.assertEqual(backend.repair_calls, 0)
        self.assertEqual(row["repairs"], 0)
        self.assertEqual(row["aborted"], 1)
        self.assertEqual(row["success"], 0)

    def test_llm_backend_plumbing_with_fake_client(self):
        reply = '{"actions":[{"tool":"move_forward","args":{"distance":2}},{"tool":"turn","args":{"direction":"right","degrees":90}}],"expect":{"x":6,"y":3,"yaw_deg":270}}'
        client = FakeClient([reply, '{"done": true}'])
        w, t = make_task("motion", 0, 0, 1, False, 1)
        row = run_episode(w, t, PRESETS["S4"], LLMBackend(client, True, True))
        self.assertEqual(row["llm_calls"], 2); self.assertEqual(row["actions_executed"], 2)
        self.assertEqual(row["input_tokens"] > 0, True)

if __name__ == "__main__": unittest.main()
