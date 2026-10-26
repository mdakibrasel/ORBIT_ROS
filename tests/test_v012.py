"""v0.12: localiser and visual re-localisation, moving obstacles, self-consistency, model mixing, token budget, vision, ROS 2 helpers."""
import json, math, struct, threading, unittest, zlib
from http.server import HTTPServer
from orbit_ros.tasks2d import make_task, movable_ids
from orbit_ros.world2d import World2D, Obj, Realism
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend, LLMBackend
from orbit_ros.ensemble import SelfConsistent, Mixed, TokenBudget
from orbit_ros.llm import ScriptedClient, OpenAICompatClient
from orbit_ros.vision import render_view
from orbit_ros import ros2_adapter as r2
from orbit_ros.continuous import FakePlant, DiffDrive

def noisy(fam, seed, yaw, odom=0.3, **kw):
    w, t = make_task(fam, seed, yaw, 3, False, 1)
    w.set_realism(Realism(odom_p=odom, seed=t.task_id, **kw)); w.movable = movable_ids(w, t)
    return w, t

class LocaliserTests(unittest.TestCase):
    def test_tool_needs_a_localiser_and_resets_odometry(self):
        w = World2D(9, 3, (0, 1), 0, []); w.set_realism(Realism(odom_p=1.0, seed="x"))
        self.assertFalse(w.execute("localize", {})["ok"])
        w.execute("move_forward", {"distance": 3}); self.assertNotEqual(w.pose()["x"], w.x)
        w.localizer = True; r = w.execute("localize", {})
        self.assertTrue(r["ok"]); self.assertEqual((r["data"]["x"], w.localize_calls), (w.x, 1))
    def test_oracle_uses_the_tool_and_auto_mode_closes_the_loop(self):
        err = {}; calls = 0
        for mode in ("none", "tool", "auto"):
            e = 0; ok = 0
            for s in range(3):
                for yaw in (0, 90, 180, 270):
                    w, t = noisy("clean_left", s, yaw)
                    r = run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0), localizer=mode); e += r["final_pose_error_m"]; ok += r["success"]
                    if mode != "none": self.assertIn("localize_calls", r)
                    if mode == "tool": calls += r["localize_calls"]
            err[mode] = (e, ok)
        self.assertGreater(calls, 0); self.assertEqual(err["auto"][0], 0); self.assertLess(err["tool"][0], err["none"][0])
        self.assertGreater(err["auto"][1], err["none"][1])
    def test_visual_relocalisation_fires_and_is_logged(self):
        n = 0
        for s in range(6):
            for yaw in (0, 90, 180, 270):
                w, t = noisy("goto", s, yaw, odom=0.5)
                n += run_episode(w, t, PRESETS["S5"], OracleBackend(0.0, 0.0), visual_reloc=True)["visual_corrections"]
        self.assertGreater(n, 0)
    def test_off_by_default(self):
        w, t = make_task("goto", 0, 0, 3, False, 1); r = run_episode(w, t, PRESETS["S5"], OracleBackend(0.05, 0.3))
        self.assertNotIn("localize_calls", r); self.assertFalse(w.localizer)

class MovingObstacleTests(unittest.TestCase):
    def test_targets_and_mentioned_objects_never_move(self):
        w, t = make_task("goto", 1, 0, 5, False, 1); mv = movable_ids(w, t)
        self.assertNotIn("o0", mv); self.assertTrue(all(o.label != "red box" for o in w.objects if o.id in mv))
        w.set_realism(Realism(moving_every=1, seed="m")); w.movable = mv
        before = {o.id: (o.x, o.y) for o in w.objects}
        for _ in range(20): w.execute("turn", {"direction": "left", "degrees": 90})
        after = {o.id: (o.x, o.y) for o in w.objects}
        self.assertEqual(before["o0"], after["o0"])
        if mv: self.assertGreater(w.object_moves, 0)
        self.assertEqual(len({(o.x, o.y) for o in w.objects}), len(w.objects))       # no two objects share a cell
        self.assertNotIn((w.x, w.y), {(o.x, o.y) for o in w.objects})
    def test_deterministic(self):
        rows = []
        for _ in range(2):
            w, t = make_task("clean_left", 0, 90, 5, False, 1); w.set_realism(Realism(moving_every=4, seed=t.task_id)); w.movable = movable_ids(w, t)
            rows.append(run_episode(w, t, PRESETS["S2"], OracleBackend(0.05, 0.3))["object_moves"])
        self.assertEqual(rows[0], rows[1])

def plan_json(n): return json.dumps({"actions": [{"tool": "move_forward", "args": {"distance": n}}]})

class EnsembleTests(unittest.TestCase):
    def ctx(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)
        return t.phases[0], {"pose": w.pose(), "obstacles_known": [], "start_pose": w.start_pose}
    def test_self_consistency_returns_the_majority_plan(self):
        replies = iter([plan_json(1), plan_json(2), plan_json(2)])
        cl = ScriptedClient(lambda p, tag: next(replies)); sc = SelfConsistent(LLMBackend(cl, True, False), 3)
        ph, ctx = self.ctx(); ctx.update(interp=[], raw=[], sensor_spec="", phase_start_pose=(0, 0, 0), landmarks={}, cleaned_known=set(), history=[])
        self.assertEqual(sc.plan(ph, ctx)["actions"][0]["args"]["distance"], 2)
        self.assertEqual((sc.calls, sc.votes), (3, [(2, 3)]))
    def test_self_consistency_votes_on_single_actions(self):
        acts = iter(['{"tool":"turn","args":{"direction":"left","degrees":90}}', '{"tool":"clean","args":{}}', '{"tool":"clean","args":{}}'])
        sc = SelfConsistent(LLMBackend(ScriptedClient(lambda p, tag: next(acts)), True, False), 3)
        ph, ctx = self.ctx(); ctx.update(interp=[], raw=[], sensor_spec="", phase_start_pose=(0, 0, 0), landmarks={}, cleaned_known=set(), history=[])
        self.assertEqual(sc.next_action(ph, ctx)["tool"], "clean")
    def test_oracle_episode_with_voting_costs_k_calls_per_step(self):
        w, t = make_task("goto", 0, 0, 3, False, 1); one = run_episode(w, t, PRESETS["C1"], OracleBackend(0.05, 0.3))
        w, t = make_task("goto", 0, 0, 3, False, 1); three = run_episode(w, t, PRESETS["C1"], SelfConsistent(OracleBackend(0.05, 0.3), 3))
        self.assertGreaterEqual(three["llm_calls"], 2 * one["actions_executed"])
    def test_mixed_routes_plans_and_steps_to_different_models(self):
        big = ScriptedClient(lambda p, tag: '{"actions":[{"tool":"move_forward","args":{"distance":1}}]}')
        small = ScriptedClient(lambda p, tag: '{"tool":"done","args":{}}')
        w, t = make_task("motion", 0, 0, 1, False, 1)
        m = Mixed(LLMBackend(big, False, False), LLMBackend(small, False, False))
        row = run_episode(w, t, PRESETS["C4"], m)                     # C4: scratchpad plan (planner) + steps (stepper)
        self.assertTrue(big.prompts and small.prompts)
        self.assertEqual({u["role"] for u in m.usage_events}, {"planner", "stepper"})
        self.assertEqual(row["llm_calls"], len(big.prompts) + len(small.prompts))
    def test_token_budget_stops_the_episode(self):
        cl = ScriptedClient(lambda p, tag: '{"tool":"turn","args":{"direction":"left","degrees":90}}')
        tb = TokenBudget(LLMBackend(cl, False, False), 1)               # any first call spends the budget
        w, t = make_task("goto", 0, 0, 3, False, 1); row = run_episode(w, t, PRESETS["C1"], tb)
        self.assertTrue(tb.exhausted); self.assertEqual(row["llm_calls"], 1)

def png_size(b):
    assert b[:8] == b"\x89PNG\r\n\x1a\n"; return struct.unpack(">II", b[16:24])

class VisionTests(unittest.TestCase):
    def test_render_is_a_valid_png_and_depends_on_the_view(self):
        a = render_view([]); b = render_view([{"label": "red box", "image_x": 0.0, "apparent_size": 0.5}])
        self.assertEqual(png_size(a), (320, 160)); self.assertNotEqual(a, b)
        for lab in ("blue shelf", "red chair", "green ball", "purple thing"):
            self.assertNotEqual(render_view([{"label": lab, "image_x": -0.5, "apparent_size": 1.0}]), a)
    def test_vision_backend_sends_the_image_and_hides_the_object_list(self):
        cl = ScriptedClient(lambda p, tag: '{"actions":[]}')
        w, t = make_task("goto", 0, 90, 3, False, 1)
        run_episode(w, t, PRESETS["S2"], LLMBackend(cl, True, False, vision=True))
        tag, prompt = cl.prompts[0]
        self.assertIn("CAMERA IMAGE", prompt); self.assertNotIn("Interpreted view", prompt); self.assertNotIn('"label"', prompt.split("Pose:")[0])
        self.assertTrue(cl.images[0]); self.assertEqual(png_size(cl.images[0][0]), (320, 160))
    def test_openai_client_sends_images_and_logs_cached_tokens(self):
        from tests.test_llm_openai_compat import Fake
        srv = HTTPServer(("127.0.0.1", 0), Fake); threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            Fake.script = [(200, {"choices": [{"message": {"content": "{}"}}],
                                  "usage": {"prompt_tokens": 50, "completion_tokens": 3, "prompt_tokens_details": {"cached_tokens": 40}}})]; Fake.seen = []
            c = OpenAICompatClient("groq", "m", base_url=f"http://127.0.0.1:{srv.server_port}/v1", api_key="K")
            c.complete("look", "plan", images=[render_view([])])
            content = Fake.seen[0]["body"]["messages"][-1]["content"]
            self.assertEqual(content[0], {"type": "text", "text": "look"}); self.assertTrue(content[1]["image_url"]["url"].startswith("data:image/png;base64,"))
            self.assertEqual((c.usage_events[0]["cached_tokens"], c.usage_events[0]["images"]), (40, 1))
        finally: srv.shutdown(); srv.server_close()

class _StubRos:
    """Stands in for Ros2Sim: twist commands drive a FakePlant, as cmd_vel would drive the simulated robot."""
    def __init__(self):
        self.p = FakePlant(); self.dt = self.p.dt; self.bumper = False
    def publish_twist(self, v, w):
        self.p.set_wheels((v - w * r2.AXLE / 2) / r2.WHEEL_R, (v + w * r2.AXLE / 2) / r2.WHEEL_R)
    def spin_for(self, s): self.p.step()
    @property
    def wheels(self): return self.p.encoders()
    @property
    def scan(self): return self.p.lidar()
    def robot_pose(self): x, y, yaw = self.p.truth(); return (x, y, math.degrees(yaw) % 360)

class Ros2HelperTests(unittest.TestCase):
    def test_kinematics_and_quaternions(self):
        v, w = r2.wheels_to_twist(1.0, 1.0); self.assertAlmostEqual(v, r2.WHEEL_R); self.assertEqual(w, 0)
        v, w = r2.wheels_to_twist(-1.0, 1.0); self.assertEqual(v, 0); self.assertAlmostEqual(w, 2 * r2.WHEEL_R / r2.AXLE)
        for yaw in (0.0, 1.0, -2.5, math.pi / 2): self.assertAlmostEqual(r2.quat_to_yaw(*r2.yaw_to_quat(yaw)), yaw)
    def test_scene_matches_the_webots_geometry(self):
        m = r2.scene_models(6, 4, [Obj("o0", "red box", 2, 1)])
        self.assertEqual([x[0] for x in m], ["obj_o0", "wall_n", "wall_s", "wall_w", "wall_e"])
        self.assertIn("<box><size>0.6 0.6 0.6</size></box>", m[0][1]); self.assertEqual(m[0][2:4], (2.0, 1.0))
        self.assertEqual(r2.wheel_positions(["a", "wheel_left_joint", "wheel_right_joint"], [9, 1.5, 2.5]), (1.5, 2.5))
        self.assertIsNone(r2.wheel_positions(["x"], [1]))
    def test_plant_glue_drives_one_cell_through_twist_commands(self):
        stub = _StubRos(); plant = r2.Ros2Plant(stub); drive = DiffDrive(plant)
        drive.front_idx = 270
        self.assertTrue(drive.rotate(90)); self.assertAlmostEqual(math.degrees(stub.p.yaw), 90, delta=3)
        self.assertEqual(drive.move_one_cell(), "ok"); self.assertAlmostEqual(stub.p.y, 1.0, delta=0.08)
    def test_without_ros_the_error_says_what_to_do(self):
        try: import rclpy  # noqa: F401
        except ImportError:
            with self.assertRaises(RuntimeError) as cm: r2.Ros2Sim()
            self.assertIn("ROS 2 is not available", str(cm.exception))

if __name__ == "__main__": unittest.main()
