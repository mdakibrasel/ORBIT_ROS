"""Adapter tests that need no Webots. FakeSim runs are plumbing tests, not Webots evidence."""
import ast, json, os, tempfile, unittest
from orbit_ros.tasks2d import make_task
from orbit_ros.world2d import World2D, Obj
from orbit_ros.webots_adapter import FakeSim, WebotsWorld2D, solid_vrml, wall_specs, calibrate_lidar
from orbit_ros.continuous import FakePlant, WHEEL_R
import run_webots_c1 as c1

ROOT = os.path.join(os.path.dirname(__file__), "..")

class BrokenSim(FakeSim):
    """Camera that forgets to flip left/right: the harness must notice."""
    def recognition(self):
        return [dict(r, px=self.camera_width - r["px"]) for r in super().recognition()]

class SlippySim(FakeSim):
    """Real wheels move 5% further than the encoders report: odometry drifts, and the harness must show it."""
    def __init__(self):
        super().__init__(); self._plant.scale = 1.05
        orig = self._plant.step
        def step():
            ox, oy, oyaw = self._plant.truth(); el, er = self._plant.encL, self._plant.encR
            orig()
            nx, ny, nyaw = self._plant.truth()
            self._plant.set_pose(ox + (nx - ox) * 1.05, oy + (ny - oy) * 1.05, nyaw)      # truth moves further than the encoders say
        self._plant.step = step

class TestAdapter(unittest.TestCase):
    def test_solid_vrml_is_recognisable_and_balanced(self):
        s = solid_vrml("o0", "red box", 3, 2)
        for token in ('name "obj_o0"', 'model "red box"', "recognitionColors [", "boundingObject", "Solid {", "translation 3 2 0.3"):
            self.assertIn(token, s)
        self.assertEqual(s.count("{"), s.count("}")); self.assertEqual(s.count("["), s.count("]"))

    def test_simulator_scene_is_what_the_agent_blocks_on(self):
        world, _ = make_task("goto", 3, 0, 3, False, 1)
        w = WebotsWorld2D.adopt(world, FakeSim())
        self.assertEqual(w._cells, {(o.x, o.y) for o in world.objects})
        o = world.objects[0]; w.x, w.y, w.yaw = o.x - 1, o.y, 0; w._push()
        r = w.execute("move_forward", {"distance": 2})
        self.assertFalse(r["ok"]); self.assertEqual(w.collisions, 1)

    def test_pose_is_read_back_from_simulator(self):
        world, _ = make_task("motion", 3, 90, 3, False, 1)
        w = WebotsWorld2D.adopt(world, FakeSim()); w._sim._plant.set_pose(9.0, 9.0, 0.0)     # simulator disagrees with the logic
        self.assertEqual(w.execute("get_pose", {})["data"]["x"], 9)
        w._push(); self.assertEqual(w.pose()["x"], w.x)

    def test_probe_passes_on_correct_mapping_and_fails_on_broken_one(self):
        self.assertTrue(c1.probe(FakeSim(), log=lambda m: None))
        self.assertFalse(c1.probe(BrokenSim(), log=lambda m: None))

    def test_fake_c1_reproduces_world2d_and_detects_a_broken_camera(self):
        with open(os.path.join(ROOT, "configs", "v0_8_1.json")) as f: cfg = json.load(f)
        rows, meta = c1.run_c1(FakeSim(), cfg, "hybrid_old+integrity", "static_n0.0", [3], os.path.join(tempfile.mkdtemp(), "ok"), limit=48, log=lambda m: None)
        self.assertEqual(sum(r["agree"] for r in rows), len(rows))
        self.assertIn("plumbing", meta["note"])
        rows, _ = c1.run_c1(BrokenSim(), cfg, "hybrid_old+integrity", "static_n0.0", [3], os.path.join(tempfile.mkdtemp(), "bad"), limit=48, log=lambda m: None)
        self.assertGreater(sum(r["view_mismatch"] for r in rows), 0)

    def test_walls_surround_the_grid_and_are_not_recognisable(self):
        names = [n for n, *_ in wall_specs(9, 7)]
        self.assertEqual(sorted(names), ["wall_e", "wall_n", "wall_s", "wall_w"])
        from orbit_ros.webots_adapter import wall_vrml
        self.assertNotIn("recognitionColors", wall_vrml(*wall_specs(9, 7)[0]))

    def test_lidar_calibration_finds_the_front_ray_wherever_it_is(self):
        for fi in (0, 90, 270, 359):
            sim = FakeSim(lidar_front_index=fi); self.assertEqual(calibrate_lidar(sim), fi)

    def test_continuous_probe_passes_and_slip_is_visible_in_odometry_error(self):
        self.assertTrue(c1.probe(FakeSim(), log=lambda m: None))
        sim = SlippySim(); c1.calibrate_lidar(sim)
        w = WebotsWorld2D.adopt(World2D(9, 7, (4, 3), 0, []), sim, "camera", "continuous")
        w.execute("move_forward", {"distance": 3})
        self.assertGreater(w.odom_error_max, 0.05)

    def test_bay_runs_inside_the_simulator_and_world_is_reloaded_afterwards(self):
        cfg = self._cfg()
        sim = FakeSim()
        rows, _ = c1.run_c1(sim, cfg, "hybrid_old+integrity", "static_n0.0", [3], os.path.join(tempfile.mkdtemp(), "b"), limit=6, log=lambda m: None)
        self.assertTrue(all(int(r["bay_runs"]) >= 1 for r in rows))
        self.assertEqual(sum(int(r["agree"]) for r in rows), len(rows))

    def test_continuous_episodes_match_world2d_on_the_fake_plant(self):
        cfg = self._cfg()
        rows, _ = c1.run_c1(FakeSim(), cfg, "hybrid_old+integrity", "static_n0.0", [3], os.path.join(tempfile.mkdtemp(), "c"), motion="continuous", limit=24, log=lambda m: None)
        self.assertEqual(sum(int(r["agree"]) for r in rows), len(rows))
        self.assertTrue(all(float(r["odom_error_m"]) < 0.05 for r in rows))

    def test_vscode_files_are_valid_json_and_scripts_exist(self):
        for n in ("settings", "launch", "tasks", "extensions"):
            with open(os.path.join(ROOT, ".vscode", n + ".json")) as f: json.load(f)
        for n in ("setup.ps1", "start_webots.ps1", "run.ps1", "check_env.py", ".env.example"):
            self.assertTrue(os.path.isfile(os.path.join(ROOT, n)), n)

    def _cfg(self):
        with open(os.path.join(ROOT, "configs", "v0_8_1.json")) as f: return json.load(f)

    def test_seed_for_c1_is_a_dev_seed(self):
        with open(os.path.join(ROOT, "configs", "v0_9_seed_registry.json")) as f: reg = json.load(f)
        self.assertIn(3, reg["dev"])

if __name__ == "__main__":
    unittest.main()
