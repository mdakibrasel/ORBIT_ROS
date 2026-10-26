"""v0.9.1 instruction ladder: clean_side, sequence, relative, motion level 4, plus a regression check that the core suite is unchanged."""
import csv, math, os, unittest
from orbit_ros.world2d import World2D
from orbit_ros.tasks2d import (make_task, check_success, side_region, FAMILIES, EXTENDED_FAMILIES, CLEAN_SIDES, REL_SELECTS, simulate_ops, world_from_robot_frame)
from orbit_ros.cognition import SensorInterpreter
from orbit_ros.agent2d import S_LADDER, run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend
from run_spatial import grid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YAWS = (0, 90, 180, 270)

def perfect(world, task, cfg="S5"):
    return run_episode(world, task, PRESETS[cfg], OracleBackend(0.0, 0.0))

class SideRegionTests(unittest.TestCase):
    def test_halves_partition_the_room(self):
        for yaw in YAWS:
            sp = (3, 2, yaw); allc = {(x, y) for x in range(6) for y in range(4)}
            for a, b in (("left", "right"), ("front", "back")):
                A, B = side_region(6, 4, sp, a), side_region(6, 4, sp, b)
                self.assertEqual(A | B, allc); self.assertFalse(A & B); self.assertEqual(len(A), len(B))
    def test_front_is_the_half_the_robot_faces(self):
        self.assertTrue(all(x >= 3 for x, _ in side_region(6, 4, (3, 2, 0), "front")))     # facing +x
        self.assertTrue(all(y >= 2 for _, y in side_region(6, 4, (3, 2, 90), "front")))    # facing +y
        self.assertTrue(all(x <= 2 for x, _ in side_region(6, 4, (3, 2, 0), "back")))

class GenerationTests(unittest.TestCase):
    def test_every_extended_instance_builds_with_the_requested_object_count_and_is_deterministic(self):
        n = 0
        for fam, s, yaw, k, m, lv in grid(list(EXTENDED_FAMILIES) + ["motion"], 6, True):
            w, t = make_task(fam, s, yaw, k, m, lv); w2, _ = make_task(fam, s, yaw, k, m, lv)
            self.assertEqual(len(w.objects), k, t.task_id)
            self.assertEqual([(o.label, o.x, o.y) for o in w.objects], [(o.label, o.x, o.y) for o in w2.objects])
            self.assertEqual(len({(o.x, o.y) for o in w.objects} | {w.start_pose[:2]}), k + 1, t.task_id)     # no overlap, none on the start cell
            self.assertTrue(all(w.in_bounds(o.x, o.y) for o in w.objects)); n += 1
        self.assertGreater(n, 500)
    def test_core_grid_is_unchanged(self):
        self.assertEqual(len(list(grid(FAMILIES, 5))), 340)           # the size of the shipped results_spatial_oracle run
    def test_motion_level4_is_a_square_back_at_the_start(self):
        for yaw in YAWS:
            w, t = make_task("motion", 3, yaw, 1, False, 4)
            self.assertEqual(len(t.phases[0].params["ops"]), 8)
            self.assertEqual(t.expected["final_pose"], (*w.start_pose[:2], yaw)); self.assertIn("square", t.phases[0].text)

class RelativeTests(unittest.TestCase):
    def test_referent_matches_its_definition_and_is_visible_from_the_start(self):
        for fam, s, yaw, k, m, lv in grid(["relative"], 5, True):
            w, t = make_task(fam, s, yaw, k, m, lv)
            it = SensorInterpreter(90, m, True).interpret(w.describe_view(), w.pose())
            self.assertEqual(len(it), k, t.task_id)                                   # every object is in view: no occlusion, nothing outside the FOV
            key = {"nearest": lambda o: o["distance_m"], "farthest": lambda o: -o["distance_m"], "leftmost": lambda o: -o["bearing_deg"]}[REL_SELECTS[lv - 1]]
            best = min(it, key=key); tgt = next(o for o in w.objects if o.id == t.expected["target_id"])
            self.assertEqual(best["world_xy"], [tgt.x, tgt.y], t.task_id)
            ranked = sorted(key(o) for o in it); self.assertGreater(ranked[1] - ranked[0], 0.3)     # a unique referent, not a near-tie

class ScoringTests(unittest.TestCase):
    def test_sequence_scores_per_stage_and_needs_the_order(self):
        w, t = make_task("sequence", 1, 0, 3, False, 3)
        self.assertEqual(len(t.phases), 3)
        self.assertEqual(t.phases[-1].kind, "return_start")
        r = perfect(w, t); self.assertEqual(r["success"], 1); self.assertEqual(r["progress"], 1.0)
    def test_sequence_without_phase_log_fails_safely(self):
        w, t = make_task("sequence", 1, 0, 3, False, 1)
        self.assertFalse(check_success(w, t)["success"])
    def test_sequence_stage_two_only_counts_if_the_robot_stops_at_it(self):
        w, t = make_task("sequence", 2, 90, 3, False, 1)
        a, b = (next(o for o in w.objects if o.id == i) for i in t.expected["target_ids"])
        w.phase_log = [w.start_pose, (a.x + 1, a.y, 0)]; w.x, w.y = a.x + 1, a.y            # stopped next to the FIRST target twice
        res = check_success(w, t); self.assertFalse(res["success"]); self.assertEqual(res["detail"], "11" if abs(a.x + 1 - b.x) + abs(a.y - b.y) == 1 else "10")
    def test_clean_side_checker_uses_the_requested_side(self):
        w, t = make_task("clean_side", 1, 0, 1, False, 1)                                  # level 1 = right
        self.assertEqual(t.expected["side"], "right")
        w.cleaned = {c for c in side_region(w.w, w.h, w.start_pose, "left") if w.free(*c)}     # cleaned the WRONG side
        self.assertFalse(check_success(w, t)["success"])
        w.cleaned = {c for c in side_region(w.w, w.h, w.start_pose, "right") if w.free(*c)}
        self.assertTrue(check_success(w, t)["success"])

class OracleSolvabilityTests(unittest.TestCase):
    def test_noise_free_oracle_with_all_safeguards_solves_the_extended_suite(self):
        """Upper bound without error injection: a task the noise-free S5 cannot solve is a harness fault, not an agent failure."""
        ok = tot = 0
        for fam, s, yaw, k, m, lv in grid(["clean_side", "sequence", "relative"], 2, True):
            w, t = make_task(fam, s, yaw, k, m, lv); ok += perfect(w, t)["success"]; tot += 1
        self.assertGreaterEqual(ok / tot, 0.99, (ok, tot))

class CoreRegressionTests(unittest.TestCase):
    def test_shipped_oracle_results_are_reproduced(self):
        path = os.path.join(ROOT, "results_spatial_oracle", "trial_results.csv")
        with open(path, newline="") as f: shipped = {(r["task_id"], r["config"]): r for r in csv.DictReader(f)}
        checked = 0
        for fam, s, yaw, k, m, lv in grid(FAMILIES, 1):
            for name in S_LADDER:                     # the shipped core results cover S0-S5; C1-C4 were added in v0.10
                w, t = make_task(fam, s, yaw, k, m, lv)
                r = run_episode(w, t, PRESETS[name], OracleBackend(0.05, 0.30, abstain=False)); old = shipped[(t.task_id, name)]
                for col in ("success", "llm_calls", "env_tool_calls", "collisions", "final_x", "final_y", "final_yaw"):
                    self.assertEqual(str(r[col]), old[col], (t.task_id, name, col)); checked += 1
        self.assertGreater(checked, 2000)

if __name__ == "__main__": unittest.main()
