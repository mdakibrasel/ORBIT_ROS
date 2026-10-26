import csv, os, tempfile, unittest
from orbit_ros.humanoid import HumanoidSession, FakeNao


def session(extra=()):
    s = HumanoidSession(sim="fake", log_dir=False, backend=FakeNao(extra_obstacles=extra))
    s.load_scene("living_room")
    return s


class SafetyAndMetrics(unittest.TestCase):
    def test_clean_corners_and_directions(self):
        s = session()
        r = s.robot_call("clean_corners", {"room": "living_room"})
        self.assertEqual(r["status"], "ok", r["message"]); self.assertGreaterEqual(len(r["data"]["corners_done"]), 2)
        self.assertEqual(s.collisions, 0)
        for d in ("left", "right", "forward", "backward"):
            r = s.robot_call("clean_direction", {"direction": d, "steps": 1})
            self.assertIn(r["status"], ("ok", "vetoed"), r["message"])
        self.assertEqual(s.robot_call("clean_direction", {"direction": "up"})["status"], "rejected")

    def test_turn_and_walk_accuracy(self):
        s = session()
        s.robot_call("turn", {"direction": "left", "degrees": 60}); s.robot_call("walk_forward", {"steps": 1})
        r = s.robot_call("motion_accuracy")["data"]
        self.assertTrue(r["turning"]["pass"]); self.assertTrue(r["walking"]["pass"]); self.assertEqual(r["samples"], 2)

    def test_scan_obstacles(self):
        d = session().robot_call("scan_obstacles")["data"]
        self.assertEqual(len(d["rays"]), 5)

    def test_unmapped_obstacle_is_detected_and_avoided(self):
        s = session()
        x, y, yaw = s.nao.pose()[:3]
        s.nao.obstacles.append({"label": "box", "x": 2.8, "y": 3.1, "sx": 0.3, "sy": 0.3})      # on the way, not on the map
        r = s.robot_call("go_to", {"x": 1.5, "y": 3.1})
        self.assertEqual(r["status"], "ok", r["message"])
        self.assertGreaterEqual(s.replans, 1); self.assertTrue(s.detections)
        self.assertEqual(s.collisions, 0)
        self.assertTrue(s.robot_call("verify_task")["data"]["passed"])

    def test_brush_and_coverage(self):
        s = session()
        s.robot_call("clean", {"mode": "vacuum"}); s.robot_call("walk_forward", {"steps": 2})
        d = s.robot_call("verify_cleaning")["data"]
        self.assertTrue(d["brush_ok"]); self.assertGreater(d["coverage"]["house_covered"], 0)

    def test_clean_house_full_coverage_and_csv(self):
        s = session()
        r = s.robot_call("clean_house", {"pattern": "quick"})
        self.assertEqual(r["status"], "ok", r["message"])
        m = s.metrics()
        self.assertGreater(m["coverage_pct"], 20); self.assertGreater(m["distance_m"], 5); self.assertEqual(m["collisions"], 0)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "m.csv"); s.export_csv(p, model="scripted", run=1); s.export_csv(p, model="scripted", run=2)
            rows = list(csv.DictReader(open(p, encoding="utf-8")))
        self.assertEqual(len(rows), 2); self.assertEqual(rows[0]["model"], "scripted")

    def test_pause_stops_coverage(self):
        s = session()
        s.robot_call("clean", {"mode": "vacuum"}); s.robot_call("pause_cleaning")
        n = len(s.covered); s.robot_call("walk_forward", {"steps": 2})
        self.assertEqual(len(s.covered), n)


if __name__ == "__main__":
    unittest.main()
