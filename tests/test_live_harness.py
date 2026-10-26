"""Code checks for the live harness (NO robot, NO LLM). Live behavior is covered by @pytest.mark.live tests that run under Webots."""
import csv, json, os, tempfile, unittest
from tools.live import webots_session as ws


class TestEvidencePackWriter(unittest.TestCase):
    def test_synthetic_pack_has_all_files(self):
        old = ws.RESULTS
        try:
            ws.RESULTS = tempfile.mkdtemp()
            poses = [(0.0, 4.8, 3.1, 180.0, 0), (0.5, 4.6, 3.1, 180.0, 0), (1.0, 4.4, 3.1, 180.0, 0)]
            d = ws._synthetic_pack("run1", "trialA", poses)
            for name in ("trace.jsonl", "true_poses.csv", "plan.json", "metrics.json", "verdict.json", "webots_console.txt"):
                self.assertTrue(os.path.exists(os.path.join(d, name)), name)
            rows = list(csv.reader(open(os.path.join(d, "true_poses.csv"), encoding="utf-8")))
            self.assertEqual(rows[0], ["t", "x", "y", "yaw", "fallen"])
            self.assertEqual(len(rows), 4)
        finally:
            ws.RESULTS = old

    def test_motion_count(self):
        self.assertEqual(ws._count_motion([(0, 0, 0, 0, 0), (1, 0.2, 0, 0, 0), (2, 0.2, 0.1, 0, 0)]), 0.3)
        self.assertEqual(ws._count_motion([(0, 1, 1, 0, 0)]), 0.0)   # a single sample = no observed motion

    def test_port_listening_is_false_on_a_dead_port(self):
        self.assertFalse(ws.port_listening(port=59999))


if __name__ == "__main__":
    unittest.main()
