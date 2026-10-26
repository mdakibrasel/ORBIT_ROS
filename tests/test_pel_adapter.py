"""ORBIT-PEL M8: the documented adapter contract. The minimal example adapter passes the conformance suite and gives the same PEL
episodes as World2D; a broken adapter (mirrored camera, wrong turn sign) is caught by the suite, so passing it means something."""
import io, contextlib, unittest
from examples.minimal_adapter import MinimalSim, main as example_main
from orbit_ros.contract_api import sim_adapter
from orbit_ros.conformance import run_suite

class MirroredCamera(MinimalSim):
    def recognition(self):
        return [dict(r, px=self.camera_width - r["px"]) for r in super().recognition()]

class WrongYaw(MinimalSim):
    def robot_pose(self):
        x, y, yaw = self.pose; return (x, y, (-yaw) % 360)          # clockwise yaw: violates REP-103

class AdapterContractTests(unittest.TestCase):
    def test_minimal_adapter_conforms_and_agrees(self):
        with contextlib.redirect_stdout(io.StringIO()) as out: rc = example_main()
        self.assertEqual(rc, 0, out.getvalue()); self.assertIn("11/11 checks passed", out.getvalue()); self.assertIn("8/8 episodes identical", out.getvalue())
    def test_suite_catches_a_mirrored_camera(self):
        failed = {r["check"] for r in run_suite(sim_adapter(MirroredCamera(), "grid", "mirrored")) if not r["passed"]}
        self.assertIn("camera_convention", failed)
    def test_suite_catches_a_wrong_yaw_convention(self):
        failed = {r["check"] for r in run_suite(sim_adapter(WrongYaw(), "grid", "wrong-yaw")) if not r["passed"]}
        self.assertTrue(failed & {"frame_units", "turn_convention"}, failed)

if __name__ == "__main__": unittest.main()
