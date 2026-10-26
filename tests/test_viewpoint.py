"""Code checks for the viewpoint math (NO Webots). The live camera modes are verified by @pytest.mark.live checks under Webots."""
import math, unittest
from orbit_ros import viewpoint as vp


def _apply(rot, v):
    """Rotate vector v by an axis-angle rot = [ax,ay,az,angle] (Rodrigues)."""
    ax, ay, az, ang = rot
    n = math.sqrt(ax*ax + ay*ay + az*az) or 1.0
    k = [ax/n, ay/n, az/n]
    c, s = math.cos(ang), math.sin(ang)
    dot = k[0]*v[0] + k[1]*v[1] + k[2]*v[2]
    cross = [k[1]*v[2]-k[2]*v[1], k[2]*v[0]-k[0]*v[2], k[0]*v[1]-k[1]*v[0]]
    return [v[i]*c + cross[i]*s + k[i]*dot*(1-c) for i in range(3)]


class TestLookAt(unittest.TestCase):
    def test_local_x_points_at_target(self):
        # the viewpoint looks along its local +x; after the rotation, +x must point from pos toward target
        pos, target = [0.0, 0.0, 3.0], [2.0, 1.0, 0.0]
        rot = vp.look_at_rotation(pos, target)
        fwd = _apply(rot, [1.0, 0.0, 0.0])
        want = [target[i] - pos[i] for i in range(3)]
        wn = math.sqrt(sum(w*w for w in want)); want = [w/wn for w in want]
        for a, b in zip(fwd, want):
            self.assertAlmostEqual(a, b, places=5)

    def test_identity_when_looking_along_x(self):
        rot = vp.look_at_rotation([0, 0, 0], [1, 0, 0])
        self.assertAlmostEqual(rot[3], 0.0, places=6)   # no rotation needed

    def test_follow_pose_looks_at_robot(self):
        pos, rot = vp.follow_pose((4.0, 3.0))
        fwd = _apply(rot, [1.0, 0.0, 0.0])
        want = [4.0 - pos[0], 3.0 - pos[1], 0.35 - pos[2]]
        wn = math.sqrt(sum(w*w for w in want)); want = [w/wn for w in want]
        for a, b in zip(fwd, want):
            self.assertAlmostEqual(a, b, places=5)
        self.assertGreater(pos[2], 1.0)   # camera is above the floor

    def test_overview_is_top_down(self):
        # the SAVED/reset view (stored in the world file, restored by reset()) is the straight-down overview
        self.assertEqual(vp.OVERVIEW["position"], [5.0, 4.0, 30.0])
        self.assertEqual(vp.OVERVIEW_SAVED["position"], [5.0, 4.0, 30.0])

    def test_adaptive_overview_is_angled_above_floor(self):
        # Phase 16: the adaptive overview() framing is an angled bird's-eye (not straight-down) so the NAO reads as a 3-D object
        pos, rot = vp.overview_pose()
        self.assertGreaterEqual(pos[2], vp.MIN_CAM_H)
        self.assertNotAlmostEqual(rot[3], 1.5708, places=2)   # differs from the straight-down saved view


if __name__ == "__main__":
    unittest.main()
