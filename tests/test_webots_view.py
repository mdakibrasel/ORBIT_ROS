"""v0.15.1: 3D view of the Webots world (display only). No Webots here: the rotation maths and the world files are checked directly."""
import math, os, sys, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros.webots_adapter import look_at_rotation, room_view, WebotsSim

def rotate(r, v):
    x, y, z, a = r; n = math.sqrt(x * x + y * y + z * z); k = (x / n, y / n, z / n); c, s = math.cos(a), math.sin(a)
    d = sum(k[i] * v[i] for i in range(3)); cr = (k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0])
    return [v[i] * c + cr[i] * s + k[i] * d * (1 - c) for i in range(3)]

class ViewTests(unittest.TestCase):
    def test_look_at_points_at_target_without_roll(self):
        for pos, tgt in (((4, -4, 8), (4, 3, 0)), ((0, 0, 5), (3, 4, 0)), ((10, 10, 2), (0, 0, 0)), ((-3, 2, 1), (5, 2, 1)), ((5, 2, 1), (-3, 2, 1))):
            r = look_at_rotation(pos, tgt)
            d = [tgt[i] - pos[i] for i in range(3)]; n = math.sqrt(sum(x * x for x in d))
            for a, b in zip(rotate(r, (1, 0, 0)), d): self.assertAlmostEqual(a, b / n, places=6)
            self.assertAlmostEqual(rotate(r, (0, 1, 0))[2], 0.0, places=6)            # horizon stays level
            self.assertGreater(rotate(r, (0, 0, 1))[2], 0.0)                          # up stays up

    def test_room_view_frames_room_from_south(self):
        for w, h in ((9, 7), (6, 4), (17, 13)):
            pos, _ = room_view(w, h)
            self.assertAlmostEqual(pos[0], (w - 1) / 2); self.assertLess(pos[1], 0); self.assertGreater(pos[2], max(w, h) / 2)

    def test_frame_view_sets_fields_and_tolerates_missing_viewpoint(self):
        class F:
            def __init__(self): self.v = None
            def setSFVec3f(self, v): self.v = v
            def setSFRotation(self, v): self.v = v
        class VP:
            def __init__(self): self.f = {"position": F(), "orientation": F()}
            def getField(self, n): return self.f[n]
        class Sup:
            def __init__(self, vp): self.vp = vp
            def getFromDef(self, n): return self.vp if n == "ORBIT_VIEW" else None
        sim = object.__new__(WebotsSim); vp = VP(); sim.sup = Sup(vp)
        sim._frame_view(6, 4)
        pos, rot = room_view(6, 4)
        self.assertEqual(vp.f["position"].v, list(pos)); self.assertEqual(vp.f["orientation"].v, rot)
        sim.sup = Sup(None); sim._frame_view(6, 4)                                   # older world without ORBIT_VIEW: nothing happens

if __name__ == "__main__": unittest.main()
