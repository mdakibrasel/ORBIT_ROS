"""Code checks (no robot, no LLM, no Webots) for the Phase-16 adaptive display camera.

Covers: viewpoint pure geometry (overview angled not straight-down, look-at, zoom clamp/dolly, smoothing), the camera() session
method side-effect contract (no robot motion, no LLM, no memory change, safe no-op without a viewpoint), the camera MCP tool
registration, and the /view chat-command parser. None of this touches a robot or a provider, so it is a pure code check.
"""
import math, unittest

from orbit_ros import viewpoint as vpmod
from orbit_ros.viewpoint import (Viewpoint, overview_pose, look_at_rotation, smoothing_alpha,
                                  MIN_CAM_H, ZOOM_DIST_MIN, ZOOM_DIST_MAX, ZOOM_STEP, OVERVIEW_SAVED)


# ------------------------------------------------------------------ fakes (local; no Webots)
class _Field:
    def __init__(self, kind, val): self.kind, self.val = kind, val
    def getSFVec3f(self): return list(self.val)
    def setSFVec3f(self, v): self.val = list(v)
    def getSFRotation(self): return list(self.val)
    def setSFRotation(self, v): self.val = list(v)
    def setSFString(self, v): self.val = v
    def setSFFloat(self, v): self.val = float(v)


class _Node:
    def __init__(self, pos, fields=None, yaw=0.0):
        self._pos = list(pos)
        self._fields = fields or {}
        self._yaw = yaw
    def getPosition(self): return list(self._pos)
    def getField(self, name): return self._fields.get(name)
    def getOrientation(self):
        import math as _m
        c, s = _m.cos(self._yaw), _m.sin(self._yaw)
        return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]   # Rz(yaw), row-major 3x3


class _Sup:
    """Minimal Supervisor stand-in: a Viewpoint node (with position/orientation/follow fields) and optional human node."""
    def __init__(self, with_human=False):
        self.vp = _Node([5, 4, 30], {
            "position": _Field("vec", [5.0, 4.0, 30.0]),
            "orientation": _Field("rot", [0.0, 1.0, 0.0, 1.5708]),
            "follow": _Field("str", ""), "followType": _Field("str", "None"), "followSmoothness": _Field("flt", 0.0)})
        self._human = _Node([3.0, 6.0, 1.27]) if with_human else None
    def getFromDef(self, name):
        if name == "ORBIT_VIEW": return self.vp
        if name == "ORBIT_HUMAN": return self._human
        return None
    def getSelf(self): return None


class _Nao:
    """Stand-in WebotsNao: a Supervisor, a robot node, a counting step(). step() must never move the robot."""
    name = "webots"
    def __init__(self, with_human=False, xy=(4.8, 3.1)):
        self.sup = _Sup(with_human=with_human)
        self.node = _Node([xy[0], xy[1], 0.33], {"name": _Field("str", "orbit_nao")})
        self.steps = 0
    def step(self, n=1): self.steps += n
    def pose(self): p = self.node.getPosition(); return p[0], p[1], 0.0, False


# ------------------------------------------------------------------ pure geometry
class TestViewpointGeometry(unittest.TestCase):
    def test_overview_is_angled_not_straight_down_and_above_min_height(self):
        pos, rot = overview_pose()
        self.assertGreaterEqual(pos[2], MIN_CAM_H)
        # straight-down would be orientation [0,1,0,~1.5708]; angled overview must differ
        self.assertNotAlmostEqual(rot[3], 1.5708, places=2)

    def test_look_at_straight_down_matches_saved_overview(self):
        r = look_at_rotation([5, 4, 10], [5, 4, 0])
        self.assertAlmostEqual(r[1], 1.0, places=3)
        self.assertAlmostEqual(r[3], math.pi / 2, places=3)

    def test_smoothing_alpha(self):
        self.assertEqual(smoothing_alpha(0.1, 0.0), 1.0)         # tau=0 -> instant
        self.assertAlmostEqual(smoothing_alpha(0.3, 0.3), 1 - math.e ** -1, places=6)
        self.assertTrue(0.0 < smoothing_alpha(0.064, 0.3) < 1.0)


# ------------------------------------------------------------------ Viewpoint modes against the fake supervisor
class TestViewpointModes(unittest.TestCase):
    def test_no_viewpoint_node_is_graceful(self):
        class Bare:  # no sup
            sup = None
        vp = Viewpoint(Bare())
        self.assertFalse(vp.available())
        self.assertFalse(vp.overview()["ok"])

    def test_overview_closeup_target_set_fields(self):
        vp = Viewpoint(_Nao())
        self.assertTrue(vp.available())
        self.assertTrue(vp.overview()["ok"])
        self.assertTrue(vp.closeup()["ok"])
        r = vp.target([2.0, 6.0]); self.assertTrue(r["ok"])
        # camera never placed below the floor minimum
        self.assertGreaterEqual(vp._get_vec("position")[2], MIN_CAM_H - 1e-9)

    def test_reset_restores_saved_overview(self):
        vp = Viewpoint(_Nao())
        vp.closeup()
        r = vp.reset()
        self.assertTrue(r["ok"])
        self.assertEqual(vp._get_vec("position"), list(OVERVIEW_SAVED["position"]))

    def test_human_mode_needs_human_node(self):
        self.assertFalse(Viewpoint(_Nao(with_human=False)).human()["ok"])
        self.assertTrue(Viewpoint(_Nao(with_human=True)).human()["ok"])

    def test_zoom_dollies_and_clamps(self):
        vp = Viewpoint(_Nao())
        vp.closeup()
        before = vp._get_vec("position")
        d0 = math.dist(before, [4.8, 3.1, 0.3])
        vp.zoom("in")
        after = vp._get_vec("position")
        d1 = math.dist(after, [4.8, 3.1, 0.3])
        self.assertLess(d1, d0)                                   # 'in' brings the camera closer
        # repeated zoom-in clamps at ZOOM_DIST_MIN, never past the look-at point
        for _ in range(30): vp.zoom("in")
        p = vp._get_vec("position")
        self.assertGreaterEqual(math.dist(p, [4.8, 3.1, 0.3]), ZOOM_DIST_MIN - 1e-6)
        self.assertGreaterEqual(p[2], MIN_CAM_H - 1e-6)

    def test_zoom_out_and_numeric_factor(self):
        vp = Viewpoint(_Nao()); vp.closeup()
        self.assertTrue(vp.zoom("out")["ok"])
        self.assertTrue(vp.zoom(2.0)["ok"])
        self.assertFalse(vp.zoom("sideways")["ok"])
        self.assertFalse(vp.zoom(-1.0)["ok"])

    def test_zoom_after_human_stays_on_the_human_not_the_robot(self):
        # regression: zooming a human framing must dolly toward the HUMAN, not snap back to the robot
        nao = _Nao(with_human=True)
        nao.node = _Node([4.8, 3.1, 0.33], {"name": _Field("str", "orbit_nao")})   # robot far from the human
        nao.sup._human = _Node([2.2, 1.2, 1.27], yaw=0.0)
        vp = Viewpoint(nao)
        vp.human()
        look = vp._current_look()
        self.assertAlmostEqual(look[0], 2.2, places=3)      # look-at is the human, not the robot at 4.8
        self.assertAlmostEqual(look[1], 1.2, places=3)
        vp.zoom(0.6)
        px, py, _ = vp._get_vec("position")
        # after zooming in on the human, the camera must be nearer the human than to the robot
        self.assertLess(math.dist((px, py), (2.2, 1.2)), math.dist((px, py), (4.8, 3.1)))

    def test_render_steps_existing_controller_without_moving_robot(self):
        nao = _Nao(); vp = Viewpoint(nao)
        p0 = nao.node.getPosition()
        self.assertTrue(vp.render(5))
        self.assertEqual(nao.steps, 5)                            # stepped the one controller
        self.assertEqual(nao.node.getPosition(), p0)              # robot did not move

    def test_closeup_camera_stays_inside_house_for_corner_robot(self):
        # regression: a subject near a wall/corner must still be framed from INSIDE the house (not from behind a wall)
        from orbit_ros.viewpoint import HOUSE_W, HOUSE_H
        nao = _Nao(xy=(0.3, 0.3))                                 # SW corner, hard against two walls
        vp = Viewpoint(nao)
        self.assertTrue(vp.closeup()["ok"])
        px, py, pz = vp._get_vec("position")
        self.assertTrue(0.0 < px < HOUSE_W, f"camera x {px} outside house")
        self.assertTrue(0.0 < py < HOUSE_H, f"camera y {py} outside house")
        # camera must be on the interior side of the subject (toward the centre), not further into the corner
        self.assertGreater(px, 0.3); self.assertGreater(py, 0.3)

    def test_human_camera_stays_inside_house_for_corner_human(self):
        from orbit_ros.viewpoint import HOUSE_W, HOUSE_H
        nao = _Nao(with_human=True)
        nao.sup._human = _Node([1.0, 1.0, 1.27])                  # human near the SW corner
        vp = Viewpoint(nao)
        self.assertTrue(vp.human()["ok"])
        px, py, _ = vp._get_vec("position")
        self.assertTrue(0.0 < px < HOUSE_W and 0.0 < py < HOUSE_H, f"human camera ({px},{py}) outside house")

    def test_human_camera_is_in_front_of_the_face(self):
        # human faces +x (yaw 0); to see the face the camera must be on the +x (east) side, looking back west
        nao = _Nao(with_human=True)
        nao.sup._human = _Node([2.2, 1.2, 1.27], yaw=0.0)
        vp = Viewpoint(nao)
        r = vp.human()
        self.assertTrue(r["ok"])
        self.assertEqual(r["framing"], "frontal")
        px, _, _ = vp._get_vec("position")
        self.assertGreater(px, 2.2, "camera should be in front (east) of a human facing +x, to show the face")


# ------------------------------------------------------------------ camera() session contract
class _Mem:
    def __init__(self): self.writes = 0
    def __getattr__(self, name):
        def rec(*a, **k): self.writes += 1
        return rec


class TestCameraSession(unittest.TestCase):
    def _session(self, with_human=False):
        from orbit_ros.humanoid import HumanoidSession
        s = HumanoidSession(sim="webots", log_dir=False, backend=_Nao(with_human=with_human), memory=_Mem())
        s.scene = "living_room"
        return s

    def test_camera_display_only_no_motion_no_memory(self):
        s = self._session()
        p0 = s.nao.node.getPosition()
        mem_writes0 = s.memory.writes
        r = s.camera(mode="closeup")
        self.assertEqual(r["status"], "ok")
        self.assertTrue(r["available"])
        self.assertEqual(r["mode"], "closeup")
        self.assertEqual(s.nao.node.getPosition(), p0)            # robot never moved
        self.assertEqual(s.memory.writes, mem_writes0)            # memory never written

    def test_camera_all_modes(self):
        s = self._session(with_human=True)
        for m in ("overview", "follow", "closeup", "target", "human", "demo", "reset"):
            args = {"mode": m}
            if m == "target": args["target"] = [2.0, 6.0]
            r = s.camera(**args)
            self.assertEqual(r["status"], "ok", f"{m}: {r}")

    def test_camera_target_requires_point(self):
        self.assertEqual(self._session().camera(mode="target")["status"], "error")

    def test_camera_zoom(self):
        s = self._session()
        s.camera(mode="closeup")
        self.assertEqual(s.camera(mode="zoom", zoom="in")["status"], "ok")
        self.assertEqual(s.camera(mode="zoom", zoom=0.5)["status"], "ok")
        self.assertEqual(s.camera(mode="zoom")["status"], "error")   # zoom-only with no amount

    def test_camera_unknown_mode(self):
        self.assertEqual(self._session().camera(mode="orbit")["status"], "error")

    def test_fake_session_is_safe_noop(self):
        from orbit_ros.humanoid import HumanoidSession
        s = HumanoidSession(sim="fake", log_dir=False)
        r = s.camera(mode="overview")
        self.assertEqual(r["status"], "ok")
        self.assertFalse(r["available"])


# ------------------------------------------------------------------ MCP tool registration + dispatch
class TestCameraMcpTool(unittest.TestCase):
    def test_camera_tool_registered_and_dispatched(self):
        from orbit_ros.mcp_server import H_SESSION_TOOLS, HumanoidMcpServer
        from orbit_ros.humanoid import HumanoidSession
        self.assertIn("camera", H_SESSION_TOOLS)
        s = HumanoidSession(sim="webots", log_dir=False, backend=_Nao())
        s.scene = "living_room"
        srv = HumanoidMcpServer(s, "orbit")
        names = [t["name"] for t in srv.tool_defs()]
        self.assertIn("camera", names)
        payload, is_err = srv.call_tool("camera", {"mode": "overview"})
        self.assertFalse(is_err)
        self.assertEqual(payload["status"], "ok")
        # camera is read-only (no robot motion) in its MCP annotation
        cam_def = next(t for t in srv.tool_defs() if t["name"] == "camera")
        self.assertTrue(cam_def["annotations"]["readOnlyHint"])


# ------------------------------------------------------------------ /view chat command parser
class TestViewCommandParser(unittest.TestCase):
    def setUp(self):
        import orbit_chat
        self.p = orbit_chat.parse_view_command

    def test_not_a_view_command(self):
        self.assertIsNone(self.p("go to the kitchen"))

    def test_plain_and_modes(self):
        self.assertEqual(self.p("/view")[0], {"mode": "overview"})
        for m in ("overview", "follow", "closeup", "target", "human", "demo", "reset"):
            if m == "target":
                self.assertEqual(self.p(f"/view {m} 1 2")[0], {"mode": "target", "target": [1.0, 2.0]})
            else:
                self.assertEqual(self.p(f"/view {m}")[0], {"mode": m})

    def test_zoom_only(self):
        self.assertEqual(self.p("/view zoom in")[0], {"mode": "zoom", "zoom": "in"})
        self.assertEqual(self.p("/view zoom 0.5")[0], {"mode": "zoom", "zoom": 0.5})

    def test_mode_plus_zoom(self):
        self.assertEqual(self.p("/view closeup zoom in")[0], {"mode": "closeup", "zoom": "in"})

    def test_errors(self):
        self.assertIsNone(self.p("/view bogus")[0])
        self.assertIsNone(self.p("/view zoom")[0])
        self.assertIsNone(self.p("/view target 3")[0])
        self.assertIsNone(self.p("/view zoom nope")[0])


if __name__ == "__main__":
    unittest.main()
