"""Adaptive Webots R2025a viewpoint for the ORBIT NAO house (Phase 12, extended Phase 16). Display only.

A viewpoint change NEVER calls the LLM, moves the robot, resets physics or changes the robot pose. It only writes the Viewpoint node's
fields through the EXISTING Supervisor (WebotsNao.sup) — no second Supervisor, no competing controller loop. Camera following uses
Webots' built-in Viewpoint `follow`/`followType`/`followSmoothness`; target framing reuses a self-contained look-at rotation (copied here
so the NAO runtime never imports the TurtleBot/World2D legacy `webots_adapter`).

Modes: overview | follow | closeup | target | human | demo | reset, plus zoom (in | out | numeric factor). The module is standard-library
only and degrades gracefully: if the installed Webots build lacks a field it falls back to setting position/orientation directly, and
records what it used.

Design notes (Phase 16):
  * overview is an angled bird's-eye (not straight-down) so the ~0.57 m NAO is recognisable against the ~10x8 m house (walls ~2.4 m).
  * closeup frames the NAO tightly from a readable three-quarter angle.
  * human frames the animated actor (DEF ORBIT_HUMAN) the same way, if that node exists in the loaded world.
  * zoom DOLLIES the camera along its view axis (moves closer/further) rather than changing FOV, so geometry/perspective stay honest;
    the camera is clamped to a minimum height and a sane distance range so a zoom can never bury the camera in the floor.
"""
from __future__ import annotations
import math

HOUSE_W, HOUSE_H = 10.0, 8.0
HOUSE_CENTER = (HOUSE_W / 2.0, HOUSE_H / 2.0)       # (5, 4)
WALL_H = 2.4
HUMAN_DEF = "ORBIT_HUMAN"

# The saved overview kept for reset()/restore compatibility (matches DEF ORBIT_VIEW in orbit_house.wbt): straight-down over the centre.
OVERVIEW_SAVED = {"position": [5.0, 4.0, 30.0], "orientation": [0.0, 1.0, 0.0, 1.5708]}
OVERVIEW = OVERVIEW_SAVED               # backward-compatible alias: the saved, restorable top-down view (reset() restores it)
FOLLOW_TYPES = ("None", "Tracking Shot", "Mounted Shot", "Pan and Tilt Shot")

# Phase-16 framing constants (metres). Camera is never placed below MIN_CAM_H.
MIN_CAM_H = 2.8
ZOOM_DIST_MIN, ZOOM_DIST_MAX = 1.5, 40.0            # how close / far a dolly may bring the camera to its look-at point
ZOOM_STEP = 1.35                                    # multiplicative factor for a single zoom in/out


def look_at_rotation(pos, target):
    """Viewpoint axis-angle that looks from pos at target. Webots viewpoints look along local +x, +z up; R = Rz(yaw)*Ry(pitch),
    pitch>0 tilts down. Copied (pure math, no deps) from the display-only helper so the NAO runtime stays free of legacy code."""
    dx, dy, dz = (target[i] - pos[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]
    ang = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if ang < 1e-9: return [0.0, 0.0, 1.0, 0.0]
    if abs(ang - math.pi) < 1e-6:
        ax = [math.sqrt(max(0.0, (R[i][i] + 1) / 2)) for i in range(3)]
        if ax[0] > 1e-6: ax[1], ax[2] = math.copysign(ax[1], R[0][1]), math.copysign(ax[2], R[0][2])
        elif ax[1] > 1e-6: ax[2] = math.copysign(ax[2], R[1][2])
        return [*ax, ang]
    k = 2 * math.sin(ang)
    return [(R[2][1] - R[1][2]) / k, (R[0][2] - R[2][0]) / k, (R[1][0] - R[0][1]) / k, ang]


def _framing_pose(look_xy, look_z, dist, height, bearing_deg):
    """A three-quarter camera that looks at (look_xy, look_z) from `dist` away on compass `bearing_deg`, at `height` (clamped)."""
    b = math.radians(bearing_deg)
    height = max(MIN_CAM_H if height >= MIN_CAM_H else height, height)   # keep callers' intent but never below MIN later
    pos = [look_xy[0] + dist * math.cos(b), look_xy[1] + dist * math.sin(b), max(MIN_CAM_H, height)]
    return pos, look_at_rotation(pos, [look_xy[0], look_xy[1], look_z])


def follow_pose(robot_xy, bearing_deg=215.0, dist=3.2, height=2.3):
    """A three-quarter chase position behind/above the robot, looking at it. Used for target/manual framing; built-in `follow`
    handles ordinary following. bearing is the camera's compass offset so the robot is seen from a readable angle."""
    b = math.radians(bearing_deg)
    pos = [robot_xy[0] + dist * math.cos(b), robot_xy[1] + dist * math.sin(b), height]
    return pos, look_at_rotation(pos, [robot_xy[0], robot_xy[1], 0.35])


def overview_pose():
    """Angled bird's-eye of the whole house from the south-west, high enough to see all four rooms but tilted so furniture and the
    NAO read as 3-D objects (not a flat floor-plan). Computed from the real 10x8 m footprint."""
    cx, cy = HOUSE_CENTER
    pos = [cx - 6.5, cy - 7.5, 11.0]                 # south-west and above
    return pos, look_at_rotation(pos, [cx, cy, 0.4])


def smoothing_alpha(dt, tau):
    """First-order smoothing coefficient alpha = 1 - exp(-dt/tau) for an exponential approach (used when a smoothed glide is wanted)."""
    if tau <= 0: return 1.0
    return 1.0 - math.exp(-max(0.0, dt) / tau)


class Viewpoint:
    """Wraps the live Viewpoint node from an already-connected WebotsNao (reuses its Supervisor; creates nothing)."""
    def __init__(self, nao):
        self.nao = nao
        self.sup = getattr(nao, "sup", None)
        self.mode = "overview"
        self.last = {}
        self._look = None          # last look-at point (x,y,z); zoom dollies toward THIS, not always the robot
        self._vp = None
        if self.sup is not None:
            self._vp = self.sup.getFromDef("ORBIT_VIEW") or (self.sup.getFromDef and self.sup.getSelf and None)
            if self._vp is None:
                # fall back to the world's Viewpoint via the standard getter if DEF lookup failed
                try: self._vp = self.sup.getViewpoint()      # not in all builds; guarded
                except Exception: self._vp = None

    def available(self):
        return self._vp is not None

    def _set_vec(self, field, vec):
        f = self._vp.getField(field)
        if f is not None: f.setSFVec3f([float(v) for v in vec]); return True
        return False

    def _set_rot(self, field, rot):
        f = self._vp.getField(field)
        if f is not None: f.setSFRotation([float(v) for v in rot]); return True
        return False

    def _get_vec(self, field):
        f = self._vp.getField(field)
        if f is not None:
            try: return list(f.getSFVec3f())
            except Exception: return None
        return None

    def _set_follow(self, name, ftype="Tracking Shot", smoothness=0.4):
        """Use the built-in Viewpoint following if the fields exist. Returns the fields actually set."""
        used = {}
        f = self._vp.getField("follow")
        if f is not None:
            try: f.setSFString(name); used["follow"] = name
            except Exception: pass
        ft = self._vp.getField("followType")
        if ft is not None:
            try: ft.setSFString(ftype); used["followType"] = ftype
            except Exception: pass
        fs = self._vp.getField("followSmoothness")
        if fs is not None:
            try: fs.setSFFloat(float(smoothness)); used["followSmoothness"] = smoothness
            except Exception: pass
        return used

    def _clear_follow(self):
        f = self._vp.getField("follow")
        if f is not None:
            try: f.setSFString("")
            except Exception: pass

    def _aim(self, pos, look):
        """Set the camera position + a look-at orientation AND remember the look-at point so a subsequent zoom dollies toward it."""
        self._set_vec("position", pos)
        self._set_rot("orientation", look_at_rotation(pos, look))
        self._look = [float(look[0]), float(look[1]), float(look[2])]

    # ---- modes ----
    def overview(self):
        """Angled bird's-eye of the whole house (not straight-down) so the NAO and furniture are recognisable 3-D objects."""
        if not self.available(): return self._record("overview", ok=False, reason="no viewpoint node")
        self._clear_follow()
        pos, rot = overview_pose()
        self._set_vec("position", pos); self._set_rot("orientation", rot)
        self._look = [HOUSE_CENTER[0], HOUSE_CENTER[1], 0.4]
        return self._record("overview", ok=True, position=pos)

    def reset(self):
        """Restore the saved straight-down overview exactly as stored in the world file (the restorable default)."""
        if not self.available(): return self._record("reset", ok=False, reason="no viewpoint node")
        self._clear_follow()
        self._set_vec("position", OVERVIEW_SAVED["position"]); self._set_rot("orientation", OVERVIEW_SAVED["orientation"])
        return self._record("reset", ok=True, position=list(OVERVIEW_SAVED["position"]))

    def follow(self, ftype="Tracking Shot", smoothness=0.4):
        if not self.available(): return self._record("follow", ok=False, reason="no viewpoint node")
        name = self._robot_name()
        used = self._set_follow(name, ftype, smoothness)
        if "follow" not in used:                             # built-in follow unavailable: place a chase cam once (static fallback)
            p = self._robot_xy()
            if p: pos, rot = follow_pose(p); self._set_vec("position", pos); self._set_rot("orientation", rot)
            return self._record("follow", ok=True, builtin=False, fallback="static chase cam", **used)
        return self._record("follow", ok=True, builtin=True, **used)

    def _interior_camera_pos(self, subject_xy, dist, min_h, margin=0.6):
        """Place a camera `dist` metres from the subject, on the side TOWARD the house interior (the house centre), so the shot is never
        taken from outside a wall. The result is clamped to stay inside the house footprint (with `margin` from the walls) and at height
        >= `min_h`. Returns a 3-vector position. This makes closeup/human framing robust when the subject stands near a wall or corner."""
        sx, sy = subject_xy
        cx, cy = HOUSE_CENTER
        # direction from the subject toward the house centre (so the camera looks back at the subject from inside the room)
        dx, dy = cx - sx, cy - sy
        n = math.hypot(dx, dy)
        if n < 1e-6:                                     # subject is at the centre: fall back to a fixed SW three-quarter bearing
            dx, dy, n = math.cos(math.radians(215)), math.sin(math.radians(215)), 1.0
        ux, uy = dx / n, dy / n
        px, py = sx + ux * dist, sy + uy * dist
        # clamp inside the house footprint so the camera never ends up behind/outside a wall
        px = min(HOUSE_W - margin, max(margin, px))
        py = min(HOUSE_H - margin, max(margin, py))
        return [px, py, max(MIN_CAM_H, min_h)]

    def closeup(self):
        """Tight three-quarter shot of the NAO: close enough that the robot fills most of the frame, framed from inside the room so a
        corner/wall never blocks the shot. Static."""
        if not self.available(): return self._record("closeup", ok=False, reason="no viewpoint node")
        self._clear_follow()
        p = self._robot_xy()
        if not p: return self._record("closeup", ok=False, reason="no robot pose")
        # NAO is ~0.57 m tall; look at its mid-torso. Camera stands close, toward the house interior, at >= MIN_CAM_H.
        pos = self._interior_camera_pos(p, dist=1.8, min_h=MIN_CAM_H)
        self._aim(pos, [p[0], p[1], 0.3])
        return self._record("closeup", ok=True, position=pos, target=[round(p[0], 3), round(p[1], 3)])

    def human(self):
        """Frame the animated human actor (DEF ORBIT_HUMAN) so its FACE/front is visible, from inside the room (never behind a wall).
        Static three-quarter shot. ok:False if no human node in the world."""
        if not self.available(): return self._record("human", ok=False, reason="no viewpoint node")
        hp = self._human_xy()
        if hp is None: return self._record("human", ok=False, reason=f"no {HUMAN_DEF} node in the loaded world")
        self._clear_follow()
        yaw = self._human_yaw()
        # Prefer a frontal three-quarter view so eyes/nose/mouth are seen; fall back to the interior-facing placement if yaw unknown.
        if yaw is not None:
            pos = self._frontal_camera_pos(hp, yaw, dist=2.6, min_h=2.4)
            framing = "frontal"
        else:
            pos = self._interior_camera_pos(hp, dist=2.6, min_h=2.4)
            framing = "interior"
        # look at head/chest height (human origin z ~1.27; head ~1.7); aim a bit high so the face is centred, not the floor
        self._aim(pos, [hp[0], hp[1], 1.5])
        return self._record("human", ok=True, position=pos, target=[round(hp[0], 3), round(hp[1], 3)], framing=framing,
                            yaw_deg=(round(math.degrees(yaw), 1) if yaw is not None else None))

    def target(self, target_xy):
        """Frame the NAO and a target together (both visible where practical). Static, computed from verified positions."""
        if not self.available(): return self._record("target", ok=False, reason="no viewpoint node")
        self._clear_follow()
        p = self._robot_xy()
        if not p: return self._record("target", ok=False, reason="no robot pose")
        mid = [(p[0] + target_xy[0]) / 2, (p[1] + target_xy[1]) / 2, 0.3]
        span = math.hypot(target_xy[0] - p[0], target_xy[1] - p[1])
        dist = max(2.5, span + 2.0)
        pos = [mid[0] - dist * 0.55, mid[1] - dist * 0.55, max(MIN_CAM_H, dist * 0.8)]
        self._aim(pos, mid)
        return self._record("target", ok=True, target=list(target_xy))

    def demo(self):
        """Demo framing = built-in follow (so the walking robot stays centred) + the HUD labels are driven by the trial/executor."""
        r = self.follow(ftype="Tracking Shot", smoothness=0.5)
        r["mode"] = "demo"; self.mode = "demo"; return r

    def zoom(self, amount="in"):
        """Dolly the camera along its view axis toward / away from the current look-at point. `amount`:
            "in"  -> move ZOOM_STEP closer   "out" -> move ZOOM_STEP further
            a positive number -> multiply the current distance by that factor (e.g. 0.5 = twice as close, 2.0 = twice as far).
        Distance is clamped to [ZOOM_DIST_MIN, ZOOM_DIST_MAX] and the camera height to >= MIN_CAM_H. Does NOT change FOV."""
        if not self.available(): return self._record("zoom", ok=False, reason="no viewpoint node")
        # Disable built-in follow: dollying only makes sense on a static framing.
        self._clear_follow()
        pos = self._get_vec("position")
        if pos is None: return self._record("zoom", ok=False, reason="viewpoint has no position field")
        look = self._current_look()
        if look is None: return self._record("zoom", ok=False, reason="no look-at reference (no robot pose)")
        vx, vy, vz = (pos[i] - look[i] for i in range(3))
        dist = math.sqrt(vx * vx + vy * vy + vz * vz)
        if dist < 1e-6:
            return self._record("zoom", ok=False, reason="camera is at the look-at point")
        factor = self._zoom_factor(amount)
        if factor is None:
            return self._record("zoom", ok=False, reason=f"invalid zoom amount {amount!r}: use 'in', 'out' or a positive number")
        new_dist = max(ZOOM_DIST_MIN, min(ZOOM_DIST_MAX, dist * factor))
        ux, uy, uz = vx / dist, vy / dist, vz / dist
        new_pos = [look[i] + (ux, uy, uz)[i] * new_dist for i in range(3)]
        if new_pos[2] < MIN_CAM_H: new_pos[2] = MIN_CAM_H
        self._set_vec("position", new_pos); self._set_rot("orientation", look_at_rotation(new_pos, look))
        return self._record("zoom", ok=True, amount=amount, factor=round(factor, 4),
                            distance_before=round(dist, 3), distance_after=round(new_dist, 3), position=new_pos)

    def _zoom_factor(self, amount):
        if isinstance(amount, str):
            a = amount.strip().lower()
            if a in ("in", "closer", "+"): return 1.0 / ZOOM_STEP
            if a in ("out", "further", "farther", "-"): return ZOOM_STEP
            try: amount = float(a)
            except ValueError: return None
        try:
            f = float(amount)
        except (TypeError, ValueError):
            return None
        return f if f > 0 else None

    def _current_look(self):
        """Look-at point for a dolly: the point the CURRENT framing is aimed at (remembered by `_aim`), so zooming a human/target shot
        stays on that subject. Falls back to the robot, then the human, then the house centre."""
        if self._look is not None: return list(self._look)
        p = self._robot_xy()
        if p is not None: return [p[0], p[1], 0.3]
        hp = self._human_xy()
        if hp is not None: return [hp[0], hp[1], 0.9]
        return [HOUSE_CENTER[0], HOUSE_CENTER[1], 0.4]

    # ---- helpers ----
    def _robot_name(self):
        try:
            nf = self.nao.node.getField("name")
            return nf.getSFString() if nf else "orbit_nao"
        except Exception:
            return "orbit_nao"

    def _robot_xy(self):
        try:
            p = self.nao.node.getPosition(); return (p[0], p[1])
        except Exception:
            return None

    def _human_xy(self):
        try:
            if self.sup is None: return None
            node = self.sup.getFromDef(HUMAN_DEF)
            if node is None: return None
            p = node.getPosition(); return (p[0], p[1])
        except Exception:
            return None

    def _human_yaw(self):
        """The human's facing yaw in radians (rotation about +z), or None. Used to frame the FACE (camera in front of the figure)."""
        try:
            if self.sup is None: return None
            node = self.sup.getFromDef(HUMAN_DEF)
            if node is None: return None
            # read the orientation matrix and recover yaw of the local +x axis in the world XY plane
            o = node.getOrientation()           # 3x3 row-major; column 0 is the world direction of local +x
            if o and len(o) >= 7:
                return math.atan2(o[3], o[0])   # atan2(R[1][0], R[0][0])
        except Exception:
            return None
        return None

    def _frontal_camera_pos(self, subject_xy, yaw, dist, min_h, margin=0.6, offset_deg=30.0):
        """Place the camera roughly IN FRONT of a subject that faces `yaw` (so the face/front is seen), offset by `offset_deg` for a
        three-quarter angle, then clamp inside the house footprint. Falls back to the interior placement if the frontal spot would be
        pushed outside the house by clamping (which would otherwise hide the face again)."""
        sx, sy = subject_xy
        b = yaw + math.radians(offset_deg)              # a bit to the subject's left-front for a three-quarter view
        px, py = sx + dist * math.cos(b), sy + dist * math.sin(b)
        cpx = min(HOUSE_W - margin, max(margin, px))
        cpy = min(HOUSE_H - margin, max(margin, py))
        # if clamping moved us a lot, the frontal spot was outside a wall; try the opposite three-quarter side
        if math.hypot(cpx - px, cpy - py) > 0.25:
            b2 = yaw - math.radians(offset_deg)
            px2, py2 = sx + dist * math.cos(b2), sy + dist * math.sin(b2)
            cpx2 = min(HOUSE_W - margin, max(margin, px2)); cpy2 = min(HOUSE_H - margin, max(margin, py2))
            if math.hypot(cpx2 - px2, cpy2 - py2) <= math.hypot(cpx - px, cpy - py):
                cpx, cpy = cpx2, cpy2
        return [cpx, cpy, max(MIN_CAM_H, min_h)]

    def _record(self, mode, ok=True, **kw):
        self.mode = mode if ok else self.mode
        self.last = {"mode": mode, "ok": ok, **kw}
        return self.last

    def render(self, frames=8):
        """Advance the EXISTING controller a few steps so a viewpoint change is rendered before a screenshot. This uses the one
        Supervisor we already have (no second controller, no second loop); stepping with no motor command does not move the robot."""
        if self.sup is None: return False
        try:
            for _ in range(max(1, frames)): self.nao.step()
            return True
        except Exception:
            return False
