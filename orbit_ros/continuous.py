"""Closed-loop differential-drive motion for the continuous Webots mode (roadmap stage C2 groundwork).

`DiffDrive` is written once against a tiny `Plant` interface, so the same controller runs on `FakePlant` (pure Python
kinematics, used by the unit tests) and on the Webots TurtleBot3 Burger (`WebotsPlant` in webots_adapter.py).

* Distance and heading feedback come from wheel-encoder odometry, as in the roadmap's tool mapping.
* A cell step is gated by the lidar: if the front sector sees something closer than `gate` (an obstacle face in the next cell
  is 0.7 m away, a free next cell with an obstacle behind it is 1.7 m away) the move is reported as blocked *before* leaving the
  current cell. Room walls are real Solids, so the same gate stops the robot at the room boundary.
* Stall (no progress while driving) and physical contact (rising edge of the robot body's contact points) also end a move.
"""
from __future__ import annotations
import math

WHEEL_R, AXLE, MAX_WHEEL = 0.033, 0.160, 6.67          # TurtleBot3 Burger: wheel radius, wheel separation, max wheel speed (rad/s)

def wrap(a: float) -> float: return (a + math.pi) % (2 * math.pi) - math.pi
def clamp(v, lo, hi): return max(lo, min(hi, v))

class Plant:
    """What DiffDrive needs. Angles in radians, lengths in metres."""
    dt = 0.032
    def set_wheels(self, wl: float, wr: float): raise NotImplementedError
    def step(self): raise NotImplementedError
    def encoders(self) -> tuple: raise NotImplementedError          # (left, right) wheel angles, rad
    def truth(self) -> tuple: raise NotImplementedError             # (x, y, yaw_rad); evaluation only, never used for control
    def lidar(self) -> list: raise NotImplementedError              # ranges, one per ray; inf/NaN where nothing is seen
    def contact_events(self) -> int: raise NotImplementedError      # cumulative rising edges of robot-body contact

class DiffDrive:
    def __init__(self, plant: Plant, v_max=0.20, gate=1.2, sector_half=8):
        self.p, self.v_max, self.gate, self.sector_half = plant, min(v_max, MAX_WHEEL * WHEEL_R), gate, sector_half
        self.front_idx = None
        self.steps = 0
        self.reset_odom()

    # --- odometry ---------------------------------------------------------------------------------------------
    def reset_odom(self):
        x, y, yaw = self.p.truth()
        self.ox, self.oy, self.oyaw = x, y, yaw
        self.enc = self.p.encoders()
    def _update_odom(self):
        l, r = self.p.encoders()
        dl, dr = (l - self.enc[0]) * WHEEL_R, (r - self.enc[1]) * WHEEL_R
        self.enc = (l, r)
        ds, dth = (dl + dr) / 2, (dr - dl) / AXLE
        self.ox += ds * math.cos(self.oyaw + dth / 2); self.oy += ds * math.sin(self.oyaw + dth / 2); self.oyaw = wrap(self.oyaw + dth)
    def odom_error(self) -> float:
        x, y, _ = self.p.truth(); return math.hypot(self.ox - x, self.oy - y)

    def _tick(self, v, w):
        wl = clamp((v - w * AXLE / 2) / WHEEL_R, -MAX_WHEEL, MAX_WHEEL); wr = clamp((v + w * AXLE / 2) / WHEEL_R, -MAX_WHEEL, MAX_WHEEL)
        self.p.set_wheels(wl, wr); self.p.step(); self.steps += 1; self._update_odom()
    def _stop(self, settle=2):
        for _ in range(settle): self._tick(0.0, 0.0)

    # --- lidar ----------------------------------------------------------------------------------------------------
    def lidar_front(self) -> float:
        if self.front_idx is None: raise RuntimeError("lidar front index unknown: run calibrate first")
        rs = self.p.lidar(); n = len(rs)
        vals = [rs[(self.front_idx + j) % n] for j in range(-self.sector_half, self.sector_half + 1)]
        vals = [v for v in vals if v == v and v != float("inf")]
        return min(vals) if vals else float("inf")

    # --- motion ---------------------------------------------------------------------------------------------------
    def rotate(self, delta_deg: float) -> bool:
        """Turn in place by delta_deg (CCW positive) on odometry heading. Returns False on timeout."""
        target = wrap(self.oyaw + math.radians(delta_deg))
        for _ in range(int((abs(delta_deg) / 8.0 + 4.0) / self.p.dt)):
            err = wrap(target - self.oyaw)
            if abs(err) < math.radians(0.3): self._stop(); return True
            w = clamp(2.5 * err, -1.5, 1.5)
            if abs(w) < 0.15: w = math.copysign(0.15, err)
            self._tick(0.0, w)
        self._stop(); return False

    def move_one_cell(self, cell: float = 1.0) -> str:
        """Drive `cell` metres along the current (90-degree snapped) heading. Returns ok | stalled | contact | timeout."""
        psi = round(self.oyaw / (math.pi / 2)) * (math.pi / 2)
        x0, y0, c0, s_mark = self.ox, self.oy, self.p.contact_events(), 0.0
        for k in range(int(9.0 / self.p.dt)):
            s = (self.ox - x0) * math.cos(psi) + (self.oy - y0) * math.sin(psi)
            rem = cell - s
            if rem < 0.004: self._stop(); return "ok"
            if self.p.contact_events() > c0: self._stop(); return "contact"
            if k and k % 30 == 0:
                if s - s_mark < 0.002: self._stop(); return "stalled"
                s_mark = s
            v = clamp(1.2 * rem, 0.04, self.v_max)
            self._tick(v, clamp(3.0 * wrap(psi - self.oyaw), -0.8, 0.8))
        self._stop(); return "timeout"

    def drive_cells(self, n: int) -> tuple:
        """Move up to n cells. Returns (cells_completed, reason) where reason is ok | lidar | stalled | contact | timeout."""
        for k in range(n):
            if self.lidar_front() < self.gate: return k, "lidar"
            r = self.move_one_cell()
            if r != "ok": return k, r
        return n, "ok"

# ---------------------------------------------------------------------------------------------------------------
class FakePlant(Plant):
    """Ideal differential-drive kinematics with square obstacles/walls, a 360-ray lidar and a contact flag.
    The lidar's ray 0 is deliberately NOT the front (`front_index`), so calibration is really exercised."""
    def __init__(self, front_index=270, rays=360, radius=0.105):
        self.front_index, self.rays, self.radius = front_index, rays, radius
        self.boxes = []                       # (cx, cy, half_x, half_y)
        self.x = self.y = self.yaw = 0.0
        self.wl = self.wr = 0.0; self.encL = self.encR = 0.0
        self.contacts = 0; self._touch = False
    def set_pose(self, x, y, yaw): self.x, self.y, self.yaw = float(x), float(y), float(yaw)
    def set_wheels(self, wl, wr): self.wl, self.wr = clamp(wl, -MAX_WHEEL, MAX_WHEEL), clamp(wr, -MAX_WHEEL, MAX_WHEEL)
    def _hit(self, x, y):
        for cx, cy, hx, hy in self.boxes:
            qx, qy = clamp(x, cx - hx, cx + hx), clamp(y, cy - hy, cy + hy)
            if math.hypot(x - qx, y - qy) < self.radius: return True
        return False
    def step(self):
        dl, dr = self.wl * WHEEL_R * self.dt, self.wr * WHEEL_R * self.dt
        ds, dth = (dl + dr) / 2, (dr - dl) / AXLE
        nx, ny = self.x + ds * math.cos(self.yaw + dth / 2), self.y + ds * math.sin(self.yaw + dth / 2)
        if self._hit(nx, ny):                                   # blocked: no penetration, wheels spin in place
            if not self._touch: self.contacts += 1
            self._touch = True; self.yaw = wrap(self.yaw + dth); self.encL += self.wl * self.dt; self.encR += self.wr * self.dt; return
        self._touch = False
        self.x, self.y, self.yaw = nx, ny, wrap(self.yaw + dth)
        self.encL += self.wl * self.dt; self.encR += self.wr * self.dt
    def encoders(self): return (self.encL, self.encR)
    def truth(self): return (self.x, self.y, self.yaw)
    def contact_events(self): return self.contacts
    def lidar(self):
        out = []
        for i in range(self.rays):
            ang = self.yaw + math.radians(i - self.front_index)
            dx, dy = math.cos(ang), math.sin(ang); best = float("inf")
            for cx, cy, hx, hy in self.boxes:
                t0, t1 = 0.0, 1e9; ok = True
                for o, d, lo, hi in ((self.x, dx, cx - hx, cx + hx), (self.y, dy, cy - hy, cy + hy)):
                    if abs(d) < 1e-12:
                        if o < lo or o > hi: ok = False; break
                    else:
                        a, b = (lo - o) / d, (hi - o) / d
                        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
                if ok and t0 <= t1 and t0 > 0: best = min(best, t0)
            out.append(best if 0.12 <= best <= 3.5 else float("inf"))
        return out
