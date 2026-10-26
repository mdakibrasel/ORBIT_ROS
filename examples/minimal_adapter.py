"""Minimal simulator adapter for ORBIT-PEL (v0.14 milestone M8): the smallest backend a new simulator has to implement.

Run it:
    python examples/minimal_adapter.py

It (1) runs the 11-check conformance suite on this adapter and (2) runs a few PEL and C4 episodes on it and on World2D and compares them.
To connect YOUR simulator, copy this file and replace the bodies of the five methods marked "REPLACE" with calls to your simulator's
API (spawn objects, teleport or drive the robot, read its pose, read the camera's recognised objects). Nothing in the planner, the
registry, the guard or the execution manager changes. Full guide: docs/PEL_ADAPTER_GUIDE.md.

This example keeps its whole "simulator" in a few Python variables (a grid with teleporting robot), so it needs no install.
Grid motion only: continuous motion also needs `plant()` (wheels, encoders, lidar, contacts); see orbit_ros/continuous.py.
"""
from __future__ import annotations
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orbit_ros.webots_adapter import SimBackend
from orbit_ros.world2d import to_robot_frame

class MinimalSim(SimBackend):
    """Units: metres and degrees. Map frame: x right, y up, yaw 0 = facing +x, counter-clockwise positive (REP-103). 1 cell = 1 m."""
    camera_width = 160                                     # pixels across the camera image
    fov_deg = 90.0                                         # horizontal field of view of the camera

    def load_scene(self, w, h, objects, start, yaw):       # REPLACE: build the room, spawn one solid per object, place the robot
        self.room = (w, h); self.objs = [(o.label, float(o.x), float(o.y)) for o in objects]
        self.set_robot_pose(start[0], start[1], yaw)

    def set_robot_pose(self, x, y, yaw):                   # REPLACE: teleport the robot (grid motion calls this after every move/turn)
        self.pose = (float(x), float(y), float(yaw) % 360)

    def robot_pose(self):                                  # REPLACE: read the robot's pose back from the simulator -> (x_m, y_m, yaw_deg)
        return self.pose

    def object_cells(self):                                # REPLACE: where the simulator holds each object -> [(label, x_m, y_m)]
        return list(self.objs)

    def recognition(self):                                 # REPLACE: the camera's recognised objects, robot frame
        """[{"label", "f": metres forward, "l": metres to the LEFT, "px": pixel column of the object's centre (0 = left edge)}]."""
        x0, y0, yaw = self.pose; out = []
        for label, ox, oy in self.objs:
            f, l = to_robot_frame(ox - x0, oy - y0, yaw)
            if f <= 0: continue                            # behind the camera
            right = -l / f / math.tan(math.radians(self.fov_deg / 2))
            out.append({"label": label, "f": f, "l": l, "px": round(right * self.camera_width / 2 + self.camera_width / 2, 1)})
        return out

def main():
    from orbit_ros.contract_api import sim_adapter
    from orbit_ros.conformance import run_suite
    from orbit_ros.tasks2d import make_task
    from orbit_ros.agent2d import run_episode, PRESETS
    from orbit_ros.backends2d import OracleBackend
    from orbit_ros.sim_select import in_simulator
    from tools.sim_agreement import compare
    print("1) Conformance suite on MinimalSim (grid):")
    res = run_suite(sim_adapter(MinimalSim(), "grid", "minimal"))
    for r in res: print(f"   {'PASS' if r['passed'] else 'FAIL'}  {r['check']:<20} {r['detail'][:80]}")
    ok = all(r["passed"] for r in res)
    print(f"   -> {sum(r['passed'] for r in res)}/{len(res)} checks passed")
    print("2) Same episodes on World2D and on MinimalSim (SYNTHETIC oracle planner):")
    sim, agree, n = MinimalSim(), 0, 0
    for fam, yaw in (("goto", 90), ("clean_left", 180), ("sequence", 0), ("prohibit", 270)):
        for cfg in ("PEL", "C4"):
            rows = []
            for backend in ("world2d", sim):
                w, t = make_task(fam, 0, yaw, 3, False, 1)
                rows.append(run_episode(in_simulator(w, backend), t, PRESETS[cfg], OracleBackend(0.05, 0.3, 0.1, abstain=True)))
            d = compare(*rows); n += 1; agree += not d
            print(f"   {fam:<11} {cfg:<4} success {rows[0]['success']}/{rows[1]['success']}  {'identical' if not d else 'DIFFERS: ' + ', '.join(d)}")
    print(f"   -> {agree}/{n} episodes identical")
    return 0 if ok and agree == n else 1

if __name__ == "__main__": sys.exit(main())
