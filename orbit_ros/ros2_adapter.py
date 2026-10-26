"""ROS 2 backend for the simulator adapter (v0.12): the same `WebotsWorld2D.adopt(world, sim, ...)` path, with a ROS 2 robot
(TurtleBot3 in Gazebo / gz sim via ros_gz) instead of Webots.

    python run_webots_c1.py --sim ros2 --mode probe                 # first: checks topics, services, lidar and motion
    python run_webots_c1.py --sim ros2 --motion continuous --limit 24 --out results_ros2_c1

What it uses (all names are constructor arguments; defaults follow TurtleBot3 + ros_gz conventions):
* /cmd_vel            geometry_msgs/Twist (or TwistStamped with twist_stamped=True, as on Jazzy)  <- wheel speeds from DiffDrive
* /joint_states       sensor_msgs/JointState, wheel_left_joint / wheel_right_joint positions         -> encoders
* /scan               sensor_msgs/LaserScan                                                             -> lidar
* /ground_truth       nav_msgs/Odometry of the true robot pose (bridge the gz pose / odometry publisher) -> truth (evaluation only)
* /bumper             std_msgs/Bool, optional; without it, contacts are not counted
* /world/<world>/create | remove | set_pose   ros_gz_interfaces services (bridge them with ros_gz_bridge) -> scene and teleports
The camera is modelled geometrically from the ground-truth pose (as FakeSim does); a detector node can replace `recognition()`.

STATUS: written against the ROS 2 Jazzy / ros_gz documentation and NOT run against ROS 2 or Gazebo by the author. The pure helpers
below are unit-tested; everything that imports rclpy is untested. Start with `--mode probe`.
"""
from __future__ import annotations
import math, time
from .continuous import Plant, WHEEL_R, AXLE
from .webots_adapter import SimBackend, OBJ_SIZE, OBJ_HEIGHT, WALL_H, wall_specs, label_color
from .world2d import to_robot_frame

# ---------------------------------------------------------------------------------------------------------- pure helpers (tested)
def wheels_to_twist(wl: float, wr: float, wheel_r: float = WHEEL_R, axle: float = AXLE) -> tuple:
    """Wheel angular speeds (rad/s) -> (linear m/s, angular rad/s) of a differential drive."""
    return (wl + wr) * wheel_r / 2, (wr - wl) * wheel_r / axle

def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def yaw_to_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

def box_sdf(name: str, sx: float, sy: float, sz: float, rgb=(0.5, 0.5, 0.5), static=True) -> str:
    r, g, b = rgb
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>{"true" if static else "false"}</static><link name="link">'
            f'<collision name="c">{geo}</collision><visual name="v">{geo}<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual></link></model></sdf>')

def scene_models(w: int, h: int, objects) -> list:
    """[(name, sdf, x, y, z)] for every object and the four room walls, in the same geometry as the Webots scene."""
    out = [(f"obj_{o.id}", box_sdf(f"obj_{o.id}", OBJ_SIZE, OBJ_SIZE, OBJ_HEIGHT, label_color(o.label)), float(o.x), float(o.y), OBJ_HEIGHT / 2)
           for o in objects]
    out += [(name, box_sdf(name, sx, sy, WALL_H, (0.45, 0.45, 0.5)), cx, cy, WALL_H / 2) for name, cx, cy, sx, sy in wall_specs(w, h)]
    return out

def wheel_positions(names, positions, left="wheel_left_joint", right="wheel_right_joint"):
    d = dict(zip(names, positions))
    if left not in d or right not in d: return None
    return (float(d[left]), float(d[right]))

ROS2_CONFIG_KEYS = ("world_name", "robot_name", "cmd_topic", "joint_topic", "scan_topic", "truth_topic", "bumper_topic", "twist_stamped",
                    "fov_deg", "dt", "settle_s", "timeout_s")

def load_ros2_config(path):
    """Ros2Sim keyword arguments from a JSON file (v0.13). Keys starting with '_' are comments; unknown keys are an error, so a typo in a
    topic name cannot fall back to a default silently. No file -> {} (the TurtleBot3 defaults)."""
    if not path: return {}
    import json
    with open(path, encoding="utf-8") as f: conf = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    bad = sorted(set(conf) - set(ROS2_CONFIG_KEYS))
    if bad: raise ValueError(f"unknown keys in {path}: {bad} (allowed: {', '.join(ROS2_CONFIG_KEYS)})")
    return conf

# ---------------------------------------------------------------------------------------------------------- rclpy side (untested)
class Ros2Plant(Plant):
    def __init__(self, sim: "Ros2Sim"):
        self.sim, self.dt = sim, sim.dt
        self.contacts, self._touch = 0, False
    def set_wheels(self, wl, wr): self.sim.publish_twist(*wheels_to_twist(wl, wr))
    def step(self):
        self.sim.spin_for(self.dt)
        touching = bool(self.sim.bumper)
        if touching and not self._touch: self.contacts += 1
        self._touch = touching
    def encoders(self):
        if self.sim.wheels is None: raise RuntimeError("no wheel positions on the joint_states topic (expected wheel_left_joint / wheel_right_joint)")
        return self.sim.wheels
    def truth(self):
        x, y, yaw = self.sim.robot_pose(); return (x, y, math.radians(yaw))
    def lidar(self):
        if self.sim.scan is None: raise RuntimeError("no LaserScan received on the scan topic")
        return list(self.sim.scan)
    def contact_events(self): return self.contacts

class Ros2Sim(SimBackend):
    def __init__(self, world_name="default", robot_name="turtlebot3_burger", cmd_topic="/cmd_vel", joint_topic="/joint_states",
                 scan_topic="/scan", truth_topic="/ground_truth", bumper_topic="/bumper", twist_stamped=False, fov_deg=90.0, dt=0.032,
                 settle_s=0.3, timeout_s=10.0):
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped
            from sensor_msgs.msg import JointState, LaserScan
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool
            from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
        except ImportError as e:
            raise RuntimeError(f"ROS 2 is not available ({e}). Source your ROS 2 installation (e.g. `source /opt/ros/jazzy/setup.bash`) "
                               "and install ros_gz_interfaces; see docs/ROS2_BRIDGE.md") from None
        self.rclpy, self.fov, self.dt, self.settle_s, self.timeout_s = rclpy, fov_deg, dt, settle_s, timeout_s
        self.world_name, self.robot_name, self.twist_stamped = world_name, robot_name, twist_stamped
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("orbit_ros_bridge")
        self._Twist, self._TwistStamped = Twist, TwistStamped
        self.cmd = self.node.create_publisher(TwistStamped if twist_stamped else Twist, cmd_topic, 10)
        self.wheels = self.scan = self._truth = None; self.bumper = False
        self.node.create_subscription(JointState, joint_topic, lambda m: setattr(self, "wheels", wheel_positions(m.name, m.position) or self.wheels), 10)
        self.node.create_subscription(LaserScan, scan_topic, lambda m: setattr(self, "scan", [float(r) for r in m.ranges]), 10)
        self.node.create_subscription(Odometry, truth_topic, self._on_truth, 10)
        self.node.create_subscription(Bool, bumper_topic, lambda m: setattr(self, "bumper", bool(m.data)), 10)
        self._spawn = self.node.create_client(SpawnEntity, f"/world/{world_name}/create")
        self._delete = self.node.create_client(DeleteEntity, f"/world/{world_name}/remove")
        self._setpose = self.node.create_client(SetEntityPose, f"/world/{world_name}/set_pose")
        self._SpawnEntity, self._DeleteEntity, self._SetEntityPose = SpawnEntity, DeleteEntity, SetEntityPose
        self.models, self._objs, self._plant, self.camera_width, self.t0 = [], [], None, 160, time.monotonic()
        self._wait(lambda: self._truth is not None, "ground-truth odometry on " + truth_topic)

    # ros plumbing
    def _on_truth(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self._truth = (p.x, p.y, math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w)) % 360)
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end: self.rclpy.spin_once(self.node, timeout_sec=max(0.0, end - time.monotonic()))
    def _wait(self, cond, what):
        end = time.monotonic() + self.timeout_s
        while not cond():
            if time.monotonic() > end: raise RuntimeError(f"timeout waiting for {what}")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
    def _call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=self.timeout_s): raise RuntimeError(f"service for {what} not available (bridge it with ros_gz_bridge)")
        fut = client.call_async(req); self._wait(fut.done, what); return fut.result()
    def publish_twist(self, v, w):
        msg = self._TwistStamped() if self.twist_stamped else self._Twist()
        t = msg.twist if self.twist_stamped else msg
        if self.twist_stamped: msg.header.stamp = self.node.get_clock().now().to_msg()
        t.linear.x, t.angular.z = float(v), float(w); self.cmd.publish(msg)

    # SimBackend
    def load_scene(self, w, h, objects, start, yaw):
        for name in self.models:
            req = self._DeleteEntity.Request(); req.entity.name = name; self._call(self._delete, req, "remove " + name)
        self.models, self.w, self.h = [], w, h
        self._objs = [(o.label, float(o.x), float(o.y)) for o in objects]
        for name, sdf, x, y, z in scene_models(w, h, objects):
            req = self._SpawnEntity.Request(); f = req.entity_factory
            f.name, f.sdf, f.pose.position.x, f.pose.position.y, f.pose.position.z = name, sdf, x, y, z
            self._call(self._spawn, req, "create " + name); self.models.append(name)
        self.set_robot_pose(start[0], start[1], yaw)
    def set_robot_pose(self, x, y, yaw):
        self.publish_twist(0.0, 0.0)
        req = self._SetEntityPose.Request(); req.entity.name = self.robot_name
        req.pose.position.x, req.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = yaw_to_quat(math.radians(yaw)); o = req.pose.orientation; o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self._call(self._setpose, req, "set_pose " + self.robot_name); self.spin_for(self.settle_s)
        if getattr(self, "_driver", None) is not None: self._driver.reset_odom()
    def robot_pose(self): return self._truth
    def object_cells(self): return list(self._objs)
    def recognition(self):
        x0, y0, yaw = self._truth; out = []
        for label, ox, oy in self._objs:
            f, l = to_robot_frame(ox - x0, oy - y0, yaw)
            if f <= 0: continue
            right = -l / f / math.tan(math.radians(self.fov / 2))
            out.append({"label": label, "f": f, "l": l, "px": round(right * self.camera_width / 2 + self.camera_width / 2, 1)})
        return out
    def sim_time(self): return time.monotonic() - self.t0
    def plant(self):
        if self._plant is None: self._plant = Ros2Plant(self)
        return self._plant
