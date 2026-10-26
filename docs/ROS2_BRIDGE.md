# ROS 2 bridge (v0.12): the benchmark on a ROS 2 robot

`orbit_ros/ros2_adapter.py` adds `Ros2Sim`, a third simulator backend next to `FakeSim` and `WebotsSim`. Agents, tasks and success checks do
not change: `WebotsWorld2D.adopt(world, sim, perception, motion)` wraps the task world exactly as for Webots, and the gate is the same
(>= 95% agreement with pure World2D, every disagreement explained).

**Status: not run against ROS 2 or Gazebo by the author.** The pure helpers (wheel speeds -> Twist, quaternions, SDF scene, joint names)
and the glue between `DiffDrive` and `Ros2Plant` are unit-tested (`tests/test_v012.py`, `Ros2HelperTests`); everything that imports
`rclpy` is untested. Treat the first run as debugging.

## What it expects (ROS 2 Jazzy + gz sim + TurtleBot3 Burger; all names are constructor arguments of `Ros2Sim`)

| interface | type | used for |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (`twist_stamped=True` for `TwistStamped`, the Jazzy TurtleBot3 default) | wheel commands from `DiffDrive` |
| `/joint_states` | `sensor_msgs/JointState` with `wheel_left_joint`, `wheel_right_joint` | wheel encoders (odometry) |
| `/scan` | `sensor_msgs/LaserScan` | lidar gate before each cell step (front ray found by `calibrate_lidar`) |
| `/ground_truth` | `nav_msgs/Odometry`, true robot pose | evaluation and teleports only, never control |
| `/bumper` | `std_msgs/Bool` (optional) | contact count; without it contacts stay 0 |
| `/world/<world>/create`, `/remove`, `/set_pose` | `ros_gz_interfaces` services, bridged with `ros_gz_bridge` | spawning the task objects and walls, resetting the robot |

The camera is computed from the ground-truth pose with a pinhole model (as `FakeSim` does). To test real perception, replace
`Ros2Sim.recognition()` by a subscriber to a detector topic (e.g. `vision_msgs/Detection2DArray`) that returns `label, f, l, px`.

## Steps
1. Start gz sim with an empty world named `default` and a TurtleBot3 Burger (`robot_name="turtlebot3_burger"`).
2. Bridge the topics and services above with `ros_gz_bridge` (ground truth: a pose or odometry publisher plugin on the robot).
3. `source /opt/ros/jazzy/setup.bash`, then in this folder: `python run_webots_c1.py --sim ros2 --mode probe`.
4. If the probe passes: `python run_webots_c1.py --sim ros2 --limit 24 --out results_ros2_c1` (grid motion), then `--motion continuous`.
5. Send back `results_ros2_c1/` and the console output. Without ROS 2 sourced, the runner stops with "ROS 2 is not available".
