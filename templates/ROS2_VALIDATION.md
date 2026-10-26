# ROS 2 + Gazebo validation record

> The ROS 2 bridge (`orbit_ros/ros2_adapter.py`) was written against the documentation and has NOT been run on ROS 2 by the author.
> Treat the first session as integration debugging. Never paste API keys or `.env` contents here.

## 1. Version matrix

| item | value |
|---|---|
| date (UTC) | |
| ORBIT ROS version and code hash | |
| OS (Ubuntu 24.04 recommended for Jazzy) | |
| ROS 2 distribution | Jazzy |
| Gazebo (gz sim) version | |
| ros_gz version | |
| TurtleBot3 packages and model | burger |
| Python / rclpy | |
| `configs/ros2_jazzy_tb3.json` changes (if any) | |

## 2. Topic and service snapshot

Run in the same terminal after `source /opt/ros/jazzy/setup.bash` and paste the output:

```
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once --field ranges | head
ros2 topic echo /ground_truth --once
ros2 service list | grep /world/
```

| name | expected type | seen type | publishing / available | note |
|---|---|---|---|---|
| /cmd_vel | geometry_msgs/TwistStamped (or Twist with `"twist_stamped": false`) | | | |
| /joint_states | sensor_msgs/JointState with wheel_left_joint, wheel_right_joint | | | |
| /scan | sensor_msgs/LaserScan | | | |
| /ground_truth | nav_msgs/Odometry (evaluator only, never an agent input) | | | |
| /bumper | std_msgs/Bool (optional) | | | |
| /world/<world>/create, /remove, /set_pose | ros_gz_interfaces services | | | |

## 3. Commands, in order

```
python run_webots_c1.py --sim ros2 --mode probe --ros2-config configs/ros2_jazzy_tb3.json
python conformance.py --sim ros2 --motion grid --ros2-config configs/ros2_jazzy_tb3.json
python run_webots_c1.py --sim ros2 --ros2-config configs/ros2_jazzy_tb3.json --limit 24 --out results_ros2_c1_<date>
python conformance.py --sim ros2 --motion continuous --ros2-config configs/ros2_jazzy_tb3.json
python run_webots_c1.py --sim ros2 --ros2-config configs/ros2_jazzy_tb3.json --motion continuous --limit 24 --out results_ros2_cont_<date>
```

## 4. Probe

- Result line (expected `[probe] RESULT: all checks passed`):
- Problems met (rclpy import, message type, bridge, spawn or reset, motion) and their fixes:

## 5. Conformance

Same 11 checks as `templates/WEBOTS_VALIDATION.md` section 4, for grid and continuous.

## 6. Small runs

| run | episodes | agreement with World2D % | gate | collisions | final pose error (m) | lidar gate | disagreements |
|---|---|---|---|---|---|---|---|
| grid | | | | | | | |
| continuous | | | | | | | |

## 7. Stop and safety behaviour (before any physical robot)

| test | how | result |
|---|---|---|
| stop command halts the robot | `stop_robot` during a long move | |
| velocity limits respected | compare /cmd_vel with the robot's limits | |
| safety gate stops before an obstacle | `--safety-gate` episode with an obstacle ahead | |
| emergency stop path | your stack's e-stop | |

## 8. Limitations

- The camera is geometric (computed from ground truth), not a detector.
-

## 9. What may be claimed

"ROS 2 validated" only after the probe, conformance and the small runs passed on a real ROS 2 + Gazebo stack. Unit tests of the pure
helpers are not enough.
