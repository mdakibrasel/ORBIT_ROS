# Connecting a simulator to ORBIT-PEL (adapter guide, v0.14 milestone M8)

ORBIT-PEL (planner, plan schema, tool registry, safety guard, execution manager) never talks to a simulator directly. It calls
`world.execute(tool, args)` on a World2D-compatible object and reads poses and camera views from it. A simulator joins by implementing
the small `SimBackend` interface (`orbit_ros/webots_adapter.py`); `WebotsWorld2D.adopt(world, sim)` then turns any task world into a
world that lives in that simulator. Nothing in the planning or execution code changes.

## 1. The five methods you implement (grid motion)

| method | returns | meaning |
|---|---|---|
| `load_scene(w, h, objects, start, yaw)` | none | build a room of w x h cells (1 cell = 1 m), one solid per object (`o.label`, `o.x`, `o.y`), robot at `start` facing `yaw` |
| `set_robot_pose(x, y, yaw)` | none | teleport the robot (grid motion calls this after every move and turn) |
| `robot_pose()` | `(x_m, y_m, yaw_deg)` | the robot's pose as the simulator holds it |
| `object_cells()` | `[(label, x_m, y_m)]` | where the simulator holds each object (used to build the occupancy the simulator enforces) |
| `recognition()` | `[{"label", "f", "l", "px"}]` | the camera's recognised objects: metres forward, metres to the LEFT, pixel column of the centre |

Plus the attribute `camera_width` (pixels). Conventions (REP-103): x right, y up on the map, yaw 0 = facing +x, counter-clockwise
positive, pixel 0 = left edge of the image.

**Continuous motion** additionally needs `plant()` returning an `orbit_ros.continuous.Plant` (wheel speeds, encoders, lidar ranges,
contact events, true pose); `WebotsSim` / `WebotsPlant` and `FakeSim` / `FakePlant` are the two existing examples.

## 2. A complete working example

`examples/minimal_adapter.py` keeps its "simulator" in a few Python variables. Run:

```
python examples/minimal_adapter.py
```

Expected output ends with `11/11 checks passed` and `8/8 episodes identical`. To connect your simulator, copy the file and replace the
five method bodies marked `REPLACE` with calls to your simulator's API.

## 3. Prove it before you claim it: conformance and agreement

1. **Conformance** (frames, units, turn sign, camera convention, freshness, invalid arguments, collision stop, stop, reset,
   ground-truth split, timeout status): `run_suite(sim_adapter(YourSim(), "grid", "yoursim"))` from `orbit_ros.conformance`, or
   `python conformance.py --sim webots|ros2` for the two built-in real simulators. All 11 checks must pass.
2. **Agreement with World2D** on the same task instances, PEL and baseline arms: `in_simulator(world, YourSim())` from
   `orbit_ros.sim_select` gives the simulator-backed world; `tools/sim_agreement.py` compares every non-timing column of the episode
   rows. In grid motion a correct adapter gives identical rows. In continuous motion physics may legitimately differ; report the
   differences, do not hide them.
3. Only then add the simulator to the compatibility matrix below, for the motion modes that passed, with the date and the file that
   holds the evidence.

The test `tests/test_pel_adapter.py` shows that the suite is not a formality: an adapter with a mirrored camera fails
`camera_convention`, and one with a clockwise yaw fails the frame checks.

## 4. Running the benchmark on a simulator

```
python run_spatial.py --planner oracle --suite all --configs C4 PEL --sim fake --motion grid --out results_fake_grid
python run_spatial.py --planner oracle --suite core --configs C4 PEL --sim webots --motion grid --out results_webots   # inside a Webots extern controller
python run_spatial.py --planner oracle --suite core --configs C4 PEL --sim ros2 --ros2-config configs/ros2_jazzy_tb3.json --out results_ros2
```

`--sim webots` and `--sim ros2` fail with an error when the simulator is not available; they never fall back to World2D.

## 5. Compatibility matrix (v0.14)

| backend | motion | conformance (11 checks) | agreement with World2D (oracle, PEL + C4) | evidence |
|---|---|---|---|---|
| World2D | grid | PASS 11/11 | reference | `conformance/conformance_world2d_grid.md` |
| FakeSim | grid | PASS 11/11 | see REPORT_V014.md (M5) | `results_v014_sim_agreement/sim_agreement_fake_grid.md` |
| FakeSim | continuous | PASS 11/11 | differs by design (lidar-gated stops, pixel quantisation) | `results_v014_sim_agreement/sim_agreement_fake_continuous.md` |
| MinimalSim (example) | grid | PASS 11/11 | 8/8 identical | `python examples/minimal_adapter.py` |
| Webots R2025a | grid / continuous | NOT RUN | NOT RUN | needs Webots and a TurtleBot world (removed from this project) |
| ROS 2 Jazzy + Gazebo | grid / continuous | NOT RUN | NOT RUN | needs a ROS 2 machine; `Ros2Sim` is untested |

Simulator independence is NOT established by this table: two of the passing rows are pure-Python simulators written for this project.
The claim needs at least one independent simulator (Webots or Gazebo) to pass both columns.
