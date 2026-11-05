# 3D mapping plan

How a drone builds a 3D model of an area when it is given a command, using the DAEP exploration planner from LiU. Written 5 Nov 2026 on branch `feat-3d-mapping`. Nothing is built yet: DAEP is cloned and the setup is described here.

Back to [[Home]]. Related: [[Codebase map]], [[StackV2]].

---

## In short

- **Goal:** a command gives a centre point and a radius. One drone flies there, explores the area and builds a 3D model of it, then saves the model.
- **DAEP decides where to fly**, it does not build the model. It picks positions that reveal the most unknown space and plans a safe path to them.
- **`octomap_server` builds the model** from the drone's depth camera or lidar. The model is an OctoMap: space split into small cubes (10 cm), each marked occupied, free or unknown.
- **The area is a boundary around the point:** DAEP already explores inside a box (`boundary/min`, `boundary/max`). We set that box from the centre and radius instead of a fixed config file.
- **DAEP is private** (LiU real-lab). It lives in our own forks on gitlab.liu.se and is never committed to Hive. See [Where the code lives](#where-the-code-lives).

## The pipeline

```
Command (centre + radius)
  → the drone flies to the area
  → DAEP picks positions that reveal unknown space
  → the depth camera / lidar sees  → octomap_server builds the 3D model
  → stop when the area is covered
  → save the model (.bt file, can be viewed in RViz)
```

| Step | What does it | Notes |
|---|---|---|
| Command | `ExploreAround` action (centre point, radius) | Result: the path of the saved map file and the coverage reached |
| Fly to the area | Bridge node → `PX4Drone.set_position()` | Same drone API as the rest of `swarm_backend` |
| Choose where to look | DAEP: `aeplanner` (local RRT + information gain) and `rrt` (global RRT* to cached frontiers), run by `rpl_exploration` | Boundary set from centre ± radius |
| Build the model | `octomap_server` with the point cloud from Gazebo (`ros_gz_bridge`) | Needs TF from the world frame to the sensor frame |
| Stop | DAEP reports "exploration complete" (no frontiers left), or the `Coverage` service passes a threshold (e.g. 95 %) | Threshold to tune |
| Save | `octomap_saver` → `.bt` | Convert to point cloud or mesh later if the report needs it |

## Where the code lives

Decided 5 Nov 2026: everything that depends on DAEP goes in the DAEP fork, so Hive builds for everyone without LiU access.

```
Hive/ros2_ws/src/
├── swarm_backend/   Hive (GitHub), unchanged
├── daep/            fork: gitlab.liu.se/josme010/daep, branch feat-3d-mapping (ignored by Hive's git)
└── daep_msgs/       fork: gitlab.liu.se/josme010/daep_msgs (ignored by Hive's git)
```

- `.gitignore` in Hive ignores `ros2_ws/src/daep/`, `ros2_ws/src/daep_msgs/`, `ros2_ws/src/lrs_*/`, DAEP run output (`ros2_ws/data/`) and saved maps (`*.ot`, `*.bt`). Commit `95361e5`.
- The bridge to PX4, the drone config and the `ExploreAround` action go in the DAEP fork.
- Only this note and other plain descriptions go in Hive. No DAEP code is copied here.
- Sources (Reporter access, read only): DAEP for ROS 2 `gitlab.liu.se/real-lab/daep`, messages `gitlab.liu.se/real-lab/daep_msgs`. The open ROS 1 version with a wiki that explains the parameters: `github.com/LudvigWiden/daeplanner`.

## What DAEP needs around it

DAEP is only the planner. The parts below come from the robot it was written for (TurtleBot4, Husky, Spot, DJI M100/M300) and must be provided by us for a PX4 drone.

| DAEP expects | What we provide |
|---|---|
| Someone that takes `goal` (`daep_msgs/Goal`: x, y, z, yaw) and answers on `/goal_reached` | **Bridge node** (new): flies the drone with `set_position()` and publishes `/goal_reached` when `reached()` is true. Also handles `/init_motion` (one turn on the spot at the start, so the first part of the map exists) |
| The robot pose on `pose` or `simple_pose` (`PoseStamped`) | PX4 odometry, converted to DAEP's frame |
| The map on `octomap` | `octomap_server` |
| A point cloud and TF `world → base_link → sensor` | `ros_gz_bridge` for the sensor, and TF from the Gazebo poses. `sensor_view.py` already does both, on private TF topics; reuse the logic |
| z up (ENU, like Gazebo) | Convert to and from Hive's NED (x north, y east, z down) in the bridge |
| `pig.py` and `dfm.py` running | They are not optional: `aeplanner` calls their services and waits for them |

## Drone config for DAEP

A new parameter file in the fork, based on the existing ones:

- `system/bbx/use_sphere_collision_checking: true`, so RRT samples in 3D. Without it, z is fixed to the ground robot's height.
- z boundary that fits the flying height, not 0–1 m.
- `camera/horizontal_fov` and `camera/vertical_fov` that match the sensor. The information gain is computed from these, so wrong values give bad choices.
- `drone_linear_velocity`, `drone_angular_velocity`, `world_frame`, `robot_frame`.
- The dynamic obstacle parts (Kalman prediction of people, DFM) can stay off; the area is static.

Sensor candidates: `gz_x500_depth` (OakD-Lite depth camera, publishes on one shared topic `/depth_camera`, fine for one drone) or our `gz_x500_lidar_3d` (360° 3D lidar).

## Order of work

1. **Build DAEP on Jazzy** in the Hive container (`colcon build --packages-up-to daep`). DAEP was written for ROS 2 Humble, Hive runs Jazzy. Known small issues: `irobot_create_msgs` (only used for TurtleBot docking, can be removed) and `lrs_util` in `pig.py` (only two quaternion functions, easy to replace).
2. **Sensor, TF and `octomap_server`** for one drone: see the map grow in RViz while flying by hand.
3. **Bridge node** (`goal` → PX4 → `/goal_reached`, pose, NED ↔ ENU).
4. **Run DAEP unchanged** with a fixed box in a known world, and check that it explores and stops.
5. **Boundary from centre + radius** at runtime, then the `ExploreAround` action.
6. **Save and evaluate:** `.bt` file, coverage over time (DAEP writes `coverage.csv`), time to finish.

## Open questions

- **Model format:** is the OctoMap (voxels) enough, or does the report need a point cloud or mesh? A mesh needs an extra conversion step at the end.
- **Area shape:** a box (exists today) or a sphere around the point (small change in `isInsideBoundaries` in `aeplanner.cpp` and `rrt.cpp`)?
- **Sensor:** depth camera or 3D lidar? It decides the FOV values and how fast the map fills.
- **Which world** to test in: it needs structure to map (buildings, trees), e.g. `walls`, `forest` or `hive_base`.
