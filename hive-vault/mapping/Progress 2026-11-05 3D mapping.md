# 3D mapping progress, 5 Nov 2026

What was built and tried on the first day of 3D mapping work. The plan is in [[3D mapping plan]]; how the tests are set up follows [[Testing]].

Back to [[Home]].

---

## In short

A drone with a depth camera (or lidar) can now build a 3D map (OctoMap) of what it sees in Gazebo: `./mount-backend.sh map`. The map grows as the drone flies, in the direction it flies, and can be saved as a `.bt` file. The DAEP exploration planner from LiU builds and starts on our ROS 2 Jazzy container, but it does not fly the drone yet: the bridge from DAEP to PX4 is the next step. Hive work is on `feat-3d-mapping`; the DAEP changes are in the private DAEP fork (same branch name), not in Hive.

## What was done

| # | What | Where |
|---|---|---|
| 1 | `feat-3d-mapping` brought up to date with `dev` | Hive |
| 2 | DAEP and daep_msgs forked from LiU real-lab and cloned next to our packages; Hive's `.gitignore` keeps them, DAEP run output and saved maps out of Hive | `ros2_ws/src/daep`, `ros2_ws/src/daep_msgs`, `.gitignore` |
| 3 | DAEP builds on Jazzy: 0 errors, only warnings. All its nodes start and connect (`pig.py`, `dfm.py`, `aeplanner`, `rrt`, `coverage_evaluation`, `rpl_exploration`) | DAEP fork |
| 4 | `pig.py` no longer needs the LiU-internal `lrs_util` (uses `tf_transformations`) | DAEP fork, `src/pig.py` |
| 5 | 5 DAEP launch files declared `config_package` but did not add it to the launch description, so it had to be passed by hand; fixed | DAEP fork, `launch/` |
| 6 | Public ROS packages for mapping and DAEP in the dev image: octomap, octomap_msgs, octomap_ros, octomap_server, irobot_create_msgs, tf_transformations, python3-rtree | `docker/Dockerfile` |
| 7 | 3D map from one drone: bridges the point cloud, publishes TF from Gazebo's true poses on a private topic, runs octomap_server, opens RViz with the voxels | `swarm_backend/mapping.py`, `./mount-backend.sh map [ID]` |
| 8 | Unit tests for the pure parts (sensor choice, octomap_server arguments, RViz layout, map statistics, NED → ENU, how far the map reaches along a bearing) | `tests/test_mapping.py` |
| 9 | Flight test: the map must grow and reach further in the direction flown | `tests/mapping_test.py`, `./mount-backend.sh maptest` |
| 10 | `mount-backend.sh` and the `docker/*.sh` scripts converted to LF in the working copy so they run from WSL (Git on Windows had checked them out with CRLF) | working copy only, no diff in git |

## How it fits together

```
Gazebo (depth camera / lidar)            Gazebo poses (truth)
   │ ros_gz_bridge                           │ TFPublisher (from sensor_view.py)
   ▼                                         ▼
/depth_camera/points ──► octomap_server ◄── /mapping_<id>/tf  (world → drone → base_link, camera_link)
                              │
                              ├─► /<ns>/octomap_full                (the 3D map; DAEP will read this)
                              └─► /<ns>/occupied_cells_vis_array    (voxels for RViz)
```

- The map is in the Gazebo world frame (ENU: x east, y north, z up), which is also what DAEP uses. The swarm backend uses NED; `mapping.ned_to_enu` converts.
- TF and the sim clock go on private topics (`/mapping_<id>/...`) like `view`, because every drone has a `base_link`.
- `mapping.py` has nothing to do with DAEP: it can be used on its own to map with any drone that has a depth camera or lidar.

## How to run

In the dev container with a depth-camera drone 0 (headless saves a lot of CPU):

```bash
./ros2_ws/src/swarm_backend/mount-backend.sh restart -w hive_base -m gz_x500_depth
./ros2_ws/src/swarm_backend/mount-backend.sh map 0          # map + RViz, Ctrl-C to stop
./ros2_ws/src/swarm_backend/mount-backend.sh maptest        # flight test, ~10-15 min on a laptop
```

Save the map while `map` runs:

```bash
ros2 run octomap_server octomap_saver_node --ros-args -r __ns:=/mapping -p octomap_path:=$PWD/map.bt
```

Unit tests: `python3 -m pytest tests/test_mapping.py tests/test_sensor_view.py` in `ros2_ws/src/swarm_backend`.

## Results so far

| Run | Occupied voxels | Notes |
|---|---|---|
| Drone 0 parked on the ground, `map` | 2 146 | All ground (z ≈ −0.1 m) in a fan 0.6–8 m in front of the drone: TF and camera frames are right (a wrong frame would put the ground up as a wall) |
| Same, after flying 3 m up, 6 m north, hover, land (`single_drone_test.py` with longer timeouts) | 12 973 | Reaches y = 14 m, in the direction flown. Few voxels above 0.5 m: no trees within 8 m of the start in `hive_base` |
| `maptest` (default: 3 m up, 6 m north, 10 s hover), run `2026-11-05_211657_b0_d6` | 1 255 → 10 460 | **PASSED**: ×8.3 voxels (need ×1.5), reaches 8.1 → 13.5 m north of the take-off point (+5.4 m, need +3 m). Map box after: x −5.3…4.3, y 6.4…19.4, z −0.1…3.8 m (ENU). Sim rate 0.11× real time; `map.bt` saved in the run folder |

The first `maptest` run aborted during take-off: it timed its waits on PX4's clock, which jumped +105 s (PX4 warns "clock jumped … (time sync?)"; it happened 3 times in the passing run too). The test now uses Gazebo's clock from the mapping bridge, like `attack_test.py`. The same run showed that `ros2 run` leaves the real node running when stopped, so `mapping.py` and the test start the executables directly.

## Known issues and open questions

- **Slow sim on a laptop.** With the depth camera the sim runs at a real-time factor of 0.10–0.15 (software rendering, Gazebo uses ~4 cores). Tests with wall-clock timeouts fail: `single_drone_test.py` gives up after 30 s waiting for take-off. `maptest` waits in sim time and takes about 10 minutes. Use Stellar for long runs.
- **PX4's clock jumps.** `Drone.stamp` jumped by 85–105 s several times per run. Anything timed on it can time out early; use Gazebo's clock.
- **Truth poses.** The map uses Gazebo's true poses, so it is more exact than a real drone would get. Fine for the simulation; say so in the report. PX4 odometry (NED → ENU) is the alternative.
- **One drone at a time.** Every `gz_x500_depth` publishes on the same `/depth_camera` topic, so two depth drones would mix their clouds. Use lidar drones (own topics) for several at once.
- **Model format.** The map is an OctoMap (voxels). A point cloud or mesh for the report needs a conversion step.

## Next

1. Bridge node in the DAEP fork: `goal` → PX4 setpoint (ENU → NED) → `/goal_reached`, plus `/init_motion` and the pose on `pose`.
2. Drone parameter file for DAEP (sphere collision checking, z bounds, the depth camera's FOV), reading `/mapping/octomap_full`.
3. DAEP explores a fixed box in `hive_base` and stops by itself.
4. Box from a centre and a radius, started by an `ExploreAround` action.
