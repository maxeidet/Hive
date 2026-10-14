# Manual: scenario 1, images and prediction

How to fly scenario 1, take camera images with the truth saved, and run the detector on them. Written 13 Oct 2026. Background: [[Detection implementation plan]], [[Progress 2026-10-13]].

Back to [[Home]].

---

Everything runs inside the container, so nothing needs to be installed on the Mac. All commands are run on the host, from `ros2_ws/src/swarm_backend/` unless it says otherwise.

## 1. Start the simulation

From the repo root:

```bash
docker/start-dev.sh
```

This builds the image, starts the container and opens a terminal with PX4 and Gazebo (drone 0). The Gazebo desktop is at http://localhost:6080/vnc.html; the password is printed by the script. Wait until Gazebo shows the world and drone 0.

The first start after 13 Oct rebuilds the image, because OpenCV was added to it.

## 2. Fly scenario 1

```bash
cd ros2_ws/src/swarm_backend
scenarios/scenario1.sh
```

What happens:
1. Drones 1 and 2 are started, both with a gimbal camera. Drone 0 stays on the ground.
2. Two RViz windows open on the VNC desktop, one per drone camera.
3. Both drones climb to 20 m. Drone 1 flies a circle (80 m across, 4 m/s). After 10 s, drone 2 starts chasing it and keeps its camera on it.
4. Ctrl-C makes both drones fly back above their start points and land, and closes RViz. A second Ctrl-C lands them where they are.

Useful options:

| Option | What it does |
|---|---|
| `--no-view` | No RViz windows. Much faster on the Mac |
| `--duration 120` | Stop after 120 simulation seconds instead of at Ctrl-C |
| `--speed 6` | Hostile speed in m/s |
| `--altitude 30` | Flight altitude in metres |

Other settings (how close the chaser gets, circle size, where the mock radar is) are in `scenarios/scenario1.yaml`.

Good to know:
- The simulation is slow on the Mac (about 7 % of real time with both RViz windows). The climb alone can take minutes.
- Do not run the swarm shell (`./mount-backend.sh`) while the scenario runs. Both would send commands to the same drones. The scenario refuses to start if a shell is open.
- If the script says the drones are not gimbal drones, they were already running as another type: run `docker/start-dev.sh` again and start over.

## 3. Take images

While the scenario flies, in a **second terminal**:

```bash
cd ros2_ws/src/swarm_backend
./mount-backend.sh snap 2 --count 10
```

This saves 10 images from drone 2's camera to `data/Manualimages/` in the repo, one per simulated second.

| Option | What it does |
|---|---|
| `snap 1 ...` | Use drone 1's camera instead |
| `--count 10` | How many images. Without it, it saves until Ctrl-C |
| `--every 3` | Simulation seconds between images (default 1; 0 = every image) |
| `--label sky` | Adds a word to the file names: `drone2_sky_000151.276.png` |

Each image gets a `.json` file with the same name. It holds the truth from Gazebo at that moment:
- the position of the camera drone and of its camera, and the directions the camera looks along;
- for every other drone: its position, the distance from the camera, the direction to it (unit vector) and the pixel where it shows in the image (`null` when it is out of view).

Positions and directions are in the world frame: x north, y east, z down, in metres.

The line printed for each image shows the distances, for example `drone 1: 9.8 m`. Use the RViz window of drone 2 to see when the target is in view and against the sky.

When flying by hand in the shell instead of with the scenario, the same thing is `snap(2, 10)` at the shell prompt.

To start a fresh set, move or delete the old files in `data/Manualimages/` first. The `data/` folder is not tracked by git.

## 4. Run the prediction

The scenario does not need to be running for this, only the container.

```bash
./mount-backend.sh predict            # the baseline detector
./mount-backend.sh predict yolo       # not trained yet: prints a message and stops
```

It reads every PNG in `data/Manualimages/` and writes to `data/results/baseline/`:

| Output | What it is |
|---|---|
| One image per input | A red box on each detection, with the estimated distance (red), the true distance (green), both direction vectors and the angle between them. A red and a green arrow from the image centre show the estimated and the true direction. A drone that is in view but was not found gets a green cross |
| `results.csv` | One row per detection: box, size in pixels, estimated and true distance, estimated and true direction, distance error, direction error in degrees, pixel error, which drone it was |
| `plots/` | Estimated against true distance, distance error, direction error, and the drone size that would make each distance exact |

Other folders: `./mount-backend.sh predict baseline --images /workspace/data/X --out /workspace/data/Y` (paths as seen inside the container, where the repo is `/workspace`).

To start clean, delete `data/results/baseline/` first; old result images are not removed by a new run.

> [!note] `predict` does more than a detector needs
> `predict` is an evaluation tool for now. Besides detecting, it reads the truth files, matches each detection to the real drone, computes the errors, draws text and arrows on every image, writes the CSV and makes the plots. None of that is needed to detect a drone during flight, and it is slow. This will be addressed later: the detection itself (find the drone, estimate distance and direction) will be separated from the evaluation, so the same detection code runs live on the drone and the evaluation is an optional step on top.

## 5. Run the detector live (publishes target_offset)

While a scenario or the shell flies, in another terminal:

```bash
./mount-backend.sh detect 2                      # baseline detector on drone 2's camera
./mount-backend.sh detect 2 --detector yolo      # once YOLO is trained
```

It publishes, for the best confirmed target, `/px4_2/detection/target_offset` (the angles from the image centre, see [[Baseline attack work plan]]) and a debug picture on `/px4_2/detection/debug` with the boxes, a white cross at the image centre and the offset in degrees next to the target. Watch the angles with `ros2 topic echo /px4_2/detection/target_offset` inside the container. The angles are not corrected for camera roll yet.

## How to read the results

- **The baseline only finds a drone against the sky.** A drone in front of trees or ground is missed; that is the detector's limit, not a bug. It also misses a drone wider than 40 px or cut off by the image edge.
- **Distance** is estimated from the box width, assuming the drone is 0.5 m wide. Expect errors of 20 % or more.
- **Direction** is estimated from where the box is in the image, using the camera's true orientation from the `.json`. So the direction error shown is from the detection alone.
- **The truth itself** is good to about 0.7°, so direction errors below 1° cannot be told apart.
- A detection with the text "no drone here" is a false alarm: nothing was within 40 px of where a drone really was.

## If something goes wrong

| Problem | What to do |
|---|---|
| `Container px4-gazebo-vnc is not running` | Run `docker/start-dev.sh` |
| The shell prints "PX4 accepted the gimbal pitch/yaw command" over and over | Two programs are flying the drones. Close the other shell or scenario |
| `snap` says "No image for 60 s" | Gazebo is paused or extremely slow. Check the VNC desktop |
| `predict` says `No module named 'cv2'` | The container is from an old image. Run `docker/start-dev.sh` to rebuild |
| The climb times out | Add `--no-view`, or raise `--climb-timeout` (wall-clock seconds) |
