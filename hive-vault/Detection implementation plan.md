# Detection implementation plan

How to build the detection component described in the planning report (`report/Planning_report-HIVE_G3/sections/detection.tex`, "Drone Detection Using Sensors") on top of the code that exists today. Written 9 Oct 2026 against `main` / `feat-yolo`.

Back to [[Home]]. Related: [[Codebase map]], [[Final state]], [[Lock-on brainstorm]], [[ai_components]].

---

## Status (updated 9 Oct 2026)

On `feat-yolo` (commits `8863650`..`3e514f8`). The baseline detector runs end to end on drone 2's Gazebo camera, but has not been flight tested against the hostile drone yet.

**Done**

| What | Where | Notes |
|---|---|---|
| OpenCV in the container | `docker/Dockerfile`, `package.xml` | `python3-opencv` (apt); `python3-numpy` and `sensor_msgs` also added to `package.xml`. The running container got it by hand; rebuild the image to keep it. |
| Bridge reuse | `sensor_view.py` | `gz_topics()` and `start_bridge()` moved out of `main()`, used by `view` and the detector. |
| Candidates (step 3) | `detect/candidates.py` | As planned: `min(R,G,B)` → black-hat (15 px ellipse) → threshold `max(min_contrast, mean + 4σ)` → connected regions, size ≤ kernel, aspect ≤ 4. **Added:** regions that are part of a larger dark object are dropped (black-hat at 1/4 resolution), because the black-hat leaves small fragments along the edges of trees and near drones. Sky mask via `above_line()` (handles roll). `draw()` for debug images. |
| Confirmation (step 3) | `detect/tracking.py` | `Confirmer`: greedy nearest neighbour inside a gate, confirmed after `k` in a row, a miss resets the streak, dropped after `max_misses`. The distance is a parameter: pixels today, `angle_between` on unit vectors once the camera math exists. |
| Camera basics (part of step 1) | `detect/camera.py` | `CameraModel`: `f` (≈ 411 px for the gimbal camera), `cx`, `cy`, `horizon_row(pitch)`, `range_from_width`, `width_at`. No pixel ↔ world ray yet, and no tests of its own yet. |
| Live detector (part of step 2) | `detect/live.py`, `mount-backend.sh detect [ID]` | `LiveDetector` bridges the camera, reads images without `cv_bridge`, runs candidates + confirmation at 10 Hz on the newest image, publishes `/<ns>/detection/debug` (yellow = candidate, red = confirmed) and can save raw + annotated PNGs (`--save`). Sky mask is a fixed row (`--horizon`). |
| Offline tuning | `detect/annotate.py` | Runs the detector on saved PNGs on a laptop, no ROS. |
| Flight test | `tests/detection_test.py`, `mount-backend.sh detecttest` | Drone 2 hovers at 30 m and keeps its gimbal on drone 0 (true positions), which flies away at 45 m from 15 to 70 m. Each image is scored against the expected pixel (small-angle model from the gimbal's reported angles). Summary per 10 m: candidate near, confirmed near, false confirmed per image, plus median box offset and ms/image. Horizon mask from the gimbal pitch. Timeouts in sim time. |
| Unit tests | `tests/test_detect.py` | 16 tests on synthetic sky images: discs of 3–13 px found; empty sky, soft cloud, 40 px object and thin line not found; mask; noise raises the threshold; confirmation rules; angle gate. Pass on Windows (OpenCV 5, NumPy 2) and in the container (OpenCV 4.6, NumPy 1.26). |

**Seen so far**
- Live on drone 2 standing on the ground (level gimbal, no mask): the pipeline works, about 88 ms per 1280×720 image on the container CPU. With no sky mask it finds ~20 candidates in gaps between tree trunks at the horizon, which is why the mask is needed.
- On the dev laptop Gazebo runs at a real-time factor of 0.04–0.06 (software rendering with `llvmpipe`, WSL memory and swap full, drone 1's 3D lidar, the Gazebo GUI). The full flight test is ~150 sim s, about an hour of wall time there. Use Stellar, or close the GUI, drop drone 1 and give WSL more memory.

**Next**
1. Run `detecttest` and record P(detect) against range and pixel size.
2. Step 1 proper: `pixel_to_ray`, `direction`, `project` + `tests/test_camera.py`, then match tracks in world direction instead of pixels.
3. Step 2 rest: pose buffer (`detect/pose_buffer.py`), attitude and gimbal in `DroneObserver`, clock check, a proper `detection_node`.
4. Step 3 rest: `report.py` and the `hive_interfaces` messages, publish `/<ns>/detection/reports`.
5. Speed: 88 ms/image is just under the 10 Hz budget; profile it (the large-object pass, mask building) before adding YOLO.

---

## What the report asks for

| Part | What it must do |
|---|---|
| Find | Flag small objects in the gimbal camera image, from about 4 px across |
| Classify | Drone or bird, from about 20 px across |
| Direction | Unit vector from the drone to the target in the world frame, with uncertainty, plus a rough range `r ≈ f·s/p` |
| Baseline | Dark object against the sky: pixels that differ from the sky → connected regions → plausible size → drone if seen in `k` consecutive images |
| Improvement | YOLO26n on enlarged crops around candidates (on the drone), ByteTrack for identities, drone if confidence > `τ` in `k` consecutive images. YOLO26s later at the base station |
| Training | Gazebo images with automatic labels (drones, birds, empty sky; 20–120 m altitude; split by run), fine-tune COCO weights to 2 classes: Seraphim first, then Gazebo + some Seraphim |
| Output | One *detection report* per followed target (time, drone, track, stage, class + confidence, direction ± σ, range ± σ, bounding box) |
| Evaluation | Baseline vs YOLO on the same seeded scenarios: missed drones, P(detect) vs range, birds flagged, background false alarms, recall vs pixel size, inference time on CPU |

## What exists to build on

- `px4/drone.py`: `Drone` has `position`, `attitude` and `gimbal` (pitch, yaw relative to the nose), and `aim_gimbal()` to keep the camera on a direction. Everything needed to turn a pixel into a world direction, except camera intrinsics.
- `intercept/guidance.py`: `gimbal_angles()` and `pointing_error_deg()` already convert between world directions and gimbal angles. The new camera math should reuse these conventions (NED, pitch from the horizon, yaw from the nose).
- `intercept/target.py`: `Fix` / `TargetSource`, so the chase can take measurements from any source. Detection becomes one more source, next to the mock radar.
- `intercept/tracker.py`: EKF for radar fixes and position fixes, horizontal only, fixed target altitude. It needs a bearing update.
- `sensor_view.py`: already finds a drone's Gazebo camera topics and bridges them to ROS 2 with `ros_gz_bridge`. The same topic discovery and bridge config can feed the detector.
- `gz_x500_gimbal` (drone 2 in `swarm.yaml`): the 115°, 1280×720 platform camera of the baseline.
- Missing: any image handling (no OpenCV / Ultralytics in the container), a narrow camera model, a bird model, a message package (`hive_interfaces` is not started), a scenario runner that scores detection.

## Design

New subpackage `swarm_backend/detect/`, written like `intercept/`: pure Python with no ROS in everything except the node, so it is unit tested with pytest outside the container.

```text
swarm_backend/detect/
├── camera.py       CameraModel (fov, width, height → f, cx, cy); pixel ↔ world ray using
│                   drone attitude + gimbal angles; project a world point to a pixel;
│                   range from box width (f·s/p) with its σ
├── candidates.py   baseline: dark object against the sky (OpenCV black-hat + threshold,
│                   connected components, size filter) → list of boxes
├── crops.py        enlarged crops around candidates for YOLO (fixed crop size, merge
│                   overlapping ones), and mapping boxes back to full-image pixels
├── classifier.py   Classifier protocol + Yolo26 wrapper (Ultralytics) + a ground-truth
│                   "perfect" classifier for evaluation
├── tracking.py     ByteTrack on full-image boxes; k-consecutive / τ confirmation per track
├── report.py       DetectionReport dataclass (fields of the report's table) and
│                   to_bearing_fix() for the tracker
└── pipeline.py     image + pose → candidates → crops → classes → tracks → reports
nodes/detection_node.py   one per drone: image + odometry + gimbal in, reports out
```

**Pipeline per image.** `candidates` always runs. In baseline mode every candidate box goes straight to tracking with stage *candidate*. In YOLO mode the crops around candidates go through YOLO26n, and its boxes replace the candidates with stage *classified*. Tracking and confirmation are the same in both modes, which makes the comparison fair.

**Pose at image time.** Image stamps are Gazebo sim time; PX4 odometry and `GimbalDeviceAttitudeStatus` are on PX4's clock (also sim time in SITL). Keep a short buffer of drone attitude and gimbal angles and interpolate to the image stamp. A gimbal moving at 0.2 s per position gives large direction errors if this is skipped. The gimbal's mount offset from the body is read once from the model SDF.

**Direction and range.** Box centre → ray in the camera frame (intrinsics) → body frame (gimbal pitch/yaw) → world NED (drone attitude). Direction σ ≈ (pixel σ + pointing error) / f, with pointing error measured in simulation against Gazebo poses. Range = f·s/p with s = 0.5 m and σ from a ±1 px width error plus an assumed ±50 % size error. Direction and range stay separate, as in the report.

**Into the tracker.** Add a `BearingFix(Fix)` to `intercept/target.py` (origin, unit direction, angular σ, optional range ± σ) and an EKF update for it in `KalmanTracker`. A `DetectionSource(TargetSource)` turns the newest confirmed report into a `BearingFix`. `ChaseAction` then uses `[detection, radar]` in order of preference, so the existing chase locks on to the camera when it has one and falls back to the mock radar when it does not. This is Stage 1 of the report's stages.

**Message format** (the report's open TODO). Recommendation: a small `hive_interfaces` package (ament_cmake, `msg/DetectionReport.msg` and `msg/DetectionReportArray.msg`) that mirrors the table field for field. `vision_msgs/Detection2DArray` has boxes and classes but no world direction, range or their σ, so it would need a sidecar message anyway. Uncertainty as one angular σ (radians, symmetric around the direction) and one range σ (metres); upgrade to covariances only if the base station needs them. The node publishes at the camera rate on `/<ns>/detection/reports`; the base-station rate `Δt = 0.2 s` is a downsample done by the sender later.

## Training data and models

**Gazebo dataset generator** (`tools/gen_detection_data.py`, runs in the container, not part of the ROS package):
- A world with sky, clouds, trees and ground (`hive_base` already has the forest), one camera on a static or teleported gimbal, and drone and bird models moved with `gz service .../set_pose` per image.
- Distances drawn so the target spans 2–100 px, weighted towards 4 and 20 px; altitude 20–120 m; random lighting (sun direction) and background; 10–20 % empty images.
- Labels: prefer Gazebo's bounding-box camera (`boundingbox_camera` sensor with the `Label` system on each model), which also handles trees in the way. Fallback: project the model's 3D box with `camera.py` (the same code the detector uses, so it is tested twice).
- Output YOLO format, with a run id per image so train / val / test are split by run.
- Needs a bird model (none in `worlds/` yet). A CC0 mesh like the trees in `worlds/trees/`, two or three species and sizes (0.3–1.5 m wingspan).

**Fine-tuning** (Ultralytics, on a GPU outside the dev container):
1. `yolo26n.pt` and `yolo26s.pt`, `nc=2` (`drone`, `bird`). Ultralytics rebuilds the class head when `nc` differs and keeps the rest of the weights, which is what the report describes.
2. Stage 1 on Seraphim, Stage 2 on Gazebo + about 20 % Seraphim.
3. YOLO26n trained on crops of the same size used at run time (e.g. 320 or 640 px), YOLO26s on full images.
4. Early stopping on val; `τ` chosen on val for near-zero missed drones; report everything on held-out Gazebo runs.
5. Export the chosen weights to ONNX for CPU inference in the container, and commit only a small model card plus a download link, not the weights.

## Evaluation

- A scenario runner (grows out of `tests/intercept_test.py`) that spawns the hostile drone and birds from seeded random directions and logs, per image: true pixel position of every object (projected from Gazebo poses), every report, and timing.
- Matching by box centre within a few pixels, not IoU, as the report says.
- One script computes all six measures from the logs, so baseline and YOLO runs are scored by the same code. The same runs with the ground-truth classifier give the "perfect detection" case the interception section asks for.
- Inference time measured on the container's CPU at the real crop size and count.

## Order of work

| Step | Deliverable | Done when | Status (9 Oct) |
|---|---|---|---|
| 1 | `detect/camera.py` + tests | Pixel → world ray → pixel round-trips; a Gazebo-projected target lands on the box within 2 px | Started: intrinsics, horizon row, range from size |
| 2 | Image bridge + `detection_node` skeleton (reuse `sensor_view` discovery), pose buffer | Node logs images with the interpolated drone and gimbal pose | Started: bridge + `live.py` with debug image; no pose buffer |
| 3 | `candidates.py` + `tracking.py` (k consecutive) + `report.py` | Baseline reports a hostile drone against the sky from drone 2 | Mostly: candidates + confirmation done and unit tested; no `report.py`; flight test written, not run |
| 4 | `BearingFix`, tracker update, `DetectionSource` | `chase()` keeps the gimbal on the hostile drone from the camera alone (Stage 1: lock-on) | Not started |
| 5 | Narrow camera model `worlds/models/x500_gimbal_narrow` (20°, 1920×1080) | Same as step 3 at 5–10× the range | Not started |
| 6 | Bird model + dataset generator | A labelled, run-split dataset with the pixel-size histogram the report asks for | Not started |
| 7 | Training (Seraphim → Gazebo), `τ` | YOLO26n/s weights + val/test numbers | Not started |
| 8 | `classifier.py` + `crops.py` in the pipeline | YOLO mode runs on the drone at a measured rate | Not started |
| 9 | Scenario runner + scoring script | Baseline vs YOLO table and plots for the report | Not started (`detection_test.py` is a first piece) |
| 10 | Base-station side (YOLO26s on crops, friendly filter) | Hand-off to the base station work ([[ai_components]]) | Not started |

Steps 1–4 need nothing new in the container except OpenCV. Steps 6–7 can run in parallel with 4–5 once step 1 exists.

## Steps 1–3 in detail

Checked in the running container (9 Oct): the `x500_gimbal` camera is `horizontal_fov` 2.0 rad (114.6°), 1280×720, 30 Hz, no lens distortion, on the gimbal's `camera_link`. Its Gazebo topic is `/world/<world>/model/x500_gimbal_<id>/link/camera_link/sensor/camera/image` (plus `camera_info`), which `sensor_view.SENSOR_TOPIC` already matches. NumPy 1.26 is installed; Python OpenCV is not (only the C++ libs), and neither is `cv_bridge`. `vision_msgs` is installed.

Steps 1 and 3 are pure Python and can be written and tested on a laptop in parallel. Step 2 needs the container.

### Step 1: camera math (`detect/camera.py`)

**Model.** `CameraModel(hfov, width, height)` with `f = (W/2) / tan(hfov/2)`, `cx = W/2`, `cy = H/2`, square pixels. For the platform camera `f ≈ 411 px`, which gives the report's 51 m detection range at 4 px (`411 · 0.5 / 4`). Values come from `camera_info` when it is there, and from the model SDF otherwise. Gazebo's camera is an ideal pinhole, so no distortion model is needed.

**Frames.** The camera frame is front-right-down: x along the optical axis, y right, z down. Image u goes right and v goes down, so a pixel is the ray `(1, (u−cx)/f, (v−cy)/f)`, normalised. The camera's orientation in world NED comes from PX4's `GimbalDeviceAttitudeStatus` quaternion. With `YAW_IN_EARTH_FRAME` set (as `PX4Drone._on_gimbal` already handles), that quaternion is the full camera attitude in NED, roll included. The gimbal sits about 0.3 m from the drone's centre, which is ignored: it is under 1 % of any range we care about.

**Functions.** All take and return plain tuples or NumPy arrays:
- `pixel_to_ray(u, v)` → unit vector in the camera frame
- `direction(u, v, q_cam)` → unit vector in world NED
- `project(point, cam_position, q_cam)` → `(u, v)`, or `None` if the point is behind the camera or outside the image. This is used for labels and evaluation.
- `range_from_width(p_px, size=0.5)` → `(r, σ_r)`
- `direction_sigma(pixel_sigma, pointing_sigma)` → radians

**Tests** (`tests/test_camera.py`, no ROS):
- `project` then `direction` round-trips.
- A target 100 m north with level gimbal, yaw 0 and heading 0 lands on `(cx, cy)`. 10° east of that lands on `u = cx + f·tan 10°`.
- Pointing the gimbal with the existing `guidance.gimbal_angles(me, heading, target)` puts the target at the image centre. This keeps the camera math and the chase on the same conventions.

**Ground-truth check** (`tests/camera_check.py`, Gazebo): hover drone 2 with the hostile drone at a known spot, project the hostile drone's Gazebo pose into the image, and save the image with a crosshair. The crosshair must sit on the drone within 2 px. The camera pose comes from Gazebo's `dynamic_pose/info`, the way `sensor_view.TFPublisher` reads it. Any error here is in the frames, so fix it before step 2.

### Step 2: images in, pose at image time

**Bridge.** Take the bridge start out of `sensor_view.main()` and put it in a function that the detector and `view` both call. It uses `find_sensors()` and `bridge_config()` to bring the gimbal camera and `camera_info` to `/<ns>/camera/image`.

**No `cv_bridge`.** Read the image directly: `np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)` for `rgb8`. Subscribe best-effort with depth 1 and process at 10 Hz first (drop frames). 1280×720 RGB at 30 Hz is about 80 MB/s through the bridge.

**Pose.**
- Extend the read-only `px4/observer.py` `DroneObserver` with attitude and gimbal. Move the gimbal handling in `PX4Drone._on_gimbal` into a shared function so the two cannot drift apart.
- Add a `detect/pose_buffer.py` that keeps the last ~1 s of `(t, position, q_cam)`. `at(t)` interpolates position and slerps orientation, and returns `None` when `t` is more than 50 ms outside the buffer.
- Check the two clocks first thing. The image stamp is Gazebo sim time and the PX4 timestamp is PX4's own clock. Log both side by side, and if they differ by a constant, add a `clock_offset` setting.

**Node.** `nodes/detection_node.py`, run as `ros2 run swarm_backend detection_node --id 2 --mode baseline` or `./mount-backend.sh detect 2`.
- Inputs: images, odometry and gimbal status.
- Outputs: reports, plus `/<ns>/detection/debug`, an image with the boxes and track ids drawn on it. The debug image gets its own display in the RViz layout `view` uses, which makes tuning step 3 visual.

**Dependencies.** Add `python3-opencv` (apt, Ubuntu noble) to `docker/Dockerfile`, and add `python3-opencv` and `python3-numpy` to `package.xml`.

**Done when** the node logs each image with an interpolated pose, and the debug image shows the projected ground-truth position of the hostile drone on top of it.

### Step 3: baseline detector, confirmation, reports

**Candidates** (`detect/candidates.py`, `find_candidates(rgb, horizon_v=None) -> list[Box]`):
1. Use the blue channel or `min(R, G, B)`. The sky is bright in blue and the drone is dark in every channel.
2. Black-hat morphology, with a kernel about twice the largest target expected (15×15 px for the platform camera). This keeps small dark things and drops slow brightness changes across the sky.
3. Adaptive threshold: black-hat > mean + k·σ of the image, with k ≈ 4 to start.
4. `cv2.connectedComponentsWithStats`. Keep areas from 3 px² to about 60×60 px, with aspect ratio < 4.
5. Sky mask: drop everything below the horizon line, which `camera.project` gives from the camera's pitch. The report already says the baseline only works against the sky, so this mask is the baseline's honest limit, not a hack.

Tests use synthetic images: a gradient sky with noise, dark discs of 3, 5 and 20 px (all must be found), and a large soft "cloud" (must not be found). Add a few PNGs saved from Gazebo in step 2 as regression cases in `tests/data/`.

**Association and confirmation** (`detect/tracking.py`). Behind the same interface ByteTrack will use in step 8, start with a nearest-neighbour associator that matches in **world direction**, not pixels. When the gimbal turns, the target jumps across the image but stays put in world direction. Gate at a few times the direction σ plus target motion. A track is confirmed after `k` consecutive images (start with `k = 3` at 10 Hz), and a miss resets the count.

**Reports.**
- `detect/report.py` holds the report dataclass. Stage is *candidate*, class is *drone* with no confidence for the baseline, and direction and range come from `camera.py`.
- Make the small `hive_interfaces` package here, since the field list is already fixed by the report's table.
- Publish on `/<ns>/detection/reports`.

**Done when** `tests/detection_test.py` runs in Gazebo and the baseline confirms the hostile drone and keeps the track. The setup is drone 2 hovering at 50 m with the gimbal level, and drone 0 flying the `line` pattern 20–40 m away. That is the 5–10 px range for this camera; at 50 m it is only 4 px. The test logs the direction error against Gazebo's true pose (target < 1°) and the range error (expect about ±50 %), like `intercept_test.py` does for the chase.

## Open questions

- [ ] Message format: custom `hive_interfaces` messages (recommended) or `vision_msgs` + a sidecar?
- [ ] Which camera for the narrow model: 20° at 1920 px (the report's example) or 30°?
- [ ] Where training runs: Stellar `--gpu`, a university cluster or someone's own GPU? Seraphim access and licence need checking.
- [ ] Container size: add `opencv-python-headless` now; add `ultralytics` + CPU PyTorch only for step 8, or run inference through ONNX Runtime instead. (OpenCV is settled: apt `python3-opencv`, added 9 Oct. The YOLO runtime is still open.)
- [ ] The report says `k` consecutive images for detection but `k` of `n` for lock-on. Same rule, or two?
- [ ] [[Final state]] and `swarm.yaml` still use a base radar, while the report says Gazebo has none. This plan keeps the mock radar as a cue and fallback until camera detection works. Confirm that is the intent.
