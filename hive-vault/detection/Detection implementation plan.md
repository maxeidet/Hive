# Detection implementation plan

How to build the detection component described in the planning report (`report/Planning_report-HIVE_G3/sections/detection.tex`, "Drone Detection Using Sensors") on top of the code that exists today. Written 9 Oct 2026 against `main` / `feat-yolo`; status updated 17 Oct.

Back to [[Home]]. Related: [[Codebase map]], [[Final state]], [[Lock-on brainstorm]], [[ai_components]].

---

## Status (updated 17 Oct 2026)

The baseline detector is done and flies with Max's attack on `dev`. Since 13 Oct the work followed [[Baseline attack work plan]] instead of steps 2–4 below: instead of a report message and the chase's tracker, the detector publishes the two agreed topics (`/px4_N/detection/target_position` and `target_offset`) and Max's attack flies on them. All five perception tasks there were done on 14 Oct; the first join test (head-on hit, crossing miss) is in [[Join test results 2026-10-14]]. How to run the detector: [[Tutorial - live detector (detect)]].

**Built** (`swarm_backend/detect/`)

| What | Where |
|---|---|
| Baseline detector: dark object against the sky, square black-hat window, sky found by its colours (no horizon line needed) | `candidates.py` |
| One detector interface, so the detector is swapped with one option (`--detector baseline\|yolo`, `predict yolo`); `yolo` is a placeholder until it is trained | `detectors.py` |
| Camera math: pixel → world direction, size → distance | `camera.py` |
| Camera orientation from PX4's gimbal status, matched to the image time through Gazebo's clock (0.35° median against the truth) | `orientation.py`, `px4_camera.py` |
| k-in-a-row confirmation, matched by world direction | `tracking.py` |
| Live detector: publishes the two topics and a debug picture | `live.py`, `mount-backend.sh detect` |
| Offline scoring on saved images with the truth | `predict.py`, `mount-backend.sh predict` |
| Flight test (not flown with this detector yet) | `tests/detection_test.py`, `mount-backend.sh detecttest` |

**Open:** why the lock drops when the target crosses the view; PX4's clock jumps (proposed fix: `UXRCE_DDS_SYNCT` = 0); `detecttest`; then steps 5–10 below (YOLO and its data).

---

## What the report asks for

| Part        | What it must do                                                                                                                                                                   |
| ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Find        | Flag small objects in the gimbal camera image, from about 4 px across                                                                                                             |
| Classify    | Drone or bird, from about 20 px across                                                                                                                                            |
| Direction   | Unit vector from the drone to the target in the world frame, with uncertainty, plus a rough range`r ≈ f·s/p`                                                                      |
| Baseline    | Dark object against the sky: pixels that differ from the sky → connected regions → plausible size → drone if seen in`k` consecutive images                                        |
| Improvement | YOLO26n on enlarged crops around candidates (on the drone), ByteTrack for identities, drone if confidence >`τ` in `k` consecutive images. YOLO26s later at the base station       |
| Training    | Gazebo images with automatic labels (drones, birds, empty sky; 20–120 m altitude; split by run), fine-tune COCO weights to 2 classes: Seraphim first, then Gazebo + some Seraphim |
| Output      | One*detection report* per followed target (time, drone, track, stage, class + confidence, direction ± σ, range ± σ, bounding box)                                                 |
| Evaluation  | Baseline vs YOLO on the same seeded scenarios: missed drones, P(detect) vs range, birds flagged, background false alarms, recall vs pixel size, inference time on CPU             |

## What exists to build on

- `px4/drone.py`: `Drone` has `position`, `attitude` and `gimbal` (pitch, yaw relative to the nose), and `aim_gimbal()` to keep the camera on a direction. Everything needed to turn a pixel into a world direction, except camera intrinsics.
- `intercept/guidance.py`: `gimbal_angles()` and `pointing_error_deg()` already convert between world directions and gimbal angles. The new camera math should reuse these conventions (NED, pitch from the horizon, yaw from the nose).
- `intercept/target.py`: `Fix` / `TargetSource`, so the chase can take measurements from any source. Detection becomes one more source, next to the mock radar.
- `intercept/tracker.py`: EKF for radar fixes and position fixes, horizontal only, fixed target altitude. It needs a bearing update.
- `sensor_view.py`: already finds a drone's Gazebo camera topics and bridges them to ROS 2 with `ros_gz_bridge`. The same topic discovery and bridge config can feed the detector.
- `gz_x500_gimbal` (drone 2 in `swarm.yaml`): the 115°, 1280×720 platform camera of the baseline.
- Missing (9 Oct): a narrow camera model, a bird model, a scenario runner that scores detection. OpenCV is in the container since then; Ultralytics is not.

## Design

New subpackage `swarm_backend/detect/`, written like `intercept/`: pure Python with no ROS in everything except the node, so it is unit tested with pytest outside the container.

What exists (17 Oct) and what is still planned:

```text
swarm_backend/detect/
├── camera.py       built: CameraModel (f, cx, cy), pixel → world direction, range from width
├── candidates.py   built: baseline, dark object against the sky
├── detectors.py    built: one interface for every detector (box, score, label);
│                   the YOLO26 wrapper goes here
├── orientation.py  built: camera orientation from PX4's gimbal status
├── px4_camera.py   built: reads it from PX4 in the live node
├── tracking.py     built: k-in-a-row by world direction; ByteTrack later for YOLO
├── live.py         built: the live node, publishes the two agreed topics
├── predict.py      built: scoring on saved images
└── crops.py        planned: enlarged crops around candidates for YOLO26n
```

**Pipeline per image.** The chosen detector (`detectors.py`) gives boxes with a score; tracking, confirmation, direction, distance and the published topics are the same whichever detector runs, which makes the comparison fair. Planned for YOLO: candidates first, then YOLO26n on the crops around them.

**Direction and range.** Box centre → ray in the camera frame (intrinsics) → world NED with the camera orientation from PX4's gimbal status (built this way, see `orientation.py`). Direction σ ≈ (pixel σ + pointing error) / f, with pointing error measured in simulation against Gazebo poses. Range = f·s/p with s = 0.5 m and σ from a ±1 px width error plus an assumed ±50 % size error. Direction and range stay separate, as in the report.

**Into the tracker** (not built; the baseline attack flies on the two topics instead). Add a `BearingFix(Fix)` to `intercept/target.py` (origin, unit direction, angular σ, optional range ± σ) and an EKF update for it in `KalmanTracker`. A `DetectionSource(TargetSource)` turns the newest confirmed report into a `BearingFix`. `ChaseAction` then uses `[detection, radar]` in order of preference, so the existing chase locks on to the camera when it has one and falls back to the mock radar when it does not. This is Stage 1 of the report's stages.

**Message format** (the report's open TODO). For the baseline attack it is settled: two standard topics, no message package ([[Baseline attack work plan]]). For the base station, the earlier recommendation still stands: a small `hive_interfaces` package (ament_cmake, `msg/DetectionReport.msg` and `msg/DetectionReportArray.msg`) that mirrors the table field for field. `vision_msgs/Detection2DArray` has boxes and classes but no world direction, range or their σ, so it would need a sidecar message anyway. Uncertainty as one angular σ (radians, symmetric around the direction) and one range σ (metres); upgrade to covariances only if the base station needs them. The node publishes at the camera rate on `/<ns>/detection/reports`; the base-station rate `Δt = 0.2 s` is a downsample done by the sender later.

## Training data and models

Details, and what has been decided since: [[Training data and YOLO training]].

**Gazebo dataset generator** (`tools/gen_detection_data.py`, runs in the container, not part of the ROS package):

- A world with sky, clouds, trees and ground (`hive_base` already has the forest), one camera on a static or teleported gimbal, and drone and bird models moved with `gz service .../set_pose` per image.
- Distances drawn so the target spans 2–100 px, weighted towards 4 and 20 px; altitude 20–120 m; random lighting (sun direction) and background; 10–20 % empty images.
- Labels: prefer Gazebo's bounding-box camera (`boundingbox_camera` sensor with the `Label` system on each model), which also handles trees in the way. Fallback: project the model's 3D box with `camera.py` (the same code the detector uses, so it is tested twice).
- Output YOLO format, with a run id per image so train / val / test are split by run.
- Needs a bird model: three (dove, mourning dove, a flock) are on `feat-birds` in `worlds/birds/`, spawned with `./mount-backend.sh bird`; their size and orientation are still to be checked ([[Manual tests to do]]). A CC0 mesh like the trees in `worlds/trees/`, two or three species and sizes (0.3–1.5 m wingspan).

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

| Step | Deliverable | Done when | Status (17 Oct) |
|---|---|---|---|
| 1 | `detect/camera.py` + tests | Pixel → world ray → pixel round-trips; a Gazebo-projected target lands on the box within 2 px | Done |
| 2 | Image bridge + `detection_node` skeleton (reuse `sensor_view` discovery), pose buffer | Node logs images with the interpolated drone and gimbal pose | Done as `live.py`, with the camera orientation from PX4 instead of a pose buffer |
| 3 | `candidates.py` + `tracking.py` (k consecutive) + `report.py` | Baseline reports a hostile drone against the sky from drone 2 | Done, publishing the two agreed topics instead of `report.py`; `detecttest` not flown with it yet |
| 4 | `BearingFix`, tracker update, `DetectionSource` | `chase()` keeps the gimbal on the hostile drone from the camera alone (Stage 1: lock-on) | Replaced by Max's attack node, which keeps the gimbal on the target from the camera alone |
| 5 | Narrow camera model `worlds/models/x500_gimbal_narrow` (20°, 1920×1080) | Same as step 3 at 5–10× the range | Not started |
| 6 | Bird model + dataset generator | A labelled, run-split dataset with the pixel-size histogram the report asks for | Bird models started on `feat-birds` (`birds.py`, `worlds/birds/`); no generator |
| 7 | Training (Seraphim → Gazebo), `τ` | YOLO26n/s weights + val/test numbers | Not started |
| 8 | `classifier.py` + `crops.py` in the pipeline | YOLO mode runs on the drone at a measured rate | Not started |
| 9 | Scenario runner + scoring script | Baseline vs YOLO table and plots for the report | Not started (`detection_test.py` and `attack_test.py`'s `data/attack_runs.csv` are first pieces) |
| 10 | Base-station side (YOLO26s on crops, friendly filter) | Hand-off to the base station work ([[ai_components]]) | Not started |

Steps 6–7 can run in parallel with step 5.

## Camera facts

Checked in the running container (9 Oct): the `x500_gimbal` camera is `horizontal_fov` 2.0 rad (114.6°), 1280×720, 30 Hz, no lens distortion, on the gimbal's `camera_link`, so `f ≈ 411 px` (`f = (W/2) / tan(hfov/2)`), which gives the report's 51 m detection range at 4 px (`411 · 0.5 / 4`). Its Gazebo topic is `/world/<world>/model/x500_gimbal_<id>/link/camera_link/sensor/camera/image`. The camera frame is front-right-down; a pixel is the ray `(1, (u−cx)/f, (v−cy)/f)`. Steps 1–3 were built on this; how they work now is in the code and [[Tutorial - live detector (detect)]].

## Open questions

- [ ] Message format for the base station: custom `hive_interfaces` messages (recommended) or `vision_msgs` + a sidecar? (For the attack it is settled: the two topics.)
- [ ] Which camera for the narrow model: 20° at 1920 px (the report's example) or 30°?
- [ ] Where training runs: Stellar `--gpu`, a university cluster or someone's own GPU? Seraphim access and licence need checking.
- [ ] Container size: add `opencv-python-headless` now; add `ultralytics` + CPU PyTorch only for step 8, or run inference through ONNX Runtime instead. (OpenCV is settled: apt `python3-opencv`, added 9 Oct. The YOLO runtime is still open.)
- [ ] The report says `k` consecutive images for detection but `k` of `n` for lock-on. Same rule, or two?
- [ ] [[Final state]] and `swarm.yaml` still use a base radar, while the report says Gazebo has none. This plan keeps the mock radar as a cue and fallback until camera detection works. Confirm that is the intent.
