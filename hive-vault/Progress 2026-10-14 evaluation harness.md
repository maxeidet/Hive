# Evaluation harness progress, 14 Oct 2026

What was built to test the report's baselines for interception and detection. How to
write and run tests in general is in [[Testing]]; the detection team's own work is in
[[Progress 2026-10-13]].

Back to [[Home]].

---

## In short

There is now a harness that runs the report's evaluation scenarios for **interception**
and **detection** and prints the report's metrics. The scenarios, the run loop and the
scoring are finished and tested. The methods themselves (pure pursuit, the dark-object
detector, YOLO26, the sector scan) are **empty placeholders** behind abstract
interfaces, because other people are writing them: as soon as one is filled in, its
sweep test stops skipping and prints the table for the report. It is on the
`feat-baseline-eval` branch, merged with `origin/dev`.

## What was done

| # | What | Where |
|---|---|---|
| 1 | Intercept scenarios: head-on and crossing at 30/60/90°, target speeds 4–17 m/s, 30 seeds each | `evaluation/scenario.py` |
| 2 | Intercept metrics: hit, time to hit, lock-on, distance to base at hit, target's closest distance to base, closest miss | `evaluation/metrics.py` |
| 3 | Intercept interfaces + placeholders: `InterceptAction` (`PurePursuitAction`), `Detector` (`NoisyTruthDetector`) | `intercept/interceptor.py`, `intercept/target_detector.py` |
| 4 | Detection scenarios: static ring of 12 defenders, a hostile drone and 3 birds from random directions, until the hostile reaches the ring | `detect/scenario.py` |
| 5 | Detection metrics: missed drones, detection probability vs range, birds flagged, background false alarms; on labelled images recall vs size, inference time | `detect/metrics.py`, `detect/dataset.py` |
| 6 | Detection interfaces + placeholders: `ImageDetector` (`DarkObjectDetector`, `Yolo26Detector`, `PixelModelDetector`), `GimbalScan` (`SectorScan`), `ImageDataset` (`GazeboImageDataset`) | `detect/detector.py`, `detect/scan.py`, `detect/dataset.py` |
| 7 | Truth per camera image (`Frame`), on the detection team's camera model and `ground_truth.project` | `detect/frame.py` |
| 8 | Fast simulation for both (seconds instead of hours in Gazebo) | `tests/sim.py` |
| 9 | Settings from the report's tables as config | `intercept:` and new `detect:` in `config/swarm.yaml`, `config.py` |
| 10 | Gazebo runner for one intercept scenario | `tests/intercept_baseline_test.py`, `./mount-backend.sh baseline-intercept` |
| 11 | Testing guide: kinds of test, rules, standard run output, checklist | [[Testing]] |

Paths are under `ros2_ws/src/swarm_backend/` and `swarm_backend/`.

Tests: `tests/test_intercept_metrics.py` and `tests/test_detect_metrics.py` test the
harness itself with small test doubles (no real method needed).
`tests/test_intercept_baseline.py` and `tests/test_detect_baseline.py` are the sweeps;
they skip until the placeholders are filled in, and their asserts are the report's
expectations, so they are the acceptance tests. 140 tests pass, 9 skip.

## How it fits together

Every component follows the same pattern (details in [[Testing]] §3):

```
make_scenario(seed)  ->  run_scenario(scenario, sim, method factories)  ->  score_run  ->  summarize
     seeded               knows only the interfaces; sim = fast or Gazebo     from the truth   the table
```

To evaluate a method, subclass its interface and pass the class to `run_scenario`.
Every method runs on the same seeded scenarios, so results compare directly.

## Merged with dev

`origin/dev` (the detection team's work, also on `feat-yolo` and
`feat-intercept-baseline`) was merged in, with dev's work taking priority:

- `mount-backend.sh` keeps all of dev's commands (`snap`, `predict`, `detect`,
  `detecttest`, `SWARM_CONFIG`) plus `baseline-intercept`. `.gitignore` keeps both
  `/data/` and `runs/`.
- Both sides had a `detect/camera.py`. Dev's is kept unchanged; the harness now uses
  its `CameraModel` (field of view 2.0 rad) and `ground_truth.project` instead of its
  own camera math.
- A camera pose is forward/right/down vectors, the format `snap` saves next to each
  image, so saved images can become evaluation frames directly.
- `DarkObjectDetector` is meant to wrap dev's existing baseline
  (`detect/candidates.py` + `detect/tracking.Confirmer`), and `GazeboImageDataset` to
  read the PNG + JSON pairs from `snap`. Both say so in their docstrings.
- None of dev's files were changed. Dev's tests (`test_camera`, `test_candidates`,
  `test_ground_truth`, `test_tracking`) pass, and `scenarios/scenario1.yaml` still loads.

## Merged with the attack branch

`origin/feat-intercept-baseline` (the intercept team's baseline attack, see
[[Baseline attack - how to run]]) was merged in afterwards, again with their work first:

- Their `intercept/detection.py` (the two detection topics and the fake detector) took
  that path. The harness's `Detector` interface moved to `intercept/target_detector.py`.
- Their `intercept/attack.py` is the real pure pursuit, so the harness's duplicate
  `guidance.pure_pursuit_velocity` stub was removed. `PurePursuitAction` is meant to wrap
  their `AttackAction`, and `NoisyTruthDetector` their fake detector (`measure`).
- `swarm.yaml` and `config.py` keep all three sections (`intercept`, `attack`,
  `detect`). `mount-backend.sh` keeps `fakedetect`, `attack`, `attacktest` and
  `baseline-intercept`.
- `Drone.set_velocity` now takes a `yaw`; the fast sim's `SimDrone` follows it.

## Results so far

No real results yet, since the methods are placeholders. To check the harness, both
sweeps were run with throwaway stand-ins (not committed):

- **Intercept, pure pursuit:** hits every target slower than the interceptor
  (v_i = 12 m/s) and almost none faster, as the report predicts. 600 runs in about 20 s.
- **Detection, 115° camera:** 97 % of hostile drones reached the ring undetected with
  12 defenders, 53 % with 60. Nothing was detected beyond about 50 m, the range where
  a 0.5 m drone is 4 px wide. This is the report's argument for a narrower camera.
  30 runs in about 45 s.

## Decisions

- **Goal radius x = 100 m** (`goal_radius`): the report leaves it open. A hit
  closer to the base than this does not count as a success.
- **Intercept detection = true position + noise** until YOLO exists.
- Runs are **scored from the truth only**, seeded, and in simulation time.
- **Fast sim for the numbers, Gazebo to check them.** The report's grid (~600 intercept
  runs) is not practical in Gazebo.

## Known issues and open questions

- **Detection range is horizontal in the intercept scenarios.** A target up to 70 m
  above or below the interceptor can start further away than the camera could
  really see it. This inflates the head-on times to hit.
- **Lock-on in fast head-on runs.** A hit often comes before 3 s of lock-on, so the
  lock-on rate is low there. This follows from the definition; worth a sentence in
  the report.
- **No Gazebo runner for detection yet.** It needs the camera images in `Frame.image`
  and the truth from `ground_truth`.
- **The intercept harness's `NoisyTruthDetector` is separate from the detection
  `ImageDetector`.** An adapter can connect them once YOLO runs.
- **The standard run folder ([[Testing]] §4) has no shared recorder yet.** Each
  test writes its own output.

## Next

1. Fill in the placeholders: `PurePursuitAction` wrapping the attack team's
   `AttackAction`, `DarkObjectDetector` wrapping dev's baseline, `PixelModelDetector` and `SectorScan`. The sweeps then
   give the report's tables.
2. `GazeboImageDataset` on the images from `snap`, for recall vs size on real images.
3. A shared recorder that writes the standard run folder, used by all flight tests.
4. A Gazebo runner for detection; then the system evaluation (detection + intercept
   together: mission success and mission time).
