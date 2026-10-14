# Testing

How we test and evaluate the swarm, so that every result in the report can be
reproduced and compared. Read this before adding a test. Code lives in
`ros2_ws/src/swarm_backend/`; see [[Codebase map]] for the layout.

> **Status (Oct 2026):** the unit-test and evaluation-harness conventions below
> are in use (intercept). The **standard run folder** (§4) is the agreed format
> but there is no shared recorder yet: until there is, a test writes those files
> itself. Flight tests still copy their connect/takeoff/land code (§6).

## 1. Three kinds of test

| Kind | What it checks | Needs | Where | How to run |
|---|---|---|---|---|
| **Unit** | One piece of pure logic: math, config, scoring | pytest, PyYAML, numpy. No ROS | `tests/test_*.py` | `python3 -m pytest tests/test_*.py` |
| **Sweep** (fast sim) | A method on *all* scenarios of a report section, many seeds; prints the report's metrics | Same as unit | `tests/test_<component>_baseline.py` etc. | `python3 -m pytest tests/test_intercept_baseline.py -s` |
| **Flight** (Gazebo) | One scenario with real PX4 drones, sensors and timing | The `px4-gazebo-vnc` container, drones spawned | `tests/<name>_test.py` (no `test_` prefix, so pytest skips it) | `./mount-backend.sh <command> [args]` |

Rules of thumb:
- Logic that can be tested without ROS **must** be, as a unit test. Keep ROS and PX4
  out of math modules (`formation.py`, `guidance.py`, `metrics.py`, `scenario.py`).
- Sweeps give the numbers for the report (hundreds of runs, seconds). Flight tests
  confirm that the sweep's simulation is not fooling us; run a few seeds of the same
  scenarios and compare.
- Run unit tests in the container (`docker exec -it px4-gazebo-vnc bash -c "cd
  /workspace/ros2_ws/src/swarm_backend && python3 -m pytest tests/test_*.py"`) or
  any Python ≥ 3.10 with pytest, PyYAML and numpy.

## 2. Rules for every evaluation

1. **Metrics come from the report.** Each test names the report section it
   evaluates and computes exactly the measures listed there, with the same names.
   If the report changes, change the test (and vice versa). See §7.
2. **Score from the truth.** Metrics use true positions (Gazebo / the simulation),
   never the method's own estimates. Estimates may be *recorded*, for diagnosis.
3. **Seeded.** Every random choice (spawn position, altitude, noise) comes from a
   seed. The same seed gives the same scenario for every method, so methods are
   compared on identical runs. Report results over seeds `0..eval_runs-1`.
4. **Simulation time.** Time everything on the drones' clock (`Drone.stamp`), not
   wall time; Gazebo often runs slower than real time.
5. **World NED frame**, metres, seconds, m/s, degrees in config and output
   (radians only inside the math). Distances to the base are horizontal; between
   drones they are 3-D.
6. **Settings in `swarm.yaml`.** Scenario values (speeds, radii, thresholds,
   number of runs) are config keys with a comment pointing at the report table they
   come from, not constants in the test.
7. **Methods are plug-ins.** The harness only knows interfaces (e.g.
   `InterceptAction`, `Detector`); a new method subclasses them and is passed in.
   Every method runs on the same harness, including with perfect and with real
   detection, to separate guidance errors from detection errors.

## 3. The evaluation harness pattern

Every component is evaluated the same way (intercept is the worked example, in
`swarm_backend/intercept/`):

```
make_scenario(..., seed, config) -> Scenario        scenario.py: geometry + seed, pure
scenario_grid(config) -> [Scenario]                 every combination x eval_runs seeds
run_scenario(scenario, sim, make_method, ...) -> [Sample]   the loop; knows only interfaces
score_run(samples, scenario, config) -> RunMetrics  metrics.py: one run, from the truth
summarize([RunMetrics]) -> str                      the table, one row per scenario cell
```

- `sim` is anything with the drones, `now()` and `advance(dt)`: `tests/sim.py`
  (`FastSim`, `SimDrone`) for sweeps, a small PX4 wrapper for Gazebo
  (`PX4Sim` in `tests/intercept_baseline_test.py`). The same scenario, loop and
  metrics run in both.
- A `Sample` is the truth at one step plus what the method reported (status,
  in view, detected). Metrics only ever see samples.
- Test `metrics.py` with hand-built samples, and the loop with tiny test doubles
  (see `tests/test_intercept_metrics.py`), so the harness is correct before any
  method exists.
- Until a method exists, its sweep test **skips** (catch `NotImplementedError`),
  and its asserts encode the report's expectations, so they become its acceptance
  test.

## 4. Standard run output

Every flight test (and a sweep, when asked to save) writes one folder per run.
`runs/` is not committed.

```
runs/<test>/<YYYY-MM-DD_HHMMSS>_<scenario>_s<seed>/
    meta.json       what produced the run
    samples.csv     one row per step (10 Hz), truth + method state
    metrics.json    the scored result (e.g. RunMetrics as a dict)
    bag/            optional: ros2 bag of raw sensor topics (images, lidar)
```

**`meta.json`**: `test`, `scenario` (all its fields), `seed`, `method`
(class names of the action/detector), `config` (the full `intercept:` or other
section used, after command-line overrides), `args`, `git_commit` (+ `dirty`),
`world`, `drone_models`, `started` (wall time), `sim_rate` (sim seconds per wall
second, measured).

**`samples.csv`**: these core columns, in this order, for every drone in the
test, then test-specific columns:

| Column | Unit | Meaning |
|---|---|---|
| `t` | s | since the run started, simulation time |
| `<role>_x`, `_y`, `_z` | m | world NED position (`role` = `interceptor`, `hostile`, `drone<N>`) |
| `<role>_vx`, `_vy`, `_vz` | m/s | world NED velocity |
| `<role>_yaw` | deg | heading from north |
| `<role>_gimbal_pitch`, `_gimbal_yaw` | deg | empty if no gimbal |
| `<role>_armed`, `_offboard` | 0/1 | PX4 state |
| test-specific | | e.g. `status`, `in_view`, `detected`, `estimate_x`, `phase` |

Everything here is available through the `Drone` interface, so one recorder can
fill it for any test. Use empty cells, not 0, for missing values.

**Raw sensor data** (needed for detection): record with `ros2 bag record` next to
the run, only the topics needed (`/px4_N/<sensor>/image`, `/points`), into `bag/`.
The bag's clock is simulation time, so it lines up with `samples.csv`.

## 5. Adding a test: checklist

- [ ] Which report section does it evaluate? Write it in the module docstring.
- [ ] Settings added to `swarm.yaml` and the config dataclass, with defaults that
      match the report and a comment citing it; `parse_config` test still passes.
- [ ] Scenario + metrics are pure functions with unit tests (hand-built samples,
      edge cases such as a hit between two samples).
- [ ] The loop runs with test doubles before the real method exists.
- [ ] Sweep test in `tests/test_<component>_*.py`, seeded, prints `summarize()`,
      asserts the report's expectations, skips while the method is a placeholder.
- [ ] Flight test (if needed) in `tests/<name>_test.py`, with a `mount-backend.sh`
      command and `--help`; reuses the same scenario/metrics; writes §4 output.
- [ ] Flight safety: fails *before* takeoff if something is missing; lands both
      drones on any error or Ctrl-C (`finally`); separates drones before landing
      straight down.
- [ ] Docs: the package README (how to run), [[Codebase map]] status table, §7 here.

## 6. Flight tests: the common shape

Until there is a shared helper, copy this order from `tests/intercept_test.py` /
`tests/intercept_baseline_test.py`:

1. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` so Ctrl-C reaches
   our code and we can still land.
2. Make the `PX4Drone`s and a `SwarmManager`; `with spinning(node):`.
3. `wait_until(manager.connected, ...)` (if it times out, Gazebo is probably
   paused or the drones are not spawned), then `manager.ready`.
4. Check that the method exists, then take off and fly to the scenario start.
5. Run the scenario; print the summary; write the run folder.
6. `finally`: stop the method, separate the drones, land, `wait_until(all_landed)`.

## 7. What is evaluated where

| Report section | Measures (report names) | Code | Status |
|---|---|---|---|
| Interception > Evaluation | hit, time to hit, lock-on (held, time to, longest), distance to base at hit, target's closest distance to base, closest interceptor–target distance; outcome hit / reached base / lost / timeout | `intercept/scenario.py`, `intercept/metrics.py`; `tests/test_intercept_metrics.py`, `tests/test_intercept_baseline.py`, `mount-backend.sh baseline-intercept` | Harness built; pure pursuit and the stand-in detector are placeholders |
| System Evaluation | mission success / failure, mission time (spawn to interception) | — | Not started (needs detection + intercept together) |
| Detection > Evaluation (scenarios) | missed drones (reach the ring undetected), detection probability vs range, birds flagged as drones, background false alarms per image; also warning distance/time at first detection | `detect/scenario.py`, `detect/metrics.py`, `detect/camera.py` (truth projection); `tests/test_detect_metrics.py`, `tests/test_detect_baseline.py` | Harness built; `DarkObjectDetector`, `Yolo26Detector`, `PixelModelDetector` (fast-sim stand-in) and `SectorScan` are placeholders. No Gazebo runner yet (needs images bridged into `Frame.image`) |
| Detection > Evaluation (labelled images) | recall vs target size (px), found and classified; background false alarms per image; inference time per image | `detect/dataset.py` (`split_by_run`, `evaluate_dataset`); `tests/test_detect_baseline.py` with `HIVE_DETECT_DATASET` (+ `HIVE_YOLO_WEIGHTS`) | Harness built; `GazeboImageDataset` (the data-collection format) is a placeholder |

A detection is correct when its centre is within `match_radius_px` of the object's
true centre projected into the image (`detect/metrics.match`).

## Related

[[Codebase map]], [[ai_components]], [[1octstella]], [[Lock-on brainstorm]]
