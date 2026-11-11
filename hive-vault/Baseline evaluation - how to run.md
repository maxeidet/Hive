# Baseline evaluation: how to run

`./mount-backend.sh baseline-eval` flies the planning report's evaluation scenarios in Gazebo, with the system we have: three gimbal defenders on the surveillance arc, the base station, the real camera detector on every defender and the real pure pursuit. It scores the runs with the report's metrics, from the evaluation harness on `feat-baseline-eval` (see [[Testing]] and [[Progress 2026-10-14 evaluation harness]]). There are no mocks; Gazebo's truth is used only for scoring. Built 3 Nov 2026.

Back to [[Home]]. The system it flies: [[Surveillance attack test - how to run]], [[Base surveillance plan]].

---

## Run it

Terminal 1, from the repo root (Gazebo and drone 0; wait for `pxh>`):

```bash
docker/start-stellar.sh
```

Terminal 2, from `ros2_ws/src/swarm_backend/`. Start a `tmux` session first: the full grid takes many hours.

```bash
export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh spawn                                   # defenders 1-3 and the hostile drone 4
./mount-backend.sh baseline-eval intercept --aspects 0 --speeds 4 --seeds 0-2   # a first check
./mount-backend.sh baseline-eval intercept                 # the whole grid: 4 aspects x 5 speeds x 30 seeds
./mount-backend.sh baseline-eval detect                    # the 30 detection seeds
./mount-backend.sh baseline-eval summary                   # the report's tables from the saved runs
```

Or in one command, without the `export` and `spawn`: `scenarios/baseline_eval.sh` takes the same arguments (`intercept`, `detect`, `summary`), starts the drones that are not running and waits for them. It also runs without a terminal, so `nohup` works as well as `tmux`.

A short trial instead: `scenarios/trial.sh` flies 3 head-on runs at 4 m/s (`scenarios/trial.sh 5` for 5; baseline_eval's arguments after the number change the settings), lands the defenders and prints the metrics of just those runs. Each trial is saved in its own folder, `data/trials/<date-time>/`.

Drone 4 flies straight in by default, as in the planning report. `--path weave` (a sine side to side, `--path-amplitude` m either way, default 15, over `--path-wavelength` m, default 60) or `--path zigzag` (the same in straight legs with sharp turns) makes it fly a curved way in; both `baseline_eval.sh` and `trial.sh` take them. It starts along the straight line and ends at the base, so the scenario's aspect still holds at the start, but not after the first swing. Runs that are not straight save to their own default folder (`data/baseline_eval_weave/`) and record `path`, `path_amplitude` and `path_wavelength`. In the fast simulation, weave and zigzag took the hit rate at 4 m/s head-on from 3/3 to 0/3: the defender loses the target from its camera at each swing, gives up and returns.

Each run is appended to `data/baseline_eval/intercept.jsonl` or `detect.jsonl`, and runs already there are skipped. So Ctrl-C (the defenders land) and starting again picks up where it stopped. `--rerun` flies saved runs again. After a batch the tables are printed and written to `data/baseline_eval/summary.txt`. Don't run `detect`, `attacktest` or the patrol and base-station commands alongside it.

## What it does

| Mode | Scenarios (seeded as in the harness) | Metrics (the report's) |
|---|---|---|
| `intercept` | Aspect 0/30/60/90° × hostile speed 4/8/12/14/17 m/s × seeds 0–29. Each seed draws a bearing (which picks the defender the scenario is built around), a side and the hostile's altitude (20–120 m). The track goes straight to the base and makes the aspect with the line of sight to that defender when it is 51 m away. It starts 60 m further out, so the real detector finds it. The base station sends whichever defender it chooses | hit, success (hit more than 100 m from the base), time to hit, lock-on (held, time to, longest), distance to base at hit, the target's closest distance to base, closest miss; outcome hit / reached base / lost / timeout |
| `detect` | Seeds 0–29. Each seed draws the hostile's altitude (20–120 m) and its direction, squeezed onto the arc's 90°. It flies at 17 m/s from 450 m out until it reaches the arc. The defenders only patrol and report: the base station sends nobody. No birds | missed drones (reached the arc undetected), first detection range, warning distance and time, detection probability vs range, background false alarms per image |

Every run, the hostile drone is restarted at its start point, climbs, and flies its track. A hit shoots it down as in `attacktest`, and the defender flies back to its slot. The next run starts when all defenders are back on patrol.

**What is scored as what:**
- **The interceptor** in the intercept metrics is the defender that hit. Without a hit, it's the first one sent; if nobody was sent, the one the scenario was built around.
- **"In view"** means the hostile drone projects into that defender's camera image. The truth camera pose comes from Gazebo, at the image's time.
- **"Detected"** means the defender's detector has a confirmed track within 25 px of that projected point, in its latest image (at most 0.3 s old).
- **Detection metrics** use every image any detector processed (`detect/live.py --record`). A confirmed track on an image with nothing in it is a background false alarm. Another defender in view counts as a drone, not a false alarm.

## Changes to fit the report's values

| Setting | Was | Now | Why |
|---|---|---|---|
| `surveillance.arc_radius` | 60 m | 150 m | The report's goal radius is 100 m: the arc must be outside it. 150 m is also just outside the forest |
| `surveillance.leash_m` | 200 m | 400 m | The leash must reach past the arc |
| attack speed in the evaluation | 5 m/s | 12 m/s | The report's interceptor speed (`intercept.interceptor_max_speed`) |
| `detect.match_radius_px` | 3 px | 25 px | Real images: the box centre and PX4's camera orientation put the truth up to ~2° (~20 px) off |
| hostile's `MPC_XY_VEL_MAX` | 12 m/s | speed + 3 | PX4 caps a drone at 12 m/s; the report's speeds go to 17 m/s |
| detection start | 1000 m outside the ring | 450 m from the base | hive_base's ground plane is 1000 × 1000 m |

Everything else is the harness's: seeds, aspects, speeds, altitudes, goal radius, hit radius 1.5 m, lock-on rule (3 of 5 images, within 20 m, for 3 s), detect range 51 m and the timeouts.

## Known limits

- **Hostile altitudes 20–25 m can't be hit.** Attacks never go below 25 m (`attack.min_altitude`, because the trees reach 20 m). Hostiles above ~75 m are above the cameras' view when close (camera pitched 10° up, ±41° vertical view). Both are part of what the baseline measures.
- **A hostile at 20 m flies through the treetops** on its way in. A run where it crashes into a tree ends as a timeout or reached-base, and should be checked.
- **The run doesn't end when the target is lost** (the harness's loop does). The base station can send a defender again, so the run goes on until a hit, the base or the timeout.
- **The full intercept grid is 600 runs** of a few minutes each. Run subsets (`--aspects`, `--speeds`, `--seeds`) and let it resume.
- **Not flown yet.** Written and unit-tested (`tests/test_arc_eval.py`), but no Gazebo run so far.

## Code

| File | What |
|---|---|
| `swarm_backend/scenarios/baseline_eval.py` | The scenario (`intercept`, `detect`, `summary`; `--sim` for the fast simulation, into `data/baseline_eval_sim/`); flies in `SurveillanceWorld` (`scenarios/surveillance.py`), shared with the attack scenario |
| `swarm_backend/intercept/arc_eval.py` | The harness's scenarios placed on the arc; recordings → the harness's `Sample`s and `FrameRecord`s |
| `swarm_backend/detect/live.py --record` | One JSON line per processed image (boxes, confirmed tracks) |
| `scenarios/surveillance.yaml` | The arc, and the `detect:` setting above |
| `scenarios/compare_paths.sh` | N runs (default 3) for each of straight, weave ±5 m, weave ±15 m and zigzag ±15 m, and drone 4 10 m below, level with and 10 m above the defenders (they fly at `surveillance.altitude`, 40 m: drone 4 at 30, 40, 50 m; 36 runs). Drone 4 starts at a seeded bearing in drone 2's 30° sector (`--sector 2`; the same seed gives the same start on every path and altitude). Comparison table per path and altitude, with the first detection's 3-D distance (`data/path_compare/<date-time>/`) |
| `baseline_eval.sh intercept --altitude-offsets -10 0 10 --sector 2` | The options on their own (`--altitudes 30 40 50` gives absolute heights). Every run also saves `detected_by`, `detect_t`, `detect_distance` (3-D, defender to drone 4), `detect_horizontal` and `detect_height` |
| `scenarios/trial.sh` | A few runs, then land and print their metrics (own folder per trial) |
| `scenarios/baseline_eval.sh`, `scenarios/lib.sh` | The one-command launcher, and what launchers share: checks, waiting for the drones, running in the container |
