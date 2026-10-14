# Baseline attack: how to run

The baseline attack from the work plan ("Baseline attack: work plan", Max's tasks 1-6): the defender flies into the hostile drone with pure pursuit, steered only by the two detection topics. Until the camera detector publishes them, a fake detector makes them from Gazebo's true positions. Built 14 Oct 2026 on `feat-intercept-baseline`.

Back to [[Home]].

---

## Run it

Start the container and drone 0 as usual, then from `ros2_ws/src/swarm_backend/` on the host:

| Command | What it does |
|---|---|
| `./mount-backend.sh spawn` | Drones 1 and 2 from `swarm.yaml`. Drone 0 is the hostile drone, drone 2 (gimbal) the defender |
| `./mount-backend.sh attacktest` | The whole test: both take off, the hostile drone flies across in front of the defender, 10 m above it, at 4 m/s, and the defender attacks. Runs the fake detector itself |
| `./mount-backend.sh attacktest --pattern headon --runs 10` | Ten head-on runs in a row, one CSV row each |
| `./mount-backend.sh attacktest --realistic --runs 10` | The same with the real detector's errors: 0.25 s delay, 1° and 20 % noise, blind below 5 m |
| `./mount-backend.sh attacktest --gain 1.0` | Gimbal gain 1.0 instead of the plan's 0.5 |
| `./mount-backend.sh attacktest --external-detector` | No fake: Oskar's detector (or `fakedetect`) publishes the topics |
| `./mount-backend.sh fakedetect [--realistic]` | Only the fake detector, for use with `attack` or the shell |
| `./mount-backend.sh attack` | Only the attack node: takes off and attacks whatever the topics report, lands on Ctrl-C |

Patterns: `crossing` (default: 35 m ahead, from 60 m left to 60 m right), `headon` (from 80 m ahead, over the defender), `circle` (radius 20 m, centre 30 m ahead). `--speed`, `--height`, `--ahead`, `--half-width`, `--timeout` change them; `--help` lists everything.

Each run ends with **hit** (within 1.5 m, from Gazebo's true positions, also between samples), **target_passed** (the hostile drone reached the end of its line) or **timeout**. The drones then move apart and go back to the start for the next run. Results: one row per run in `data/attack_runs.csv` (hit, time from first detection to hit, closest distance, time in each state, end reason, settings).

A hit is a real collision at up to 12 m/s. The drones can tumble in Gazebo; the test still tries to land them. If one has crashed, restart the simulation.

## The contract

| Topic | Type | Content |
|---|---|---|
| `/px4_2/detection/target_position` | `geometry_msgs/PointStamped` | Target relative to the drone, world NED, metres |
| `/px4_2/detection/target_offset` | `geometry_msgs/Vector3Stamped` | x right, y up, radians from the image centre, roll corrected |

Only for a confirmed target, stamped with the image time. Code: `intercept/detection.py`.

## What is where

| File | What |
|---|---|
| `intercept/detection.py` | Topic names, the fake detector's math (field of view, range, blind zone, noise, delay, k-in-a-row) |
| `intercept/attack.py` | The attack: SEARCH, ATTACK, COAST, LOST, HIT; pure pursuit; gimbal on the offset; limits |
| `intercept/attack_scenario.py` | The test's hostile paths and hit scoring |
| `nodes/fake_detection.py` | Fake detector node on Gazebo's poses |
| `nodes/attack_node.py` | Attack node on a PX4 drone |
| `tests/attack_test.py` | The flight test (tasks 4-6) |
| `tests/test_detection.py`, `test_attack.py`, `test_attack_scenario.py` | Unit tests without ROS, including closed-loop runs against a simulated drone |

Settings: the `attack:` section of `config/swarm.yaml`.

## Limits (task 3)

Every velocity command is scaled down, direction kept, to PX4's limits (12 m/s across, 3 m/s up, 1.5 m/s down), and descent stops at `min_altitude` (5 m), also while coasting. Keeping the direction matters: clipping only the vertical speed, as PX4 would on its own, missed head-on targets by 6-7 m in the simulated runs instead of 2.

## What the simulated runs showed (not Gazebo yet)

From the unit tests' simulated drone (12 m/s, 4 m/s², PX4's vertical limits, fake detector without errors):

- **Head-on, 10 m above:** hits.
- **Head-on, 15 m above:** falls short by about 2 m. The 3 m/s climb limit is too slow to get up to the target in time. That is why the test's default height is 10 m.
- **Crossing with the plan's gimbal gain 0.5:** the first pass misses by about 3 m. Close in, the target moves fast across the image, the gimbal (half the error per image, 4 images per second) falls behind, the target leaves the view and the drone coasts past. With gain 1.0 it hits.
- **Hostile at 17 m/s flying away:** never caught, as the report predicts for pure pursuit.

Open for the team: gimbal gain 0.5 or 1.0 (check with `--realistic`), how far above the defender the hostile drone should fly, and the search direction.
