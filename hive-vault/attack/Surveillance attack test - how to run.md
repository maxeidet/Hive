# Surveillance attack test: how to run

`./mount-backend.sh attacktest` flies the whole chain since the surveillance branch was merged into `dev` (3 Nov 2026). Three defenders patrol the arc, a hostile drone flies in toward the base, and the base station sends one defender, which attacks on its real camera detector. On a hit the hostile drone is shot down (taken out of the battle) and the defender flies back to its slot. This replaces the one-on-one test in [[Baseline attack - how to run]].

Back to [[Home]]. The design: [[Base surveillance plan]]. The attack each defender flies: [[Baseline attack work plan]].

---

## Run it

The test needs the three gimbal defenders 1–3 and the hostile drone 4 from `scenarios/surveillance.yaml`. Drone 0, which the start script starts, only stays parked at the base: its PX4 is the container's own process, so stopping it stops the container.

Terminal 1, from the repo root (Gazebo and drone 0; wait for `pxh>` and leave it open):

```bash
docker/start-stellar.sh           # or docker/start-dev.sh
```

Terminal 2, from `ros2_ws/src/swarm_backend/`:

```bash
export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh spawn          # adds defenders 1-3 and hostile 4; ~15 s to connect
./mount-backend.sh attacktest --fake-detector   # first check
./mount-backend.sh attacktest                   # real detectors
```

Use `spawn`, not `restart`, after starting the container: `restart` stops Gazebo and drone 0 and starts them again in the background. `restart 4` is only for resetting a running sim (after a crash, or when drones 1–3 run as other models). `attacktest` uses `surveillance.yaml` even without the `export`, but `spawn` and `restart` need it. Don't run `detect`, `fakedetect`, `patrol` or `basestation` alongside it: the test starts its own.

The test never touches drone 0, so the `pxh>` shell in terminal 1 stays.

| Command | What it does |
|---|---|
| `./mount-backend.sh attacktest` | One run: the hostile drone comes straight in from the threat heading (through the middle slot), 150 m out, 10 m above the defenders, at 1 m/s. Real detectors, gimbal gain 0.5 |
| `./mount-backend.sh attacktest --bearing -30 0 30 --runs 3` | One run through each slot, in turn |
| `./mount-backend.sh attacktest --runs 5 --bearing 15` | Five runs between the middle and right slots |
| `./mount-backend.sh attacktest --fake-detector` | Fake detectors from Gazebo's poses instead of the camera (`--realistic` adds delay, noise and the blind zone) |
| `./mount-backend.sh attacktest --gain 1.0 --detect-rate 10` | Gimbal gain 1.0, and the real detectors at 10 instead of 20 images per second |

`--help` lists everything (`--start`, `--height`, `--speed`, `--hit-radius`, `--max-speed`, `--csv`).

## What happens

1. **Start.** The test starts a patrol for each defender, the base station, and a real detector for each defender (`detect ID --rate 20`, logs in `/tmp/detect_<id>.log` in the container). The defenders take off to 30 m (the trees reach 20 m; attacks never go below 25 m, `attack.min_altitude`) and fly to their slots on the arc (−30°, 0°, +30° on a 60 m arc), sweeping their cameras.
2. **Each run.** The hostile drone 4 is stopped and started again (a fresh PX4) on the ground at the run's start point. It climbs and then flies straight at the base. The base station sends the defender that sees it first.
3. **The end of a run:**
   - **hit:** a defender within 1.5 m of the hostile drone, by Gazebo's true positions. The hostile drone is shot down: its model is moved at once to a graveyard on the ground at Gazebo (450, −450), far outside the forest and every camera's range, and then its PX4 is stopped (the other way round it fell through the defenders' view for seconds and they chased it into the trees, 3 Nov). The base station gets a `kill` event, and the defender, which survives, gets RETURN and flies back to its slot. The other two widen their sweeps to 45° while it is away and go back to 30° 10 s after it returns (`resweep_delay_s`).
   - **reached_base:** the hostile drone got to the base. It goes to the graveyard too.

   Why not remove the model from Gazebo: Gazebo keeps a removed model's sensors publishing, so a drone spawned again under the same name gets every IMU and barometer sample twice, its estimator never starts and it cannot arm (seen 3 Nov). The test therefore keeps one hostile model, moves it, and starts a fresh PX4 attached to it (`PX4_GZ_MODEL_NAME`).
4. The next run starts when every defender is back on patrol with its sector.
5. **At the end** the defenders land at their slots and drone 4 is started again at its spawn point (150 m north).

Results: one row per run in `data/surveillance_attack_runs.csv`. Columns include the result, which defender killed it and how far from the base, the time from the first sighting to the hit, who saw and engaged it first, the closest any defender got, and how long the killer took to get back on patrol.

## What changed in the code for this

| Where | Change |
|---|---|
| `swarm_backend/scenarios/attack.py` (was `tests/attack_test.py`; `./mount-backend.sh scenario attack [--sim]`) | Rewritten: the whole chain (patrols, base station, detectors, referee) in one process, on Gazebo's clock. The one-on-one patterns (`crossing`, `headon`, `circle`, `offset`) are gone |
| `intercept/attack_scenario.py` | `Inbound` / `inbound_path`: from `start_m` out, `bearing_deg` off the threat heading, straight at the base |
| `intercept/base_station.py` | `on_kill`: the target is gone, and the attacker survives and returns. `on_hit` (attacker lost) stays for a defender that goes silent, and for `surveillance_sim.py` |
| `nodes/base_station_node.py` | Handles the new `kill` event. `contact` (the detector saw the target within 3 m) is now only logged, no longer counted as a hit that loses the attacker. Takes a clock |
| `nodes/patrol_node.py` | Takes a clock. The test gives Gazebo's, because PX4's time stamps jump 6–7 s every ~18 s (seen on 3 Nov) and would trip the attack's coast/lost timers |
| `px4/px4_drone.py` | `reset(home)` for a drone whose PX4 was restarted somewhere else |
| `mount-backend.sh` | `attacktest` defaults to `scenarios/surveillance.yaml` |
| `scenarios/surveillance.yaml` | The hostile is drone 4 (`attack.target_id: 4`), spawned 150 m north; the defenders start 4 m south of the base, clear of the tents |
| `intercept/patrol.py`, `nodes/patrol_node.py` | Detections count only if under 1 s old (the detectors run during take-off), and detections within 8 m of another defender (from the positions in their status) are dropped, so defenders never attack each other |

## Open points

- The standalone surveillance run (`scenarios/surveillance.sh`) has no referee, so nothing sends `kill`. An attacker there returns once it loses the target.
- `surveillance_sim.py` (the pure-Python evaluation) still counts the attacker as lost on a hit. Change it to match if the defender is meant to survive there too.
- Three gimbal cameras and three detectors at 20 images per second slow Gazebo down. Lower `--detect-rate` if the runs crawl.
