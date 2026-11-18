# Baseline 36-case comparison: how to run

Step by step: fly the baseline (the planning report's setup: the **baseline blob detector**, the **Confirmer** tracker and **pure pursuit**) on the 36 test cases of `scenarios/compare_paths.sh`, in Gazebo. About **3 hours**.

The evaluation itself (scenarios, metrics, known limits): [Baseline evaluation - how to run](../../../../hive-vault/Baseline%20evaluation%20-%20how%20to%20run.md). A copy of the vault note [Baseline 36-case comparison - how to run](../../../../hive-vault/Baseline%2036-case%20comparison%20-%20how%20to%20run.md): change both.

---

## What the 36 cases are

Drone 4 (the hostile) flies at the base at 4 m/s, against the three gimbal defenders on the surveillance arc (`scenarios/surveillance.yaml`):

| | Values | |
|---|---|---|
| Path | straight, weave ±5 m, weave ±15 m, zigzag ±15 m | 4 |
| Altitude of drone 4 | 10 m below, level with, 10 m above the defenders (30 / 40 / 50 m) | × 3 |
| Seed | 0, 1, 2: a start bearing inside drone 2's sector, the same on every path and altitude | × 3 |
| | | **= 36 runs** |

**Use `--setup baseline`.** Without it, `compare_paths.sh` flies YOLO + ByteTrack + MPC (the default setup, `yolo-bytetrack-mpc`), not the baseline. What each setup selects is in `swarm_backend/scenarios/common.py` (`SETUPS`):

| `--setup` | Detector | Tracker | Attack |
|---|---|---|---|
| `baseline` | baseline (blob) detector | Confirmer | pure pursuit |
| `yolo-bytetrack-mpc` (default) | YOLO, `--lock` | ByteTrack | MPC |

---

## Step by step

### 1. Check the machine (Stellar rules)

```bash
cat /etc/stellar/booking.json   # booked by someone else? Ask before starting
nvidia-smi                      # another user's process on the GPU? Use --no-gpu in step 2
df -h / /scratch
```

### 2. Terminal 1: Gazebo and drone 0

```bash
cd ~/Desktop/Hive
docker/start-stellar.sh         # wait for the pxh> prompt, then leave this terminal alone
```

### 3. Terminal 2: a tmux session

The run takes hours; tmux keeps it alive if the terminal closes.

```bash
tmux new -s baseline
cd ~/Desktop/Hive/ros2_ws/src/swarm_backend
```

Detach with `Ctrl-b` then `d`; come back with `tmux attach -t baseline`.

### 4. Optional: a dry run in the fast simulation (a few minutes, no Gazebo)

```bash
scenarios/compare_paths.sh 3 --setup baseline --sim
```

Checks that everything works before spending 3 hours. The fast simulation has simpler drones and a fake detector, so its numbers are not the Gazebo ones.

### 5. Optional: record which code runs

For the code clean-up (`tools/runtime_coverage/`): records which `swarm_backend` code runs in the container.

```bash
tools/runtime_coverage/runtime_coverage.sh on
```

### 6. Fly the 36 cases

```bash
scenarios/compare_paths.sh 3 --setup baseline
```

It spawns the defenders (1–3) and drone 4 if they are not running, waits for them, then flies the paths one after the other; the defenders land after each path. No `export SWARM_CONFIG` or `spawn` is needed: the script does it.

- **Ctrl-C** stops the path that is flying (the defenders land) and skips the rest. The comparison is printed for what was flown.
- A path that fails (e.g. a drone that does not start) is marked in the comparison, and the next one is flown.
- Don't run other scenarios, `attacktest` or the swarm shell at the same time: they would fight over the drones.

### 7. Optional: watch it

In another terminal, from `ros2_ws/src/swarm_backend/`:

```bash
./mount-backend.sh view 2       # RViz: drone 2's camera

export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh ui           # operator UI, then open http://localhost:8090
```

`SWARM_CONFIG` is a path **inside the container** (`/workspace` is the repo), and `export` only lasts in that terminal. Without it the UI reads `config/swarm.yaml` and does not show the surveillance arc's drones. RViz and the UI only open if the container was not started with `--headless`.

### 8. Results

At the end, a comparison per path and altitude is printed: hits, the median closest pass, and the first detection (how far, 3-D, the first defender that confirmed drone 4 was from it). Everything is saved in the repo:

```
data/path_compare/<date-time>_baseline/
    compare.txt                         the comparison
    <path>/intercept.jsonl              one line per run
    <path>/summary.txt                  the report's metrics for that path
```

### 9. Afterwards

```bash
tools/runtime_coverage/runtime_coverage.sh off          # if you did step 5
tools/runtime_coverage/runtime_coverage.sh report swarm_backend/intercept
```

Then type `shutdown` at `pxh>` in terminal 1: the simulation stops and the container is removed.

---

## Variations

| Command | What |
|---|---|
| `scenarios/compare_paths.sh 5 --setup baseline` | 5 seeds per case (0–4): 60 runs |
| `scenarios/compare_paths.sh 3 --setup baseline --seeds 10-12` | other start bearings |
| `scenarios/compare_paths.sh 3 --setup baseline --altitude-offsets 0` | only level with the defenders: 12 runs |
| `scenarios/compare_paths.sh 3` | the same 36 starts with YOLO + ByteTrack + MPC, to compare with the baseline: [YOLO-ByteTrack-MPC 36-case comparison - how to run](README-yolo-bytetrack-mpc.md) (needs the YOLO weights) |
| `scenarios/compare_paths.sh --help` | all options |

## Known limits

From [Baseline evaluation - how to run](../../../../hive-vault/Baseline%20evaluation%20-%20how%20to%20run.md):

- The baseline evaluation had not been flown in Gazebo when it was written. Check the first runs by eye.
- Attacks never go below 25 m (`attack.min_altitude`, the trees reach 20 m). Hostiles well above the defenders can be above the cameras' view when close.
- In the fast simulation, weave and zigzag took the hit rate at 4 m/s head-on from 3/3 to 0/3: the defender loses the target from its camera at each swing, gives up and returns.
