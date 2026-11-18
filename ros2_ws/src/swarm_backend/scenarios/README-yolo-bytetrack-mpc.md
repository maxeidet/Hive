# YOLO + ByteTrack + MPC 36-case comparison: how to run

Step by step: fly the improved setup (the **YOLO** detector, **ByteTrack** locked on one target, and the **MPC** intercept) on the 36 test cases of `scenarios/compare_paths.sh`, in Gazebo. About **3 hours**. The same 36 starts as [Baseline 36-case comparison - how to run](README.md), so the two can be compared run by run.

The evaluation itself (scenarios, metrics, known limits): [Baseline evaluation - how to run](../../../../hive-vault/Baseline%20evaluation%20-%20how%20to%20run.md). A copy of the vault note [YOLO-ByteTrack-MPC 36-case comparison - how to run](../../../../hive-vault/YOLO-ByteTrack-MPC%2036-case%20comparison%20-%20how%20to%20run.md): change both.

---

## What the 36 cases are

Drone 4 (the hostile) flies at the base at 4 m/s, against the three gimbal defenders on the surveillance arc (`scenarios/surveillance.yaml`):

| | Values | |
|---|---|---|
| Path | straight, weave ±5 m, weave ±15 m, zigzag ±15 m | 4 |
| Altitude of drone 4 | 10 m below, level with, 10 m above the defenders (30 / 40 / 50 m) | × 3 |
| Seed | 0, 1, 2: a start bearing inside drone 2's sector, the same on every path and altitude | × 3 |
| | | **= 36 runs** |

This setup is `compare_paths.sh`'s **default**, so no `--setup` is needed (`--setup yolo-bytetrack-mpc` says the same). What each setup selects is in `swarm_backend/scenarios/common.py` (`SETUPS`):

| `--setup` | Detector | Tracker | Attack |
|---|---|---|---|
| `yolo-bytetrack-mpc` (default) | YOLO, `--lock` (only the locked drone is published) | ByteTrack | MPC: a Kalman filter predicts the meeting point, the MPC steers the last 5 s |
| `baseline` | baseline (blob) detector | Confirmer | pure pursuit |

---

## Step by step

### 1. Check the machine (Stellar rules)

YOLO runs on the GPU (onnxruntime-gpu in the container on x86), so this matters more than for the baseline.

```bash
cat /etc/stellar/booking.json   # booked by someone else? Ask before starting
nvidia-smi                      # another user's process on the GPU? Ask before starting
df -h / /scratch
```

### 2. Get the YOLO weights (once)

The weights are not in git: they are GitHub Releases of the repo, and the detector loads the **newest** `.onnx` under `models/` in the repo (`models/` is git-ignored). The team's live tests use `yolo26n-gazebo-only-1280.onnx`:

```bash
mkdir -p ~/Desktop/Hive/models/yolo26n-gazebo-only
curl -L -o ~/Desktop/Hive/models/yolo26n-gazebo-only/yolo26n-gazebo-only-1280.onnx \
  https://github.com/maxeidet/Hive/releases/download/model-gazebo-only/yolo26n-gazebo-only-1280.onnx
```

About 9 MB. The other releases:

| Release | File | Trained on |
|---|---|---|
| `model-gazebo-only` | `yolo26n-gazebo-only-1280.onnx` | Gazebo images only |
| `model-dvb-gazebo-v1` | `yolo26n-dvb-gazebo-1280.onnx` | real Drone-vs-Bird images, then fine-tuned on Gazebo |
| `model-dvb-v1` | `yolo26n-dvb-1280.onnx` | real Drone-vs-Bird images only |

With more than one `.onnx` in `models/`, the newest file wins. To choose one, pass it in step 7 (it replaces the setup's detector options, so repeat them):

```bash
--detect-args "--detector yolo --lock --weights /workspace/models/yolo26n-gazebo-only/yolo26n-gazebo-only-1280.onnx"
```

`/workspace` is the repo inside the container. Background: [Training data and YOLO training](../../../../hive-vault/detection/Training%20data%20and%20YOLO%20training.md), `training/README.md`.

### 3. Terminal 1: Gazebo and drone 0

```bash
cd ~/Desktop/Hive
docker/start-stellar.sh         # wait for the pxh> prompt, then leave this terminal alone
```

### 4. Terminal 2: a tmux session

The run takes hours; tmux keeps it alive if the terminal closes.

```bash
tmux new -s yolo-mpc
cd ~/Desktop/Hive/ros2_ws/src/swarm_backend
```

Detach with `Ctrl-b` then `d`; come back with `tmux attach -t yolo-mpc`.

### 5. Optional: a dry run in the fast simulation (a few minutes, no Gazebo)

```bash
scenarios/compare_paths.sh 3 --sim
```

Checks the paths, the MPC and the scoring before spending 3 hours. The fast simulation uses a fake detector, **not YOLO**, so it does not test the weights; step 7's first minutes do (look for the `YOLO: ...` line below).

### 6. Optional: record which code runs

For the code clean-up (`tools/runtime_coverage/`): records which `swarm_backend` code runs in the container.

```bash
tools/runtime_coverage/runtime_coverage.sh on
```

### 7. Fly the 36 cases

```bash
scenarios/compare_paths.sh 3
```

It spawns the defenders (1–3) and drone 4 if they are not running, waits for them, then flies the paths one after the other; the defenders land after each path. No `export SWARM_CONFIG` or `spawn` is needed: the script does it.

- **Check that YOLO loaded:** each detector prints `YOLO: <path> (onnxruntime GPU, ...)`. `onnxruntime CPU` means CUDA did not start (it works, but slower). `No YOLO weights at ...` means step 2 is missing.
- **Ctrl-C** stops the path that is flying (the defenders land) and skips the rest. The comparison is printed for what was flown.
- A path that fails (e.g. a drone that does not start) is marked in the comparison, and the next one is flown.
- Don't run other scenarios, `attacktest` or the swarm shell at the same time: they would fight over the drones.

### 8. Optional: watch it

In another terminal, from `ros2_ws/src/swarm_backend/`:

```bash
./mount-backend.sh view 2       # RViz: drone 2's camera

export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh ui           # operator UI, then open http://localhost:8090
```

`SWARM_CONFIG` is a path **inside the container**, and `export` only lasts in that terminal. Without it the UI reads `config/swarm.yaml` and does not show the surveillance arc's drones. RViz and the UI only open if the container was not started with `--headless`.

### 9. Results

At the end, a comparison per path and altitude is printed: hits, the median closest pass, and the first detection (how far, 3-D, the first defender that confirmed drone 4 was from it). Everything is saved in the repo:

```
data/path_compare/<date-time>_yolo-bytetrack-mpc/
    compare.txt                         the comparison
    <path>/intercept.jsonl              one line per run
    <path>/summary.txt                  the report's metrics for that path
```

To compare with the baseline, put its `compare.txt` (`data/path_compare/<date-time>_baseline/`) next to this one: same seeds, same starts.

### 10. Afterwards

```bash
tools/runtime_coverage/runtime_coverage.sh off          # if you did step 6
tools/runtime_coverage/runtime_coverage.sh report swarm_backend/intercept
```

Then type `shutdown` at `pxh>` in terminal 1: the simulation stops and the container is removed.

---

## Variations

| Command | What |
|---|---|
| `scenarios/compare_paths.sh 5` | 5 seeds per case (0–4): 60 runs |
| `scenarios/compare_paths.sh 3 --seeds 10-12` | other start bearings |
| `scenarios/compare_paths.sh 3 --altitude-offsets 0` | only level with the defenders: 12 runs |
| `scenarios/compare_paths.sh 3 --guidance pursuit` | YOLO + ByteTrack, but pure pursuit instead of the MPC (saved in its own folder) |
| `scenarios/compare_paths.sh 3 --tracker confirmer` | YOLO + MPC, but the Confirmer instead of ByteTrack |
| `scenarios/compare_paths.sh 3 --detect-args "--detector yolo --lock --max-lost 10"` | more ByteTrack / lock options (repeat `--detector yolo --lock`) |
| `scenarios/compare_paths.sh 3 --setup baseline` | the baseline on the same 36 starts: [Baseline 36-case comparison - how to run](README.md) |
| `scenarios/compare_paths.sh --help` | all options |

## Known limits

- The baseline evaluation, which this flies, had not been flown in Gazebo when it was written ([Baseline evaluation - how to run](../../../../hive-vault/Baseline%20evaluation%20-%20how%20to%20run.md)). Check the first runs by eye.
- Attacks never go below 25 m (`attack.min_altitude`, the trees reach 20 m).
- YOLO's thresholds (ByteTrack high 0.3, low 0.1, new 0.4) come from [Threshold tuning 2026-11-11](../../../../hive-vault/detection/Threshold%20tuning%202026-11-11.md), tuned on scenario 2, not on these paths.
