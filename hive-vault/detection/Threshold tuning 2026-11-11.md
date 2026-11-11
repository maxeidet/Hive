# Threshold tuning 2026-11-11

Tuning the YOLO detector, ByteTrack and `--lock` thresholds on recorded scenario 2 flights. **Status: all 6 runs done, replayed.** The proposed values are at the end. **Applied 11 Nov:** `TRACK_SCORES['yolo']` in `live.py` is now high 0.3, low 0.1, new 0.4.

Back to [[Home]]. Related: [[Tutorial - live detector (detect)]], [[Following the target with detection]].

## How

- `scenarios/record_tuning.sh` flies scenario 2 (YOLO + ByteTrack + `--lock`) and records every YOLO box down to conf 0.05, together with Gazebo's truth.
- `detect/replay_eval.py` labels each box with the hostile it lies on, using the projected truth position. Boxes on drone 0, parked on the base, count as "ours", apart from the false boxes. It then replays ByteTrack and the lock offline over a grid: 549 tracker settings, each with no lock and with lock timeouts of 0.5, 1 and 2 s.
- Data is in `data/tune/nov11-*`. The report is `runs/tune/nov11/report2.txt` and every combination is in `all2.csv`.
- Caveat: the replay is open-loop. Locking on earlier would change how the defender flies, so the chosen values still need a real Gazebo run.

## The flights

| Run | Setup | Result |
|---|---|---|
| watch1 | watch only, 5 m/s | hostile 2 tracked from 67 m |
| watch2 | watch only, 7 m/s, 12 s stagger | hostile 2 tracked from 36 m |
| pursuit1 | pursuit, 5 m/s | **both hit** (t = 33.5 s, 39.6 s) |
| pursuit2 | pursuit, hostiles at 8 m/s | **both hit** (t = 19.6 s, 29.9 s) |
| mpc1 | MPC, 5 m/s | **both hit** (t = 26.5 s, 42.8 s). First report at 55 m, 0 switches |
| mpc2 | MPC, 12 s stagger | **both hit** (t = 47.9 s, 64.4 s). First report at 52 m, 0 switches |

All four attack runs hit both hostiles: 8 hits out of 8. In the watch-only runs hostile 3 is hardly reported. That is the lock staying on hostile 2 as designed, not a detection miss.

## Detector (YOLO)

Share of images where a hostile is in view and YOLO has a box on it at or above `conf` (6 runs, 4124 images). The last column is false boxes per image, not counting drone 0:

| conf | 0–20 m | 20–40 m | 40–60 m | 60–80 m | 80–100 m | 100+ m | false/img |
|---|---|---|---|---|---|---|---|
| 0.05 | 96% | 94% | 79% | 47% | 12% | ≤2% | 0.33 |
| 0.1 | 96% | 94% | 71% | 33% | 5% | ≤1% | 0.16 |
| 0.3 | 95% | 93% | 49% | 13% | 0% | 0% | 0.07 |
| 0.5 | 92% | 90% | 23% | 3% | 0% | 0% | 0.03 |
| 0.6 | 91% | 86% | 12% | 0% | 0% | 0% | 0.01 |
| median score of the real drone | 0.90 | 0.85 | 0.39 | 0.18 | 0.09 | 0.07 | |

- YOLO is reliable to about 40 m, partial at 40–80 m and blind past about 100 m. Starting the hostiles at 150 m wastes most of a run, and about 100 m is enough.
- With `new` = 0.6, tracks can hardly start before about 45 m, because the median score at 40–60 m is only 0.39.

## Tracker and lock (replay)

Ranked so that settings switching target more than 2 times a minute come last, then by coverage. Coverage includes the far images where YOLO sees nothing, so about 55% is close to the ceiling.

| low | high | new | k | max-lost | lock | coverage | false | ours | switches/min | first track |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.1 | 0.5 | 0.6 | 3 | 30 | off | 53% | 2.3% | 8.2% | 40 | 49 m |
| 0.1 | 0.5 | 0.6 | 3 | 30 | 1 s | 50% | 2.9% | 5.1% | 0.44 | 49 m |
| 0.1 | 0.3 | 0.4 | 3 | 30 | 1 s | 53% | 4.0% | 4.4% | 0.44 | 62 m |
| 0.05 | 0.3 | 0.4 | 3 | 30 | 1 s | 55% | 5.1% | 3.5% | 0.44 | 67 m |
| 0.05 | 0.3 | 0.4 | 3 | 30 | 0.5 s | 56% | 5.1% | 5.3% | 0.73 | 67 m |

The first two rows are today's defaults (lock off and lock on). The 0.05 row with a 1 s lock is the replay's best with the 1 s lock.

- **Keep the lock.** Without it the target jumps about 40 times a minute. With it, 0.4 times.
- **`high`/`new` 0.3/0.4** is the biggest win. The first track starts 13–18 m earlier (62–67 m instead of 49 m), and coverage goes up 3–5 points.
- **`low`/`conf` 0.05 against 0.1:** 0.05 tracks 5 m further and gains 2 points of coverage, but has about 1 point more false targets and twice the false boxes. It is a small gain either way.
- **Lock timeout 1 s** rather than 0.5 s: almost the same coverage, but fewer switches and fewer picks of drone 0. 2 s is no better.
- **`k` 3 and `max-lost` 30** stay as they are. `k` 2 switches more, and a lower `max-lost` mostly shortens the first-track distance.

## False targets: drone 0 and trees

- **Drone 0, parked on the base**, is chosen as the target in 3–8% of published images. That is a real drone, but our own, and **the lock does not tell friend from foe**: the defender can pick drone 0 as its target. **Fix needed in `live.py`:** ignore boxes that point at the known positions of our own drones.
- **A group of trees about 85 m to the north-east** (N 55, E 65) gets a few boxes in every run. This is a real YOLO false positive.

## Proposed thresholds (applied)

| Setting | Before | Now |
|---|---|---|
| `--conf` (YOLO, = low) | 0.1 | 0.1 |
| `--low` | 0.1 | 0.1 |
| `--high` | 0.5 | **0.3** |
| `--new` | 0.6 | **0.4** |
| `--k` | 3 | 3 |
| `--max-lost` | 30 | 30 |
| `--lock` | on in `bytetrack.sh` | on |
| `--lock-timeout` | 1 s | 1 s |

`low` 0.05 is the alternative. It is slightly better in the replay but has more false boxes, so it is worth one flight next to 0.1:

```
scenarios/mpc.sh                         the new defaults
scenarios/mpc.sh --detect-args --low 0.05
```

Next steps:

1. Fly these two settings in Gazebo.
2. Add a friendly-drone mask in `live.py`.
3. Start the hostiles at about 100 m in future recordings.
