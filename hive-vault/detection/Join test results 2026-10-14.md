# Join test results, 14 Oct 2026

The first runs of Max's baseline attack flying on Oskar's real camera detector (the "when we meet again" step of [[Baseline attack work plan]]), on `dev` after both branches were merged. **Result: the chain works end to end, and head-on gives a hit on the real detector; crossing still misses by about 4 m.**

Back to [[Home]]. How to run it: [[Manual tests to do]] ("The join"). Detector: [[Tutorial - live detector (detect)]].

---

## Setup

- `dev` at commit `27d27f8` (both branches merged), 163 unit tests passing.
- Light simulation: `scenarios/attack.yaml` (only drone 0 = hostile, drone 2 = defender with the gimbal camera), world `baylands`, on Oskar's Mac.
- Real detector: `./mount-backend.sh detect 2 --rate 20` (baseline, square black-hat window, camera orientation from PX4).
- Attack: `./mount-backend.sh attacktest --pattern … [--external-detector] [--gain …]`. Hostile at 4 m/s; a hit is within 1.5 m by Gazebo's true positions.

## Runs

One run per line, from `data/attack_runs.csv`. Times are simulation seconds from the start of the run.

| Time | Path | Detector | Gimbal gain | Result | First detection | Hit after | Closest | Messages | Attack / coast / lost |
|---|---|---|---|---|---|---|---|---|---|
| 21:10 | headon (10 m above) | **real** | 0.5 | **hit** | 4.7 s | 5.0 s | 1.09 m | 24 | 3.5 / 1.4 / 0 s |
| 21:01 | offset (same height) | fake | 0.5 | **hit** | 2.6 s | 4.0 s | 1.45 m | 23 | 4.0 / 0 / 0 s |
| 20:51 | crossing (10 m above) | real | 1.0 | passed | 4.3 s | – | 4.31 m | 41 | 7.3 / 6.3 / 7.2 s |
| 20:39 | crossing (10 m above) | real | 0.5 | passed | 3.5 s | – | 4.44 m | 50 | 7.2 / 6.8 / 6.5 s |
| 20:23 | offset (same height) | real | 0.5 | passed | 9.8 s | – | 6.19 m | 3 | 0.8 / 1.5 / 3.0 s |

"Passed" = the hostile drone reached the end of its line without a hit. "Messages" = target_position messages the attack received.

## What the runs show

- **The interface works.** In every real-detector run the attack switched from SEARCH to ATTACK on our topics; no code changes were needed to join the two halves.
- **Head-on hits on the real detector.** The hostile stays near the image centre and against the sky. The attack flew 3.5 s, then coasted the last 1.4 s through the blind zone under 5 m (the target too big for the detector), as the plan intended.
- **Crossing misses by about 4.4 m, with both gains.** About as much time is spent coasting or lost (6–7 s each) as attacking (7 s): the lock keeps dropping while the target moves across the view. Gain 1.0 instead of 0.5 made little difference (4.31 m against 4.44 m); one run each is too few to judge.
- **Offset is not a test for the baseline detector.** At the defender's own height the hostile is against the horizon and ground. In the 20:23 run (detector log checked) it was confirmed only at 11–14 m and 35–43° to the side of the picture, held for 2 images, and lost in the defender's first hard turn (camera roll swinging to −30°). With the fake detector the same path hits. Max's code also marks `offset` as for the fake detector only.
- **Detection range against the sky: about 68 m.** In the 20:23 log the detector tracked the hostile flying away from 9.8 m out to 68.5 m (3 px wide).

## Problems seen

| Problem | Seen in | Effect | Fix |
|---|---|---|---|
| False alarms on the ground during take-off | 20:23 detector log, 28–30 s: tracks confirmed low in the picture, 3–25 m *below* the drone | None in this run (the attack was still in setup), but in SEARCH the defender would fly at them | Proposed: publish only targets above (or a few degrees below) the horizon, as a setting; decide the default with Max |
| PX4 clock jumps of 60–79 s | Every run ("PX4 clock jumped … (time sync?)") | The attack is not affected (it runs on Gazebo's clock). The detector loses the camera orientation for about one image after a jump ("camera roll unknown"). Tests timed on PX4's clock (`detecttest`, the intercept chase) can time out | Proposed: switch off PX4's DDS time sync (`UXRCE_DDS_SYNCT` = 0) in the container |
| Lock drops in crossing | 20:39, 20:51 | Misses by ~4.4 m | Not known yet: needs the detector's output during a crossing run |

## Next

- [ ] Crossing: run again with the detector's output saved, to see whether the target leaves the picture or is lost against the background.
- [ ] More runs per setting (`--runs 5`) for crossing and head-on, gain 0.5 and 1.0, with the fake and the real detector.
- [ ] The same with the fake detector's realistic errors (`--realistic`) to compare with the real detector.
- [ ] Above-the-horizon filter for ground false alarms (decide with Max).
- [ ] PX4 time-sync fix.
