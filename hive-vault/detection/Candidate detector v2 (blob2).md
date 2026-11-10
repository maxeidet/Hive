# Candidate detector v2 (blob2)

Plan for a better first stage: a new blob detector that finds candidates for YOLO to check (approach A/F in [[Training data and YOLO training]]). Written 18 Oct 2026, not built. Background: [[Lock-on brainstorm]] (candidate detection options), [[Detection implementation plan]].

## Why

In the two-stage pipeline the first stage sets the ceiling: YOLO only looks at cut-outs around candidates, so a drone the first stage misses is never found. Its job is different from YOLO's: **miss almost nothing, stay cheap**. False alarms are fine, YOLO rejects them.

The current baseline (`detect/candidates.py`) was built to be precise, the opposite. Where it loses drones:

| Rule today | Why it loses drones |
|---|---|
| Fixed contrast: 60 grey levels darker than the surroundings | A 2–4 px drone is smeared over its neighbours, so its contrast is often lower |
| Darkest pixel at most grey level 100 | Distant drones look grey, not black |
| Dark objects only (black-hat) | A sunlit drone against a dark cloud or trees is lighter than its background |
| 90 % of a ring around it must be "sky" by colour | Nothing is found against clouds, haze or trees, or right next to them |
| One window size (41 px), max 40 px, nothing touching the image edge | Close drones and drones half out of view are dropped |

**Measured 18 Oct** (`makecrops` on 118 generated images): the baseline puts a candidate on 61 % of the 2–4 px drones, 74 % at 4–8 px, 61 % at 8–16 px, 76 % at 16–32 px, 32 % at 32–64 px and none above 64 px. So about a third of the small drones never reach YOLO today.

## Design

1. **Contrast relative to local noise.** Not "60 darker" but "N times darker than the background varies here": black-hat (or top-hat) response divided by a local standard deviation of the background. On a smooth sky even a faint 2 px dot stands out; in busy clouds it takes more. The biggest gain.
2. **Both polarities.** Black-hat for dark targets, top-hat for bright ones.
3. **Several window sizes** (e.g. 9, 21, 61 px), so small and large drones are both found and a big neighbour does not hide a small dot. The largest window sets the largest target.
4. **"Calm background" instead of "sky".** The ring around a candidate must be fairly uniform (low texture), not necessarily blue. Works against clouds and haze; trees stay hard (that is for motion and YOLO).
5. **Keep the N best per frame** (e.g. 20, ranked by score) instead of a hard threshold. A fixed cost for YOLO, and a weak but real drone is not thrown away for being just under a line.
6. **Candidates at the image edge are kept**, marked as cut off (the distance estimate from their size is then unreliable). The drone's own propeller tips are handled by the own-body mask or YOLO, not by dropping everything at the edge.
7. **Optional sky/horizon mask** from the camera orientation, as today, to save work.

Built as a new detector `blob2` in `detectors.py`, next to `baseline`, so `predict`, `detect --detector` and `makecrops` can compare them on the same images.

## Tuning on labelled data

**What "tuning" means here:** letting the data pick blob2's settings instead of guessing them. blob2 has knobs (how many times darker than the background noise a candidate must be, the window sizes, how many candidates to keep per image), and every combination finds a different share of the drones at a different cost. Because we know where every drone is in the generated images, we can simply measure each combination and keep the best one. This is ordinary hyperparameter search, the same idea as choosing YOLO's confidence threshold on val.

The generated images (`data/gen/`) say exactly where every drone is. So instead of guessing thresholds, a small script (`tune_blob2.py`):

1. runs `blob2` with many parameter combinations (contrast factor, window sizes, ring uniformity, N) on the training runs;
2. for each combination counts **recall per pixel size** (share of labelled drones with a candidate on them, matched by centre) and **candidates per image** (the cost for YOLO);
3. picks the combination with the highest recall at a cost limit, e.g. ≥ 95 % recall at ≥ 4 px with ≤ 20 candidates per image;
4. checks it once on the val runs (never tuned on test), and writes the chosen settings as `blob2`'s defaults.

## Next to it: motion

Motion-compensated differencing (brainstorm option B, training note section D) as a second candidate source: warp the previous image onto the current one with the rotation from PX4's camera orientation (`K R K⁻¹`), subtract, keep what moved. Finds low-contrast movers and rejects static clouds; blind to a hovering drone, so it adds to blob2 rather than replacing it. Its candidates go to YOLO the same way. Needs consecutive frames (flown sequences, or `detect` live).

## How to judge it

- Candidate recall per pixel size and per background (sky, cloud, trees), and candidates per image, on the test runs: blob2 against the baseline.
- End to end: blob2 + YOLO on cut-outs against YOLO on full frames, same test runs (`makecrops` keeps the split).
- Time per image on the container's CPU.

## Open

- [ ] Which N per frame the drone's CPU can afford for YOLO on cut-outs (measure YOLO time per cut-out).
- [ ] How to label "background" (sky / cloud / trees) per object for the per-background recall: from the gen meta (camera pitch, what is behind the target) or by hand on a sample.
