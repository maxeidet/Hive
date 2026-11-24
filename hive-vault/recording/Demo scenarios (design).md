# Demo scenarios (design)

Design, 24 Nov 2026, from a brainstorm with Oskar. Status: **draft for Oskar's approval** (his answers of 24 Nov folded in: 1 seed, green birds, film at most 5 min). Builds on [[Recording - smoother demo videos (design)]] and [[Recording demo videos]]; how to run things: `recording/README.md`.

## Why

A demo video for the course examiner. The examiner should come away seeing that Hive handles realistic threats, and that it does better than the baseline on the same runs. So every scenario is flown as a **pair**: the baseline (`--setup baseline`: blob detector, Confirmer, pure pursuit) and ours (`--setup yolo-bytetrack-mpc`), on the same seed, cut side by side with the existing `compare` and `ghost` layouts.

## Goals

1. **Three scenarios, each as a baseline/ours pair, in one shoot file** (`recording/shoots/examiner.yaml`), recorded unattended with `recording/shoot.sh`:
   - **One hostile** (exists): head-on, plus the zigzag path, where the difference shows most.
   - **Two hostiles at once** (new): two hostiles fly in at the same time from different bearings. What the examiner should see: the base station sends **two different defenders**, one at each, and both are hit.
   - **One hostile among birds** (new): 5-10 birds fly in around the hostile. Ours tracks and locks the drone and ignores the birds; the baseline blob detector is expected to be distracted.
2. **A quick check that `shoot.sh` works** before the full shoot (`recording/shoots/quick-check.yaml`: one head-on pair, about 25 min). Running on 24 Nov while this design was written.
3. Each new scenario can be filmed as soon as it is built: **step 1 birds, step 2 two hostiles**.
4. **The film is at most 5 minutes** (exact length still open): the examiner story is cut to fit, e.g. by using speed-ups for the approaches.

## Non-goals

- New paths, speeds or detectors. The scenarios reuse what `baseline_eval` already flies.
- The 1-defender scenarios (`mpc.sh`, `yolo.sh`, `birds.sh` on `scenario2.yaml`). They stay as they are.
- Making the sim faster (a separate open item, [[Manual tests to do]]).
- Pre-screening the new scenarios in the fast sim (`--prescreen`): the fast sim has no camera images, so birds mean nothing there; the two-hostile entries use fixed seeds.

## Anti-goals

- A demo that only works on hand-picked luck: the seeds are fixed and written in the shoot file, so the same film can be shot again. A film longer than 5 minutes.
- Breaking what works: the single-hostile `trial.sh`/`compare_paths.sh` runs, their results files and the 36-case evaluation must give the same results as before.

## Constraints

- Stellar (vultaumar): the sim runs at about **0.35x** in flight, so one run is about 10-12 min wall time; 8 runs is about 1.5-2 h. Disk: 54 GB free on 24 Nov, a run is 0.3-0.5 GB.
- `drone 0` runs the container's PX4 and Gazebo: it stays, parked in the base (it is a friendly since 24 Nov).
- Everything filmed is in Gazebo's sim time; the film tools find a run's engagements (takes) from the hostile's poses and the UI's events.

## Approaches

| | Approach | Trade-off | |
|---|---|---|---|
| **A** | Extend the 3-defender scenario (`baseline_eval`, used by `trial.sh`): `--birds N` and `--hostiles 2` | The same scene, seeds, setups and results for every entry; the most work (the two-hostile run and the film tools) | **Picked** |
| B | Use the existing 1-defender scripts (`mpc.sh` two hostiles, `birds.sh`) as shoot entries | Least scenario work, but one defender (no "two defenders, one each"), no baseline setup or seeds, a different scene, and the film tools would still need work | Ruled out: works against the point of scenario 2 |
| C | Birds now; two hostiles as two quick engagements in a row | Less work, but not "at once": the coordination is less visible | Not picked |

Oskar picked **A, in two steps: birds first, then two hostiles**.

## Step 1: birds (`--birds N`)

- `baseline_eval` (and so `trial.sh`, `compare_paths.sh`) gets `--birds N` (default 0: nothing changes) and `--bird-seed S` (default: the run's seed, so a pair gets the same birds).
- Each run, after the hostile is placed and before it is released, N birds are planned with `plan_birds()` from `intercept/inbound.py`, the planner `scenarios/birds.sh` (via `mpc.sh --birds`) already uses: the same rule (5-25 m sideways and up to 8 m in height from the start, level and straight over the base and 60 m past it, the hostile's speed ±30 %, every kind in `worlds/birds/` before any repeats). They fly with `python3 -m swarm_backend.birds fly` (the plan as JSON on stdin, on Gazebo's sim clock), started with the run and stopped and removed when the run ends (hit, end of track or Ctrl-C). Gazebo backend only: the fast sim (`--sim`) ignores `--birds` and says so once.
- Bird names start with `run.json`'s `birds_prefix` (`bird_`), so `posesrec` logs them and the map draws them already.
- **Birds are marked green in the films** (Oskar): in the live camera clips (`chase`, `wide`, `video.sh cam`), every bird in view gets a green box and a `bird` label, like the drones' labels (`camera_video.label_lines`); on the map the birds are green dots instead of grey. The YOLO views already draw the detector's bird boxes in green.
- The results row of each run records `birds: N` and the bird seed.

## Step 2: two hostiles at once (`--hostiles 2`)

- **Config:** a second hostile, drone 5 (`gz_x500`), in `scenarios/surveillance.yaml`, parked in the camp next to drone 4 (e.g. 5 m west of it). `attack.target_id` stays 4; the hostiles are 4 and, with `--hostiles 2`, 5. `trial.sh` and `mount-backend.sh spawn` start drone 5 only when asked for (the single-hostile runs keep 4 drones + drone 0, so they are not slowed down).
- **The base station** already keeps one target per target id (`base_station.py`, `targets`) and assigns a defender to each; nothing new there, but a test with two sightings of two targets checks it sends two different defenders.
- **Start points:** hostile A in drone 1's sector, hostile B in drone 3's sector (the outer thirds of the arc), at seeded bearings inside each, released at the same moment, same speed, path and altitude options as today. So the base station has two clearly separate targets.
- **The run (`surveillance.py` `fly()`, both backends):** both hostiles fly; a judge per (defender, hostile); a hit takes out that hostile only (its own graveyard spot: `GRAVEYARD_ENU` becomes one per hostile, far apart); the run ends when both are down, at the end of both tracks or at the timeout. Each hostile's PX4 is restarted per run as drone 4's is today.
- **The fast sim** (`sim_surveillance.py`) flies both too, with the fake detector seeing both: the shared `fly()` logic is where the unit tests check it.
- **Results:** one row per run, with a list of per-hostile outcomes (hit or not, by which defender, closest pass, first sighting); `summary` counts hostiles (`hits 2/2`). Single-hostile rows keep their current fields, so old results files still load.
- **Film tools:** `run.json` already lists `hostiles`. A take is still one engagement (one release). New moments `hit_1`, `hit_2` and `first_sighting_1`, `first_sighting_2` (per hostile, in the order of `hostiles`), with `hit` = when the last hostile is down and `first_sighting` = the first sighting of either, so the existing stories still cut. Facts per hostile (attacker, range at sighting, time to hit). The map draws both hostiles; the `chase` camera (`attacker_watch`) follows the defender nearest any hostile.
- **A story for it** (`recording/stories/examiner.yaml`, see below) shows the UI or map with both, and the chase camera.

## The shoot and the film

`recording/shoots/examiner.yaml` (fixed seeds, 2 per scenario, all as pairs):

```yaml
- name: overview
  run: idle 60                       # no scenario: the orbit camera around the base
  options: [--cameras, base_orbit, --no-yolo]
- name: head-on
  run: scenarios/trial.sh 1 --aspects 0 --speeds 4
  compare: [baseline, yolo-bytetrack-mpc]
  seeds: 0
- name: zigzag
  run: scenarios/trial.sh 1 --path zigzag
  compare: [baseline, yolo-bytetrack-mpc]
  seeds: 0
- name: birds
  run: scenarios/trial.sh 1 --birds 8
  compare: [baseline, yolo-bytetrack-mpc]
  seeds: 0
- name: two-hostiles
  run: scenarios/trial.sh 1 --hostiles 2
  compare: [baseline, yolo-bytetrack-mpc]
  seeds: 0
```

9 runs (8 + the overview), about 1.5 h; one seed per scenario (Oskar's choice: a bad run is flown again rather than kept as a spare); run without `--screen` (unattended, the films use the map), or with it when at the machine. The film: `recording/stories/examiner.yaml`, built from `compare.yaml`: title; overview; per scenario a card, the pair side by side (`compare`) and the ghost map; for two hostiles also a grid with both hits; the end card with both setups' results.

## Testing

- Unit tests with the fast sim and made-up runs (`fake_run.py`), as for the recording work: birds planned and passed to the bird process; two hostiles judged separately, both hit, ends when both are down; results rows and summary; takes with `hit_1`/`hit_2`; old results files still load.
- On Stellar: one pair per new scenario before the full shoot (a `quick-check` entry each), then the full shoot.

## Calls made in the design (Claude's, for Oskar to check)

- 8 birds per run; zigzag as the second single-hostile scenario; the overview run kept.
- The second hostile starts in drone 3's sector and the first in drone 1's, released together at the same speed.
- `hit` means "the last hostile is down", so existing stories still cut two-hostile runs.
- Drone 5 is only spawned when `--hostiles 2` is used.

## Left to the builder

- The second hostile's parking spot and graveyard spot (anything clear of the arc, the camp's tents and the first hostile).
- How the results row stores the per-hostile outcomes, as long as old rows load and the single-hostile summary is unchanged.
