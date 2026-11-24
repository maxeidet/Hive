# Recording: smoother demo videos (design)

Design, 23 Nov 2026, from a brainstorm with Oskar. Status: **draft for Oskar's approval, nothing built**. Builds on what exists in `recording/` ([[Recording demo videos]], `recording/README.md`).

Back to [[Home]].

## Intent

The demo video for the course presentation: **5 minutes or more**, made **once**, but making it should be easy. A failed or boring run should be cheap to redo, many runs should be flown in one go so the best ones can be picked, and trying another story should not mean an evening of editing. Today two things make it hard:

1. **Recording** takes six terminals started and stopped in a strict order, a browser made fullscreen by hand and clicked to film it, and waiting for each recorder to report it is ready.
2. **Cutting** is by hand: read `video.sh events`, work out BASE and the attacking drone, cut four sources one by one, then edit the sim times in `final_cut.py`'s `SCENES`, which are hard-coded for the run of 18 Nov.

**What the audience should take away** (Oskar, 23 Nov): a bit of both "it works" and "how it works", but **most of all how it compares with the baseline**, and less about the inside.

The demo shows mainly the surveillance arc (`scenarios/trial.sh`: 3 defenders, the base station, a hostile flying in). Scenarios with birds and other enemy drones moving in other ways will be added later, and the tools must be ready for them (Oskar, 23 Nov).

## Goals

- **One command records a run**, in one terminal: it starts every recorder in order, waits until each is really ready, runs the scenario, and stops everything in order at the end or on Ctrl-C.
- **No clicking, no F11**: the UI film is **optional** (`--screen`); when it is on, the browser is opened and filmed by the command. The tactical map (section 8) is the default overview instead. *(Oskar, 23 Nov.)*
- **A shoot flies many runs one after the other** from a list (designed runs, or a bit random), unattended, and the takes are **ranked** so the best can be picked.
- **A new run cuts itself**: no sim times typed, no code edited.
- **The story is modular**: the story is not decided yet, so scenes are blocks in a small file that can be added, removed, reordered and changed one at a time. A story can use **several takes** (head-on, birds, two hostiles...). Several story files can sit side by side.
- **Fast to try**: a preview builds in a second or two; the full film in seconds to a minute.
- **Ready for new scenarios**: drones, birds and their roles come from the run, not from names written into the tools.
- **One file that plays anywhere** at the presentation, plus one clip per scene for slides, and a PDF of stills and a few short GIFs for the report.
- **The comparison carries the video**: baseline and our system on the same runs, side by side, with the numbers that show the difference.

## Non-goals

- Not a pipeline that runs after every code change; one good video is the target.
- **Writing the new scenarios** (birds in the surveillance scenario, other hostile movements, several hostiles at once). That is separate work in the scenario and attack code; this design only makes the video tools ready for them.
- No new recording kinds: the UI film, the YOLO views (`debugrec`) and the live cameras (`camrec`) stay as they are.
- Narration, music and sound.
- A live part in the presentation (Oskar: not good).

## Anti-goals (a failure even if it works)

- Anything with Chromium or another browser inside the container (tried twice, hangs on Stellar; Oskar: forget it).
- A command that hides failures: if a recorder never gets ready or dies, the run must say so loudly, not produce a video with a missing source.
- Changing the detection or attack code, or the ROS topics between them, for the video.
- Making the sim slower than today.

## Constraints

- Films on **Stellar** (GPU, rootless Docker, `start-stellar.sh`, which removes the container when PX4 stops). `restart` is not used there.
- Nothing installed on the host: everything runs in the `hive-px4-vnc` image or uses what Stellar already has (bash, tmux, a desktop browser).
- Everything stays on sim time, as today (frame i of a cut = S + i/25).
- Claude does not fly the sim; live checks go to [[Manual tests to do]].
- Never `git push`; Oskar commits.

## Approaches considered

**Filming the UI:**
- Browser in the container on a virtual display. *Rejected:* Chromium does not work on Stellar (Oskar).
- **Oskar's own browser, opened in kiosk mode by the command, filming the whole screen (chosen).** Proven (the 18 Nov final cut was filmed this way, only fullscreen by hand and a clicked window). Trade-off: the screen is busy for the whole run, and anything that pops up on it is filmed.

**Cutting:**
- Scenes in Python with named moments added. Still means editing code per story. *Rejected.*
- **A story file of scenes tied to named moments of chosen takes (chosen).** The one way that gives both "modular" and "a new run cuts itself".

**Many runs (the shoot)** *(Claude's call, see below)*:
- All runs in one sim and one recording folder. Fast (no sim restart between runs), but only works while every run uses the same drones (one config); scenarios with other drones or birds would not fit.
- **One recording folder per shoot entry, the sim restarted between entries with a different config, kept running between entries with the same one (chosen).** Every scenario can bring its own drones; a restart costs a couple of minutes, only where needed.

## The design

### 1. `recording/record.sh SCENARIO [ARGS]` — one run

One command, e.g. `recording/record.sh scenarios/trial.sh 1` or `recording/record.sh ./mount-backend.sh attacktest`. It is **a shoot with one entry** (section 2): the same code path, so a single run and a long shoot behave the same. *(Claude's call.)*

- It runs everything in a **tmux session** (`hive-rec`), one window per part, so each part keeps the terminal it needs (`start-stellar.sh` wants a TTY for `pxh>`), its output can be looked at (`tmux attach -t hive-rec`), and it survives the terminal closing. `scenarios/record_tuning.sh` already drives `start-stellar.sh` from tmux on Stellar.
- **Start order, each waiting until its part is ready** (a line in its log, or a file):
  1. sim: `docker/start-stellar.sh --record --headless` → `pxh>` and `Recording this run to ...` (with `--watch`: the Gazebo window instead, see Options)
  2. UI node: `mount-backend.sh ui --record auto`, with the scenario's `SWARM_CONFIG` → `Recording the UI on ...`
  3. *only with `--screen`*: browser opened on `http://localhost:8090` in kiosk (fullscreen) mode on this computer's desktop → the window exists
  4. *only with `--screen`*: screen film, `video.sh screen --region <whole screen>` (no click) → ffmpeg running and `screen.mkv` growing
  5. YOLO views: `mount-backend.sh debugrec <the defenders>` → started (first images come once the scenario starts the detectors)
  6. live cameras: `mount-backend.sh camrec [ANGLES]` → `first image` for every camera
  7. the scenario
- **Stop order** when the scenario ends or on Ctrl-C: scenario (defenders land) → camrec → debugrec → screen film → browser → UI node → sim (`shutdown` at `pxh>`). Each gets a timeout; one that does not stop is killed and reported.
- **Health**: if a part does not get ready within its timeout, or dies during the run, the command stops everything and says which part failed and where its log is. At the end it checks the run folder: every expected file is there and non-trivial (`gz/` more than a few KB, `screen.mkv` with `--screen`, one `debug_d<ID>.mkv` per defender, one `cam_<angle>.mkv` per angle, `ui.jsonl`, `poses.jsonl`).
- **The run describes itself**: it saves `run.json` in the folder: the scenario command and its arguments, the config (`SWARM_CONFIG`, copied in), the seeds, and the drones by role (defenders, hostiles, birds) as Gazebo names them. Everything after this reads roles from there (section 3), and a good take can be flown again from it.
- **Options**: `--screen` (film the UI, off by default), `--cameras attacker_watch,arc_side` (default as today), `--no-cameras`, `--no-yolo`, `--dry-run` (prints what it would start), `--keep-sim` (used by the shoot, below).
- **`--watch`** (off by default): Gazebo with its window, to see what is happening. Off, the sim is headless and a little faster. On together with `--screen`, the browser is kept in front and the command warns that the Gazebo window ends up in the UI film if it takes the front. *(Oskar, 23 Nov: "I want to have the option to see what is happening", as a flag.)*
- **At the end** it prints the run folder, the sim-time range every source covers, the takes with their moments and ranking (sections 3, 4), and the command to cut. With `--export DIR` (e.g. a USB stick or the home folder) it copies the run's small files and clips off `data/`, which is not backed up on Stellar.

### 2. `recording/shoot.sh SHOOT.yaml` — many runs in a row

A shoot is a list of runs in a file, e.g. `recording/shoots/demo.yaml`, flown one after the other, unattended (start it in tmux and walk away):

```yaml
- name: head-on
  run: scenarios/trial.sh 1 --aspects 0 --speeds 4 --seeds 3
- name: weave
  run: scenarios/trial.sh 1 --path weave --seeds 7
- name: random
  run: scenarios/trial.sh 3 --path zigzag --seeds random
  repeat: 2
```

- Each entry is any scenario command, so a new scenario (birds, other hostile movements) is one more line, with nothing to change in the tools.
- `seeds: random` (or `--seeds random`) picks seeds and saves them in `run.json`, so a good random take can be flown again exactly (idea 6: photogenic runs).
- Each entry gets its own recording folder (`data/recordings/<shoot>/<entry>-<n>/`). Between entries with the same config the sim keeps running (`record.sh --keep-sim`); a different config restarts it. *(Claude's call: different scenarios may bring different drones.)*
- An entry that fails is logged and the shoot goes on with the next one.
- **Camera-only entries**: `run: idle 60` flies no scenario, it only records the cameras for 60 s, e.g. the orbit shot around the base (section 10), so the real runs are not slowed down by an extra camera. *(Claude's call.)*
- **Pre-screening in the fast simulation** (idea 5): `shoot.sh --screen N` first flies N candidate runs per random entry with the scenarios' own `--sim` (seconds per run, no Gazebo), keeps the ones that match the entry's `want:` (e.g. `want: [ours_hits, baseline_misses]`, `near_camera: arc_side`), and only flies those in Gazebo. The screening results are saved, so the choice can be checked. *(Claude's note: the fast sim has no camera images, so how "interesting" it is can only be judged from the poses and the outcome; Gazebo may still differ.)* At the end: a table of every entry, its takes and their ranking.

### 3. Takes, roles and named moments

A recording may hold several engagements (`trial.sh 3`): each is a **take**, from the hostile being put in place for a run until the defenders are back. Takes are numbered across the whole shoot (`head-on/1`, `random-2/3`, ...).

**Roles** come from `run.json`: `defenders`, `hostiles`, `birds`. With several hostiles, each take still has one `attacker` and one `target` per engagement.

**Moments** of each take, found from the UI events (`ui.jsonl`) and the poses (`poses.jsonl`):

| Moment | From |
|---|---|
| `start` / `end` | where every source of the take overlaps |
| `release` | the hostile sets off on its run: its first **horizontal** movement after being put in place (it first climbs straight up and hovers), from `poses.jsonl`. Comparison pairs are synced on it. |
| `takeoff` | the first defender `armed` event |
| `first_sighting` | the first `Drone N sees the target` |
| `intercept` | the first `Drone N ... → intercept` |
| `hit` | the target's jump into the graveyard in `poses.jsonl` (the Gazebo scenarios log no `hit the target` event: the patrol is called back first). The attacker is the defender that went to intercept. A placement after which the hostile never flies (back on its spawn point at the end) is no take. |
| `return` | the attacker's `back to searching` after the hit |
| `landed` | the last defender `landed and disarmed` |

`takeoff` and `landed` may belong to the whole recording when the defenders stay up between runs. With several hostiles there is one `hit` per hostile: `hit`, `hit2`, ... in order. `video.sh moments [RUN|SHOOT]` lists every take with its moments.

### 3b. Facts: every number, computed once

Per take, one table of **facts** is computed from the moments, `ui.jsonl` and `poses.jsonl`, and saved next to the run (`takes.json`): hit or miss, attacker, target, range at first sighting, time from release to sighting, time from sighting to hit, closest pass, estimation error (median, at sighting), time the target was in the chase camera's frame, the setup (baseline / ours). **Everything that shows or uses a number reads it from here**: the insights (9), the ranking (4), the end card and the comparison (7), the subtitles (10), and the pre-screening's `want:` filter (2), which computes the same facts from the fast sim's output. A new fact is one function, and every user gets it. *(Claude's call: one place to compute and test instead of five.)*

### 4. Ranking the takes

`video.sh moments` also ranks the takes, best first, with the reasons shown, all from the facts:
- a hit (else last),
- shorter time from first sighting to hit,
- the target in the chase camera's frame for most of the engagement,
- the YOLO box big enough to see (if recorded).

The ranking is only a suggestion: a story names its takes itself, or says `best`, `best2`, ...

### 5. The story file

`recording/stories/<name>.yaml`. A header for the cards, then a list of scenes:

```yaml
title: Hive - a drone swarm that defends its base
subtitle: TDDE19, group 3
take: best                   # the default take; a scene may name another
end_card: results            # the results of the takes used (hits, time to hit)

scenes:
- caption: Defenders arm and take off
  from: takeoff - 2
  to: takeoff + 12
  layout: side
  show: [wide, map]
- caption: Patrol - the hostile approaches (8x speed)
  from: takeoff + 12
  to: first_sighting - 5
  layout: full
  show: [map]
  speed: 8
- caption: Drone {attacker} detects the hostile
  from: first_sighting - 1
  to: first_sighting + 1
  layout: zoom                # idea 2
  show: [yolo]
  freeze: 1.5
- caption: Drone {attacker} intercepts
  from: first_sighting
  to: hit
  layout: grid
  show: [chase, yolo, map, wide]
- caption: The hit - half speed
  from: hit - 2.5
  to: hit + 0.3
  layout: side
  show: [chase, yolo]
  speed: 0.5
- card: "A weaving hostile"   # a text card between scenes
- caption: Weaving in
  take: weave/1
  from: first_sighting - 5
  to: hit + 1
  layout: grid
  show: [chase, yolo, map, wide]
```

- **Layouts**: `full`, `side`, `grid` as today, plus `zoom`: the YOLO view cropped and enlarged around the target's box (known from the tracker), with an optional `freeze` in seconds on the first frame where the box is confirmed.
- **Sources by role**: `map` (the tactical map, always there), `ui` (the screen film, only when recorded with `--screen`), `yolo` (the attacker's YOLO view), `yolo:<id>` (a fixed defender's), `chase` (`cam attacker_watch`), `wide` (`cam arc_side`), `cam:<angle>` for any other recorded camera. Roles live in one table, so a new source is one line. Captions may use `{attacker}`, `{take}`.
- **Cards**: `title`, `subtitle`, a `card:` scene for text between scenes, and an `end_card` with the results of the takes used (hits, time from sighting to hit), computed from the takes.
- The 18 Nov cut becomes `stories/default.yaml`.

### 6. `video.sh final [--story NAME] [--preview] [RUN|SHOOT]`

- Reads the story, resolves each scene's take and moments, works out which sources and sim ranges are needed, **cuts the missing ones by itself** (the existing `screen-cut`, `debug`, `cam`, cached in the run folder so a second build only redoes what changed), and builds the film as `final_cut.py` does today (fades, captions, labels, sim time in the corner).
- `--preview`: quarter resolution, fast encoding, for trying the story (idea 8). It prints each scene's take, sim range and length, and the total.
- Writes `final_<story>.mp4` (or `preview_<story>.mp4`): H.264, 1920×1080, 25 fps, yuv420p, `+faststart`, the format that plays in PowerPoint, Keynote, browsers and VLC (idea 10). `video.sh compress` stays for size limits.
- If a scene needs a source or a moment its take does not have (recorded with `--no-cameras`, no hit), it **stops before building** and names the scene and what is missing; `--skip-missing` builds the other scenes anyway. *(Claude's call: the anti-goal says never quietly produce a video with a source missing.)*
- `final_cut.py`'s hard-coded `BASE`, `ATTACKER`, `SOURCES` and `SCENES` go.

### 7. The comparison: baseline vs. ours

- **Pairs in the shoot**: an entry with `compare: [baseline, yolo-bytetrack-mpc]` flies the same run (same seeds, path, speed) once per setup (`--setup`, `scenarios/common.py` `SETUPS`). The two takes are linked as a pair.
- **`compare` layout**: the pair side by side, **synced on `release`**: both runs fly the same hostile path, so from the same time after release the audience sees which one detects and hits first (syncing on each take's own sighting would hide exactly that difference). Times in a compare scene are given on the first take's moments, e.g. `from: release + 20`, labelled "Baseline" / "Ours", each with its result when it comes ("hit 3.1 s after sighting" / "miss, closest 6 m"). It works with any source (`chase`, `map`, `yolo`).
- **Summary card** (`end_card: compare`): over every pair of the shoot, hit rate, median time from sighting to hit, and median detection range, baseline vs. ours, as a small bar chart drawn in the film.
- **Backed by the evaluation** (idea 3): the summary card can also read the results of the earlier Gazebo evaluations (the 36-case comparison, `baseline_eval`, given as a path in the story: `evidence: data/baseline_eval/...`) and show them next to the shoot's pairs ("36 runs: baseline 12/36 hits, ours 29/36"), since many runs convince more than three.
- **Ghost overlay** (idea 1, layout `ghost`, on the `map` source): both takes of a pair on one map, ours in colour and the baseline's defender as a grey ghost, synced on `release` like `compare`.

### 8. The tactical map (a new source, `map`)

A clean top-down view drawn in Python from `poses.jsonl` and `ui.jsonl` after the run, at 25 fps on sim time like the other sources: no browser, no GPU, sharp at any size, nothing pops up on it. It is a source like `ui` or `chase`, so a story can show it anywhere. **It is the default overview** now that the UI film is optional, in the UI's colours (blue friendly, red hostile) so the two look alike when both are shown. The basic map (base, arc, drones, trails, cones, timeline) comes in step 1; truth vs. belief and the ghost in step 3.

- the base, the arc and the search sector, the defenders and the hostile(s) with short trails, birds as small grey marks;
- each defender's camera cone (from the gimbal heading in `ui.jsonl`);
- **truth vs. what the system believes**: the hostile's true track next to its estimated position from the detections, with the error in metres;
- the attacker's commanded velocity and the target it is flying at (from the attack status);
- a timeline bar along the bottom: the take's moments as marks, a cursor running along.

### 9. Insights: numbers at the right moment

Short labels that appear in the film where they belong, computed from the take (`ui.jsonl`, `poses.jsonl`): "seen at 74 m" at `first_sighting`, "D3 assigned" at `intercept`, "3.1 s from sighting to hit" and "closest pass 0.8 m" at `hit`. A scene turns them on with `insights: true`; which ones exist is one table, so a new one is one line. In a `compare` scene each side gets its own.

### 10. More sources and outputs (smaller)

- **Synced graphs** (source `plot:<name>`): range to target, distance of the hostile to the base, drawn as a running graph on sim time, from the same files.
- **Orbit shot** (camera angle `base_orbit`): a live camera that circles the base slowly (radius, height and period in `recording_config.json`), for an overview at the start.
- **One clip per scene**: `final` also writes `scenes/<story>/01_<caption>.mp4`, ... for putting each scene on its own slide.
- **Subtitles** (idea 9): `final` writes `final_<story>.srt` from the scene captions, cards and insight labels, timed to the film, and `--burn-subtitles` draws them into the film instead (for players that don't load .srt).
- **Stills and GIFs**: `final --stills` writes a PDF with one frame per scene and its caption, and a short GIF of each `hit` scene (small, for the report and the README).

### Order of the work

Big enough to do in steps, each usable on its own:

1. **Record and cut**: `record.sh` (a one-entry shoot) with `--screen` optional, takes, moments and the facts table, the basic `map` source, the story file, `final` with `--preview`, cards, one clip per scene. (The core: from this step on a video can be made without editing code, and without the browser.)
2. **The comparison**: `shoot.sh` with pairs and pre-screening, ranking, the `compare` layout and the summary card with the evaluation numbers.
3. **Map details and insights**: truth vs. belief, the ghost overlay, the insight labels.
4. **Extras**: zoom on the YOLO box, graphs, the orbit shot, subtitles, stills and GIFs.

## Decided by Oskar (23 Nov)

- The video is a one-off demo for the course presentation, 5 minutes or more; the process must be easy.
- Recording in one go is the main concern; cutting must be plug and play and modular.
- The story will stay the same across runs, but is not decided yet, so it must be modular.
- No Chromium; film his own browser.
- Several engagements per recording, as takes; pick one by number, default the first with a hit (now: `best`).
- Gazebo's window is a flag (`--watch`).
- Wanted from the ideas: preview (8), ranking takes (5) with a script that flies many runs, designed or a bit random (the shoot), photogenic runs that can be flown again (6), one file that plays anywhere (10), zoom on the YOLO box (2), title and end cards (1).
- Mainly `trial.sh`; scenarios with birds and other enemy drones will be added, and the tools must be ready for them. A hard case with ~20 birds around a drone is wanted once such a scenario exists.
- From the second round of ideas: baseline vs. ours (1), hard cases (2), the tactical map (3) with insights, synced graphs (4), an orbit shot around the base (5), one clip per scene (6), a PDF of stills and short GIFs (8). Not a live part (7).
- The audience should get a bit of both, most of all the comparison with the baseline, less how it works inside.
- From the last round: the ghost overlay (1), the evaluation numbers on the end card (3), pre-screening in the fast sim (5), subtitles (9).
- The UI film is optional; the map is the overview (after the re-evaluation).
- The insights wanted most: numbers at the right moment, a shoot summary (baseline vs. ours), and truth vs. what the system believes.

## Risks to check early

- Pre-screening: whether the fast sim (`--sim`) supports `--setup baseline`, and whether its outcomes predict Gazebo well enough. Test first in step 2; drop pre-screening if not.
- The zoom needs the YOLO box's coordinates. `detect/live.py` already has a record option (for `detect/replay_eval.py`) that saves every box; use that rather than changing the detection code.
- `release` must be found reliably from the poses in every scenario (the hostile is moved into place, then starts flying).

## Claude's calls (check these)

- tmux for `record.sh`, as `record_tuning.sh` already does on Stellar.
- A part that never gets ready, or dies, stops the whole run.
- One recording folder per shoot entry; the sim restarts only when the config changes.
- `final` stops on a missing source unless `--skip-missing`.
- Roles and seeds saved per run in `run.json`, read by everything after.
- Ranking criteria as in section 4.
- A comparison pair = the same run flown once per setup, linked in the shoot.
- The map is drawn offline from the recorded files (no extra recorder during the run, so the sim is not slowed down).
- The order of the work in four steps; the basic map moved into step 1 because the default story uses it.
- Comparison pairs and the ghost synced on `release`, not on each take's sighting.
- One facts table per take (3b) used by insights, ranking, end card, subtitles and pre-screening.
- `record.sh` is a one-entry shoot; the orbit shot is a camera-only shoot entry; the story sets its default take once at the top.

## Left to the builder

- The exact readiness lines and timeouts per part, and the tmux window layout.
- How the browser is opened in kiosk mode on Stellar (which browser is there), and how the screen's size is found.
- How the scenarios report seeds and roles for `run.json` (read from their arguments and config where they don't print them).
- The YAML parsing (PyYAML is in the image), the moment expression syntax (`moment ± seconds`), cache staleness.
- How the zoom finds the box (the debug images have it drawn; the tracker's box may need to be recorded next to them).
- Tests: moments and takes from a made-up `ui.jsonl` and `poses.jsonl`, story parsing and scene ranges, ranking, the start/stop order as a dry run.
