# Demo videos: recording a scenario and cutting the film

This folder turns a Hive scenario (e.g. `scenarios/trial.sh 1`) into a demo video, in two
commands: **`record.sh`** records a run, **`video.sh final`** cuts it into a film from a
**story file**. Every recording is put on Gazebo's **simulation time**, so all the pictures
line up to the frame, and the story names **moments** (`first_sighting`, `hit`, ...) instead of
times, so any run cuts itself. Design: `hive-vault/Recording - smoother demo videos (design).md`.

What a run records (`record.sh` starts all of it):

| Recording | What it shows | Part | File in the run folder |
|---|---|---|---|
| Gazebo log | every model's pose, 50 Hz (for manual replays) | `start-stellar.sh --record` | `gz/` |
| Who is who | the command, config, setup, seeds, drones by role | `demo/runinfo.py` | `run.json` |
| UI states | what the operator UI shows, 10 Hz | `mount-backend.sh ui --record` | `ui.jsonl` |
| Poses | every drone's, bird's and camera's true pose, 25 Hz | `mount-backend.sh posesrec` | `poses.jsonl` |
| YOLO views | each defender's camera with the boxes and tracks drawn | `mount-backend.sh debugrec` | `debug_d<ID>.mkv` |
| Live cameras | third-person views from invisible cameras that follow the action | `mount-backend.sh camrec` | `cam_<angle>.mkv` |
| UI film (`--screen`, optional) | the operator UI, filmed from the screen | `video.sh screen --fullscreen` | `screen.mkv` |

The **tactical map** (`map`) is drawn afterwards from the poses and UI states, so it needs no
recorder and is always there. **`data/` is not in git and not backed up: copy what you keep
(`record.sh --export DIR`).** Everything runs in the `hive-px4-vnc` Docker image; the host
only needs bash and tmux (both on Stellar).

---

## Quick start

### 0. Check the machine (Stellar rules)

```bash
cat /etc/stellar/booking.json      # booked by someone else? Ask before starting
nvidia-smi                         # someone else's heavy GPU work? Ask first
df -h / /scratch
```

The first `start-stellar.sh` after a change to `docker/Dockerfile` rebuilds the image (minutes).

### 1. Record (one terminal, nothing to click)

```bash
cd ~/Hive && recording/record.sh scenarios/trial.sh 1
```

It starts the sim (headless), the UI, the pose logger, the YOLO views and the live cameras,
each only when the one before is ready, then flies the scenario, stops everything in order and
checks the run folder (✓ / ✗ per file). It prints the run's takes and `Ready to cut: ...`.

- Watch what is happening: `tmux attach -t hive-rec` (one window per part; detach with
  `Ctrl-b d`). Add `--watch` to see Gazebo's window too.
- `--screen` also films the operator UI (opens your browser in kiosk mode, films the whole
  screen: leave the computer alone while it runs).
- `--name head-on` names the folder; `--cameras attacker_watch` (one camera: the sim slows down
  less), `--no-cameras`, `--no-yolo`; `--export /media/usb` copies the run off the machine.
- `--dry-run` prints every step it would take. Ctrl-C stops everything in order.
- Any scenario works: `recording/record.sh ./mount-backend.sh attacktest --runs 3`.

### 2. Cut

```bash
recording/video.sh moments                    # the takes, their moments and facts
recording/video.sh final --preview            # stories/default.yaml at 960x540, fast
recording/video.sh final                      # the film: data/recordings/<run>/final_default.mp4
```

Edit `recording/stories/default.yaml` (or copy it: `--story mine`) and build again: sources are
cut once and cached, so a rebuild takes seconds. Each scene is also saved on its own in
`scenes/<story>/01_...mp4`, for slides. If a scene's moment or recording is missing (no hit,
recorded without cameras) nothing is built and every problem is listed; `--skip-missing` builds
the rest. `--take 2` uses take 2 for every scene.

### 3. Share

```bash
recording/video.sh compress final_default.mp4 final_10mb.mp4 10        # to a size limit
```

All file names are relative to the newest run folder; put a run's folder name last for an
older one, e.g. `recording/video.sh final --preview 20261118-145347`.

---

## The story file

`recording/stories/<name>.yaml`:

```yaml
title: Hive - a drone swarm that defends its base   # title card (title_seconds: 3)
subtitle: TDDE19, group 3
take: best                  # default take: the first that ended in a hit, or a number
end_card: results           # hits and times of the takes used (end_seconds: 3)

scenes:
- caption: Drone {attacker} detects the hostile and intercepts   # {attacker}, {take}
  from: first_sighting - 5  # a moment ± seconds
  to: hit
  layout: grid              # full (1 source), side (2), grid (4)
  show: [chase, yolo, map, wide]
  speed: 1                  # 8 = 8x faster, 0.5 = slow motion
  take: 2                   # optional: this scene from another take
- card: A weaving hostile   # a text card between scenes (duration: 3)
```

**Moments** of a take (`video.sh moments` lists them): `start`, `release` (the hostile sets off),
`takeoff`, `first_sighting`, `intercept`, `hit`, `return` (the attacker back to searching),
`landed`, `end`. A take is one engagement: a recording with `trial.sh 3` has three.

**Sources**: `map` (tactical map), `ui` (screen film, only with `--screen`), `yolo` (the
attacker's YOLO view), `yolo:<id>` (a fixed defender's), `chase` (`cam_attacker_watch`), `wide`
(`cam_arc_side`), `cam:<angle>` (any other recorded camera).

**Facts** (in `takes.json`, on the end card): hit, attacker, range at first sighting, release to
sighting, sighting to hit, closest pass, setup, the share of the engagement the chase camera kept
the target in frame (`in_frame_frac`), and the position error of the estimate (at the sighting and
the median).

**Takes by name**: `best`, `best2` (the ranking: a hit first, then a short time to hit, then the
target in frame), a number in a single run, or `<run>/<k>` in a shoot (`head-on-1-ours/1`).
`video.sh moments <shoot>` prints the ranking.

**More layouts**:

| Layout | Shows | Keys |
|---|---|---|
| `compare` | a baseline/ours pair side by side (baseline left), synced on `release`; each side's result appears at its hit | `take: best_pair` or `pair:<id>`, `show: [map]` (any one source); `from`/`to` are moments of the ours run |
| `ghost` | both runs of a pair on one map, the baseline in grey | `take: best_pair`, `show: [map]` |
| `zoom` | the YOLO view cropped around the locked box, frozen first | `show: [yolo]`, `zoom: 3`, `freeze: 1.5` |

**Insights**: `insights: true` (or a list: `seen_at`, `assigned`, `est_error`, `time_to_hit`,
`closest`, `no_hit`) shows the take's numbers at the moment they belong to ("seen at 74 m",
"3.1 s from sighting to hit"). In a compare scene each side gets its own.

**Graphs**: `show: [chase, "plot:range"]`, also `plot:base_distance`, `plot:est_error`.

**End cards**: `end_card: results` (the takes used), `end_card: compare` (the pairs used, plus
`evidence: data/baseline_eval`: the earlier Gazebo evaluation's hit rates as bars).

**Outputs** of `final`: the film, one clip per scene, a subtitle file (`.srt`) next to the film;
`--burn-subtitles` also writes `<film>_subtitled.mp4`; `--stills` writes `stills_<story>.pdf` (one
frame per scene) and a GIF of every scene with "hit" in its caption (480 px, at most 8 s).

Stories in this folder: `default.yaml` (one run), `compare.yaml` (a shoot with pairs).

---

## Many runs: `shoot.sh`

```bash
recording/shoot.sh recording/shoots/demo.yaml --prescreen 30      # in tmux, then walk away
recording/video.sh moments demo-<date>                            # the takes, ranked
recording/video.sh final --story compare --preview demo-<date>
```

A shoot file lists runs (`recording/shoots/demo.yaml`):

```yaml
- name: head-on
  run: scenarios/trial.sh 1 --aspects 0 --speeds 4   # any scenario command
  compare: [baseline, yolo-bytetrack-mpc]            # the same run once per setup: a pair
  seeds: 3                                           # or random
- name: zigzag
  run: scenarios/trial.sh 1 --path zigzag
  compare: [baseline, yolo-bytetrack-mpc]
  seeds: random
  want: [ours_hits, baseline_misses]                 # what --prescreen looks for
  repeat: 3
- name: overview
  run: idle 60                                       # no scenario: only the cameras
  options: [--cameras, base_orbit, --no-yolo]        # the orbit around the base
```

- Each run is one `record.sh` into `data/recordings/<shoot>/<run>/`. Runs in a row with the same
  config share the sim; only the first run's `gz/` has the Gazebo log.
- `--prescreen N` flies N random seeds per `want:` entry in the fast sim first (well under a
  second each, no Gazebo) and keeps the seeds that match. The fast sim has no camera images, so
  Gazebo can still turn out differently. Wants: `ours_hits`, `ours_misses`, `baseline_hits`,
  `baseline_misses`, `ours_sees_earlier`, `fast_hit`.
- A failed run is noted and the shoot goes on. Run the same command again after a stop: done
  runs are skipped (`shoot.json`), failed ones are recorded again.
- `--dry-run` prints the runs and their `record.sh` commands. Options after `--` go to every
  `record.sh` (e.g. `-- --screen`).

---

## How each recording works

### Everything on simulation time

The sim runs slower than real time, and unevenly: 0.55× with the YOLO views, 0.34× with two
live cameras as well. So a clip cut on the computer's clock would drift. Every recording is
therefore put on sim time. Frame *i* of every cut clip shows sim time `S + i / 25`.

- **YOLO views and live cameras** carry the sim time of each camera image. The recorder writes
  25 frames per sim second: each frame shows the newest image taken at or before its time. The
  start time is in the `.json` next to each `.mkv`.
- **UI film:** x11grab stamps every frame with the computer's clock. `ui.jsonl` saves the
  computer's clock next to the sim time 10 times a second. `screen-cut` maps one onto the
  other and speeds the film up to sim time. A test with a known colour switch landed on the
  right frame, with the sim at half speed.

### UI film (`video.sh screen`, `screen-cut`)

`video.sh screen` runs ffmpeg in a container of its own with the host's IPC namespace
(`--ipc=host`). x11grab hands the X server a shared-memory segment (MIT-SHM), and that fails
from the sim container (`Cannot get the image data event_error ... major_code:130`).

- `--region 1920x1080+0+0` films a fixed rectangle instead of a clicked window.
- `screen-cut` refuses a start before the film began: start `screen` before the scenario.
- `ui.jsonl` must exist, because it holds the clock pairs: run `ui --record auto`.

### YOLO views (`debugrec`, `video.sh debug`)

Each detector (`detect/live.py`) publishes its camera picture with the boxes drawn on
`/px4_<ID>/detection/debug`: yellow = candidate, red = confirmed track with its offset, green
= bird, blue = friendly drone. `debugrec` subscribes to them and encodes one video per drone
(two CPU threads each). The Gazebo log has no camera images, and the cameras don't run in a
replay, so this is the only way to get this picture.

- `debugrec 3` records only drone 3.
- Before a drone's first image, `video.sh debug` holds that first image, so the clip still
  starts at `S`.

### Live cameras (`camrec`, `video.sh cam`)

`camrec` puts one invisible camera per angle in the running world: a camera sensor on a link
without visual, collision or gravity. The defenders' YOLO can't see it and nothing hits it. A
director moves the cameras every sim step. A **separate mover process** sends the moves to
Gazebo, because a Gazebo service call from a process that also receives Python subscriptions
stalls the whole simulation (8 of 12 calls timed out, the sim at 0.04 s per 4 s). Every
model's true pose goes to `poses.jsonl` at 25 Hz for the labels.

```bash
./mount-backend.sh camrec                          # attacker_watch arc_side
./mount-backend.sh camrec attacker_watch           # one camera: the sim slows down less
./mount-backend.sh camrec d3_watch base_tripod     # any angles below
./mount-backend.sh camrec --width 1920 --height 1080 --hfov 1.0
```

`video.sh cam --angle A --start S --duration D` cuts one camera and draws the labels:
- a bracket on every drone where it really is in the picture
- `D3 defender` with its UI status and its range to the target (`intercept  74 m`)
- the attacker in orange, `D4 hostile` in red, the others in blue
- the sim time and the newest UI event in the top-left corner

`--plain` cuts without labels. The labels come from `poses.jsonl` and `ui.jsonl`, so you can
change them in `camera_video.py` (`label_lines`, `draw_overlay`) and cut again without flying
again.

**Angles** (`recording_config.json`, `replay_angles`). Positions are Gazebo ENU: x east, y
north, z up. The defenders are `x500_gimbal_1..3`, left to right on the arc, 150 m north of the
base at 40 m. The hostile is `x500_4`.

| Angle | What |
|---|---|
| `attacker_watch` | behind whichever defender is nearest the hostile, as seen from the hostile, looking at it. Cuts to another defender only when that one is clearly nearer (15 %) and 2 s after the last cut. **The best live angle.** |
| `d1_watch`, `d2_watch`, `d3_watch` | the same, behind one fixed defender |
| `hostile_chase`, `hostile_close` | behind the hostile, 10 m / 5 m, in its heading frame |
| `arc_side` | a tripod east of the arc at defender height, panning after the hostile: the wide shot |
| `base_tripod` | at the base, looking out at the arc, panning after the hostile |
| `arc_overhead` | high above the arc, panning after the hostile |

Live-camera behaviour at the hit:
- The hostile is moved to the graveyard, `graveyard_enu` = `gazebo_surveillance.GRAVEYARD_ENU`.
  The cameras hold it where it was, so they stay on the place of the hit.
- Within 15 m of the target, and after the hit, a watch angle keeps its direction and looks at
  its defender. Without that, the camera swung round in front of the defender as it flew past
  the hit point (run 20261118-145347, from 224.3 s).
- When the hostile is moved out again for the next run, it is let go.
- `live_camera.smooth_s` (0.3 s) smooths the motion.

**Cost:** each camera renders 1280×720 at 25 Hz on the GPU while the sim runs, beside the
three gimbal cameras. Keep it to one or two cameras, and check `nvidia-smi`.

---

## Putting clips together

### Two clips: `video.sh compose`

```bash
recording/video.sh compose --ui A.mp4 --gazebo B.mp4 --layout side  --out x.mp4   # side by side
recording/video.sh compose --ui A.mp4 --gazebo B.mp4 --layout pip   --out x.mp4   # B as an inset on A
recording/video.sh compose --ui A.mp4 --gazebo B.mp4 --layout stack --out x.mp4   # B above A
```
- `--ui` and `--gazebo` are just the first and second clip: any clips work.
- `pip` takes `--pip-width 760 --pip-corner br|bl|tr|tl`. Note that an inset covers part of the
  first clip.
- If two clips are off by a little, `--gz-offset SEC` shifts the second one (in sim seconds).
- A composed video can be composed again with a third clip.

### The film: `video.sh final`

See [The story file](#the-story-file). `swarm_backend/demo/final.py` replaced `final_cut.py`
(whose times were written for one run): the 18 Nov cut is `stories/default.yaml` now.

---

## Problems we hit, and what to do

| Symptom | Cause and fix |
|---|---|
| The UI shows only drones 0–2 | `SWARM_CONFIG` was not exported in the UI's terminal. Check with `echo $SWARM_CONFIG` before `ui`. |
| `Cannot get the image data event_error ... major_code:130` from `screen` | x11grab's shared memory from the sim container. `video.sh screen` uses its own `--ipc=host` container now; update the repo. |
| `debugrec` prints no `first image` | The detectors aren't running yet (they start with the scenario), or that drone has no detector. |
| The sim is much slower | Every camera costs. 0.55× with the YOLO views, 0.34× with two live cameras as well. Use one camera. |
| A chase camera loses the drone at the hit | Fixed in the director (see Live cameras). Runs before that fix have it in their `cam_attacker_watch.mkv`: end that clip at the hit. |
| The film shows browser tabs or other windows | It films the screen as it is. Make the browser fullscreen (F11) and keep it on top. |
| `screen-cut`: `starts ... after sim time S` | `screen` was started later than S: pick a later S. |
| `Unknown angle` | The name is not in `recording_config.json`'s `replay_angles`. |
| A log of only a few KB (`du -sh data/recordings/*/gz`) | The sim was not started with `--record`. |

---

## Other ways (not needed for the workflow above)

### Recording by hand (what `record.sh` does)

One terminal per part, from `~/Hive`, each started when the one before is ready, stopped in
the reverse order (the scenario first):

```bash
RECORD_RUN=myrun docker/start-stellar.sh --record --headless             # T1, wait for pxh>
export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
docker exec -w /workspace/ros2_ws/src/swarm_backend px4-gazebo-vnc python3 -m swarm_backend.demo.runinfo \
  write /workspace/data/recordings/myrun --config $SWARM_CONFIG -- scenarios/trial.sh 1
ros2_ws/src/swarm_backend/mount-backend.sh ui --record auto              # T2
ros2_ws/src/swarm_backend/mount-backend.sh posesrec                      # T3
ros2_ws/src/swarm_backend/mount-backend.sh debugrec                      # T4
ros2_ws/src/swarm_backend/mount-backend.sh camrec --no-pose-log          # T5
recording/video.sh screen --fullscreen                                   # T6, optional
cd ros2_ws/src/swarm_backend && scenarios/trial.sh 1                     # T7
```

### Filming a replay of the Gazebo log by hand

The Gazebo log (`--record`) can be replayed from any camera angle, also after the run. This is
the Flatbed Falcon kit's way. You film each angle yourself with the replay window's recorder:
```bash
recording/video.sh angles                                   # the angles
recording/video.sh replay -a hostile_chase --start 212      # Gazebo must not be running
```
1. Wait for `Replay paused at sim time 212.00 s` and `Camera set to hostile_chase`.
2. Don't touch the time slider (seeking backwards makes models vanish).
3. Click the video recorder button, choose **mp4**, press **play**.
4. Click the record button again before the log ends. Save as
   `/workspace/data/recordings/<run>/hostile_chase.mp4` (the terminal prints it).

The clip starts at sim time S like the others. `attacker_watch` only exists live. In a replay,
follow angles use the drone's own heading frame.

### On the Mac

The same commands, with `./mount-backend.sh restart --record` instead of `start-stellar.sh
--record`. Windows open in the VNC desktop, and rendering is in software (slow, but the clips
are on sim time). Don't use `restart --record` on Stellar: a restart stops drone 0's PX4, and
`start-stellar.sh` then removes the container.

---

## Reference

### Commands

`recording/video.sh` (on the host, from the repo; `RUN` = a folder in `data/recordings/`,
default the newest):

| Command | |
|---|---|
| `moments [RUN]` | the takes with their moments and facts (writes `takes.json`) |
| `final [--story NAME] [--preview] [--skip-missing] [--take N]` | the film from a story |
| `map --start S --duration D` | the tactical map -> `map_S.mp4` |
| `events [RUN]` | the UI's events with their sim times |
| `screen [--region WxH+X+Y \| --fullscreen]` | during the run: film the UI's browser window |
| `screen-cut --start S --duration D` | the UI film on sim time -> `screen_S.mp4` |
| `debug --id N --start S [--duration D]` | drone N's YOLO view -> `debug_dN_S.mp4` |
| `cam --angle A --start S [--duration D] [--plain]` | a live camera, labelled -> `cam_A_S.mp4` |
| `compose --ui F --gazebo F [--layout pip\|side\|stack] [--out F]` | two clips in one |
| `compress IN OUT [MB] [WIDTH]` | two-pass H.264 to a size |
| `angles` | the camera angles |
| `replay -a ANGLE [--start S]` | a replay of the Gazebo log at one angle (by hand) |

`ros2_ws/src/swarm_backend/mount-backend.sh` (during the run):

| Command | |
|---|---|
| `ui --record auto` | the UI, saving its states to the newest run folder |
| `debugrec [IDS]` | the YOLO views of drones IDS (default 1 2 3) |
| `camrec [ANGLES] [--width W --height H --hfov RAD] [--no-pose-log]` | live cameras (default `attacker_watch arc_side`) |
| `posesrec [--out DIR]` | the true poses to `poses.jsonl` |

### Files in a run folder

| File | From | |
|---|---|---|
| `gz/` | `start-stellar.sh --record` | Gazebo's log (poses, 50 Hz) |
| `ui.jsonl` | `ui --record auto` | UI states with sim time and the computer's clock |
| `screen.mkv`, `screen.json` | `video.sh screen` | the UI film (frames stamped with the computer's clock) and its region |
| `debug_d<ID>.mkv`, `.json` | `debugrec` | YOLO views, 25 fps on sim time; `.json`: start, end, images, dropped |
| `cam_<angle>.mkv`, `.json` | `camrec` | live cameras, 25 fps on sim time; `.json` also the field of view |
| `poses.jsonl` | `posesrec` (or `camrec` without `--no-pose-log`) | every drone's, bird's and camera's true pose, 25 Hz |
| `run.json` | `record.sh` | the command, config, setup, seeds, drones by role |
| `takes.json` | `moments`, `final` | the takes, their moments and facts |
| `logs/` | `record.sh` | one log per part |
| `cache/` | `final` | sources cut for the film |
| `scenes/<story>/` | `final` | one clip per scene and card |
| `*_<S>.mp4` | the cut commands | clips from sim time S |
| `final_<story>.mp4`, `preview_<story>.mp4` | `video.sh final` | the film |

### Files in this folder

| File | |
|---|---|
| `record.sh` | records a run in one command (tmux) |
| `video.sh` | the host commands |
| `stories/*.yaml` | the films' stories |
| `screen.py` | films the UI window and puts it on sim time |
| `compose.py` | two clips in one (ffmpeg) |
| `compress_to_size.sh` | two-pass H.264 to a target size |
| `recording_config.json` | world, model paths, camera angles, live camera settings, graveyard |
| `enable_log_record.py` | adds LogRecord to the container's copy of the world before Gazebo starts |
| `replay_camera.py` | pauses the replay at `--start` and sets the replay window's camera |
| `replay_gui_config.py` | the replay window's layout with a sim-time video recorder |
| `kitconf.py` | reads `recording_config.json` |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/debug_video.py` | `debugrec` and `video.sh debug`; the sim-time encoder the cameras share |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/camera_video.py` | `camrec` (cameras, director, mover) and `video.sh cam` (labels) |
| `../ros2_ws/src/swarm_backend/swarm_backend/demo/` | `runinfo` (run.json), `takes` (takes, moments, facts), `story`, `map_video`, `final`, `poselog`, `fake_run` (a made-up run for tests) |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/replay.py` | the UI recorder (`ui --record`) and reading it back (`events`, camera labels) |

Tests, in the container: `docker exec px4-gazebo-vnc bash -c "cd
/workspace/ros2_ws/src/swarm_backend && python3 -m pytest tests/test_demo_*.py
tests/test_debug_video.py tests/test_screen_recording.py tests/test_camera_video.py
tests/test_ui_replay.py"`. They run on a made-up run (`python3 -m swarm_backend.demo.fake_run
DIR` makes one to try the commands on).

### Why the odd Gazebo replay settings (from the Flatbed Falcon kit, all hit for real)

1. No `<record_resources>` in LogRecord: copying meshes into the log segfaults Gazebo when PX4
   spawns a drone. The replay finds meshes through `GZ_SIM_RESOURCE_PATH` instead.
2. The replay layout goes to `~/.gz/sim/<major>/playback_gui.config`: `--gui-config` with
   `--playback` shows the start menu, and closing it crashes.
3. The replay uses a server config with only UserCommands + SceneBroadcaster: PX4's config
   loads the Contact system, which segfaults on replayed contact sensors.
4. The replay's recorder has `lockstep=false`, `use_sim_time=true`: with lockstep the window
   freezes while the replay is paused or at its end.
5. Never seek backwards in a replay (models vanish). To redo an angle, close and run again.
6. Fully automatic filming of a replay (seek + screenshot per frame) does not work: the replay
   never advanced. That is why the live cameras exist.
