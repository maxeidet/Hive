# Demo videos: recording a scenario and cutting the film

This folder turns a Hive scenario (e.g. `scenarios/trial.sh 1`) into a demo video. Everything
is recorded while the scenario runs, and afterwards any part of it can be cut out, put side by
side and edited into a final cut. All the pictures line up to the frame because every recording
is put on Gazebo's **simulation time**.

What you can record, each one optional:

| Recording | What it shows | Command during the run | File in the run folder |
|---|---|---|---|
| Gazebo log | every model's pose, 50 Hz (for manual replays) | `docker/start-stellar.sh --record` | `gz/` |
| UI states | what the operator UI shows, 10 Hz | `./mount-backend.sh ui --record auto` | `ui.jsonl` |
| UI film | the operator UI, filmed from the screen | `recording/video.sh screen` | `screen.mkv` |
| YOLO views | each defender's camera with the boxes and tracks drawn | `./mount-backend.sh debugrec` | `debug_d<ID>.mkv` |
| Live cameras | third-person views from invisible cameras that follow the action | `./mount-backend.sh camrec` | `cam_<angle>.mkv`, `poses.jsonl` |

Each run gets its own folder, `data/recordings/<date-time>/`, made by `start-stellar.sh
--record`. The other recorders write into the newest folder. **`data/` is not in git and not
backed up: copy the videos you keep somewhere else.**

Everything runs in the `hive-px4-vnc` Docker image (ffmpeg, OpenCV and Gazebo are in it).
Nothing is installed on the host.

---

## Quick start: a full run

### 0. Check the machine (Stellar rules)

```bash
cat /etc/stellar/booking.json      # booked by someone else? Ask before starting
nvidia-smi                         # someone else's heavy GPU work? Ask first
df -h / /scratch
```

The live cameras render on the GPU. The first `start-stellar.sh` after a change to
`docker/Dockerfile` rebuilds the image, which takes a few minutes.

### 1. Start, in this order (six terminals)

**T1: the sim, recording.** Wait for the `pxh>` prompt.
```bash
cd ~/Hive && docker/start-stellar.sh --record
```
It prints `Recording this run to /workspace/data/recordings/<run>/gz`.

**T2: the UI, recording**
```bash
cd ~/Hive/ros2_ws/src/swarm_backend
export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh ui --record auto
```
`SWARM_CONFIG` must be exported **in this terminal**. Without it the UI shows only drones 0–2,
not the defenders 1–3 and the hostile 4 of the surveillance arc. Then open
http://localhost:8090 in a browser on this computer's desktop and make it **fullscreen (F11)**:
the film shows exactly what is on screen, tabs included.

**T3: film the UI**
```bash
cd ~/Hive && recording/video.sh screen      # then click the browser window
```
It prints `Filming WxH at (x, y) of :2 ...`. Keep the window where it is, with nothing on top
of it, until the end.

**T4: the YOLO views**
```bash
cd ~/Hive/ros2_ws/src/swarm_backend && ./mount-backend.sh debugrec      # drones 1 2 3
```

**T5: the live cameras**
```bash
cd ~/Hive/ros2_ws/src/swarm_backend && ./mount-backend.sh camrec        # attacker_watch + arc_side
```
Wait until it prints `Camera ...: first image at sim time ...` for each camera.

**T6: the scenario**
```bash
cd ~/Hive/ros2_ws/src/swarm_backend && scenarios/trial.sh 1
```
`trial.sh` spawns the drones itself. Any scenario works, e.g. `./mount-backend.sh spawn` and
`./mount-backend.sh attacktest`.

### 2. Stop, in this order

1. **T6** Ctrl-C (the defenders land; wait for it).
2. **T5** Ctrl-C (it removes the cameras and prints how many images each got).
3. **T4** Ctrl-C (prints the sim times each drone covers).
4. **T3** Ctrl-C.
5. **T2** Ctrl-C (prints how many UI states it saved).
6. **T1** type `shutdown` at `pxh>` (the container is removed, that's fine).

### 3. Make the videos (from `~/Hive`)

**Pick the times.** Look for the first `sees the target` and note which drone intercepts:
```bash
recording/video.sh events
```
```
 sim time       T+  event
    218.9    199.0  Drone 3 sees the target  (40 m up)
    219.0    199.0  Drone 3 flying → intercept
    224.6    204.7  Drone 4 link lost  (last message 2 s ago)      <- the hit
```

**Cut each recording** to the same start `S` and length `D` in sim seconds. In the example
below, S = 212, D = 20, and drone 3 attacks:
```bash
recording/video.sh screen-cut --start 212 --duration 20                    # -> screen_212.mp4
recording/video.sh debug --id 3 --start 212 --duration 20                  # -> debug_d3_212.mp4
recording/video.sh cam --angle attacker_watch --start 212 --duration 20    # -> cam_attacker_watch_212.mp4
recording/video.sh cam --angle arc_side --start 212 --duration 20          # -> cam_arc_side_212.mp4
```
Every clip's first frame shows sim time `S`, so any two of them can be put together.

**Put two clips together**
```bash
recording/video.sh compose --ui screen_212.mp4 --gazebo cam_attacker_watch_212.mp4 --layout side --out demo.mp4
```

**Or make the final cut:** several scenes, each with its own layout. See
[The final cut](#the-final-cut).
```bash
recording/video.sh final                                                   # -> final_cut.mp4
```

**Shrink for sharing**
```bash
recording/video.sh compress final_cut.mp4 final_cut_10mb.mp4 10           # 10 MB
recording/video.sh compress final_cut.mp4 final_cut_10mb.mp4 10 1280      # and 1280 px wide
```

All file names are relative to the newest run folder. To use an older run, put its folder name
last, e.g. `recording/video.sh events 20261118-145347`.

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

### The final cut

`recording/final_cut.py` builds a film from scenes. Each scene has a caption, a sim-time
range, a layout and a speed. Nothing covers anything else:

| Layout | Shows |
|---|---|
| `full` | one picture, 1920×1080 |
| `side` | two pictures side by side (960×540 each) |
| `grid` | four pictures in a 2×2 grid |

Every picture gets its name in the top-right corner. Scenes fade in and out, and the sim-time
range is in the bottom-right corner.

The scenes are the `SCENES` list at the top of the script. The run 20261118-145347 was cut
like this, 51.8 s in all:

```python
BASE = 131.0          # sim time of the sources' first frame
ATTACKER = 3
SCENES = [
    ('Defenders arm and take off', 131.0, 145.0, 'side', ['wide', 'ui'], 1.0),
    ('Patrol - the hostile approaches (8x speed)', 145.0, 212.0, 'full', ['ui'], 8.0),
    ('Drone 3 detects the hostile and intercepts', 212.0, 224.2, 'grid', ['chase', 'yolo', 'ui', 'wide'], 1.0),
    ('The hit - replay at half speed', 221.5, 224.2, 'side', ['chase', 'yolo'], 0.5),
    ('Aftermath - the defenders return', 224.2, 236.0, 'side', ['wide', 'ui'], 1.0),
]
```

To cut a run:
1. Find where all recordings overlap: each `.json` has `start` and `end`. With `camrec`
   started before the trial, that is about from takeoff to landing. Set `BASE` to its start,
   and `ATTACKER` to the drone that intercepts.
2. Cut the four sources once, from `BASE` over the whole range (here 131 + 105 s):
   ```bash
   recording/video.sh screen-cut --start 131 --duration 105
   recording/video.sh debug --id 3 --start 131 --duration 105
   recording/video.sh cam --angle attacker_watch --start 131 --duration 105
   recording/video.sh cam --angle arc_side --start 131 --duration 105
   ```
3. Edit `SCENES`, then run `recording/video.sh final`. Building takes seconds, so try freely.

---

## Problems we hit, and what to do

| Symptom | Cause and fix |
|---|---|
| The UI shows only drones 0–2 | `SWARM_CONFIG` was not exported in the UI's terminal. Check with `echo $SWARM_CONFIG` before `ui`. |
| `Cannot get the image data event_error ... major_code:130` from `screen` | x11grab's shared memory from the sim container. `video.sh screen` uses its own `--ipc=host` container now; update the repo. |
| `video.sh ui` (headless render of the UI) never finishes | Chromium in the container: with the GPU it hangs at start; without it the UI's `backdrop-filter: blur` is redrawn in software every frame. Film the screen instead (`video.sh screen`). |
| `debugrec` prints no `first image` | The detectors aren't running yet (they start with the scenario), or that drone has no detector. |
| The sim is much slower | Every camera costs. 0.55× with the YOLO views, 0.34× with two live cameras as well. Use one camera. |
| A chase camera loses the drone at the hit | Fixed in the director (see Live cameras). Runs before that fix have it in their `cam_attacker_watch.mkv`: end that clip at the hit. |
| The film shows browser tabs or other windows | It films the screen as it is. Make the browser fullscreen (F11) and keep it on top. |
| `screen-cut`: `starts ... after sim time S` | `screen` was started later than S: pick a later S. |
| `Unknown angle` | The name is not in `recording_config.json`'s `replay_angles`. |
| A log of only a few KB (`du -sh data/recordings/*/gz`) | The sim was not started with `--record`. |

---

## Other ways (not needed for the workflow above)

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

### Rendering the UI instead of filming it

`recording/video.sh ui --start S --duration D [--collapse-events]` renders the UI from
`ui.jsonl` in headless Chromium, frame-exact, without a screen. It hangs on Stellar (see the
problems table). It may work on a machine where Chromium can use the GPU.

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
| `events [RUN]` | the UI's events with their sim times |
| `screen [--region WxH+X+Y]` | during the run: film the UI's browser window |
| `screen-cut --start S --duration D` | the UI film on sim time -> `screen_S.mp4` |
| `debug --id N --start S [--duration D]` | drone N's YOLO view -> `debug_dN_S.mp4` |
| `cam --angle A --start S [--duration D] [--plain]` | a live camera, labelled -> `cam_A_S.mp4` |
| `compose --ui F --gazebo F [--layout pip\|side\|stack] [--out F]` | two clips in one |
| `final [--out F]` | the final cut from `final_cut.py`'s `SCENES` |
| `compress IN OUT [MB] [WIDTH]` | two-pass H.264 to a size |
| `angles` | the camera angles |
| `replay -a ANGLE [--start S]` | a replay of the Gazebo log at one angle (by hand) |
| `ui --start S [--duration D]` | the headless UI render (hangs on Stellar) |

`ros2_ws/src/swarm_backend/mount-backend.sh` (during the run):

| Command | |
|---|---|
| `ui --record auto` | the UI, saving its states to the newest run folder |
| `debugrec [IDS]` | the YOLO views of drones IDS (default 1 2 3) |
| `camrec [ANGLES] [--width W --height H --hfov RAD]` | live cameras (default `attacker_watch arc_side`) |

### Files in a run folder

| File | From | |
|---|---|---|
| `gz/` | `start-stellar.sh --record` | Gazebo's log (poses, 50 Hz) |
| `ui.jsonl` | `ui --record auto` | UI states with sim time and the computer's clock |
| `screen.mkv`, `screen.json` | `video.sh screen` | the UI film (frames stamped with the computer's clock) and its region |
| `debug_d<ID>.mkv`, `.json` | `debugrec` | YOLO views, 25 fps on sim time; `.json`: start, end, images, dropped |
| `cam_<angle>.mkv`, `.json` | `camrec` | live cameras, 25 fps on sim time; `.json` also the field of view |
| `poses.jsonl` | `camrec` | every drone's and camera's true pose, 25 Hz |
| `*_<S>.mp4` | the cut commands | clips from sim time S |
| `final_cut.mp4` | `video.sh final` | the final cut |

### Files in this folder

| File | |
|---|---|
| `video.sh` | the host commands |
| `screen.py` | films the UI window and puts it on sim time |
| `final_cut.py` | the final cut's scenes |
| `compose.py` | two clips in one (ffmpeg) |
| `compress_to_size.sh` | two-pass H.264 to a target size |
| `recording_config.json` | world, model paths, camera angles, live camera settings, graveyard |
| `enable_log_record.py` | adds LogRecord to the container's copy of the world before Gazebo starts |
| `replay_camera.py` | pauses the replay at `--start` and sets the replay window's camera |
| `replay_gui_config.py` | the replay window's layout with a sim-time video recorder |
| `kitconf.py` | reads `recording_config.json` |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/debug_video.py` | `debugrec` and `video.sh debug`; the sim-time encoder the cameras share |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/camera_video.py` | `camrec` (cameras, director, mover) and `video.sh cam` (labels) |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/replay.py` | the UI recorder and its replay |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/render.py` | the headless UI render |

Tests (on the host: `cd ros2_ws/src/swarm_backend && ../../../.venv/bin/python -m pytest
tests/test_debug_video.py tests/test_screen_recording.py tests/test_camera_video.py
tests/test_ui_replay.py`). The ffmpeg test in `test_debug_video.py` only runs in the container.

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
