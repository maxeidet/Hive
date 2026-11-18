# Demo videos: Gazebo and the operator UI, in sync

This folder records a Hive scenario (e.g. `attacktest`) and turns it into a video where the
Gazebo view and the operator UI show the same moment, frame for frame. It is the Gazebo
recording kit from the Flatbed Falcon project (log record, replay with a scripted camera, the
GUI's recorder on sim time, two-pass compression), with Hive's angles, plus a recorder and
renderer for the operator UI.

## The idea

Nothing is filmed during the run, so the run behaves exactly as it does without recording:

- **Gazebo**: `LogRecord` saves every model's pose at 50 Hz (`--record`). Afterwards the log
  is replayed (`gz sim --playback`) once per camera angle and filmed with the GUI's video
  recorder, which is set to **sim time**: frame *i* of the mp4 shows sim time `S + i / 25`,
  where *S* is where the replay was paused when you pressed record.
- **Operator UI**: `ui --record` saves what the page gets from the UI node, 10 times a second,
  each stamped with Gazebo's sim time. Afterwards a headless Chromium opens the page in replay
  mode with its clock under control (Playwright's `page.clock`) and takes one screenshot per
  frame at exactly `S + i / 25`.

Both videos start at the same sim time *S* (`--start S` for both), so they line up.

```
 LIVE RUN                                     AFTERWARDS
 docker/start-stellar.sh --record             recording/video.sh events            pick S
   -> data/recordings/<run>/gz/state.tlog     recording/video.sh replay -a ANGLE --start S
 mount-backend.sh ui --record auto              you: record, play, stop -> <run>/ANGLE.mp4
   -> data/recordings/<run>/ui.jsonl          recording/video.sh ui --start S --duration D
 mount-backend.sh attacktest ...                -> <run>/ui_S.mp4
                                              recording/video.sh compose ... -> <run>/demo.mp4
                                              recording/video.sh compress demo.mp4 demo_10mb.mp4 10
```

Everything runs in the `hive-px4-vnc` image (ffmpeg, Playwright and Chromium are in it, see
`docker/Dockerfile`). `video.sh` uses the sim container when it runs, else starts a temporary
one (`px4-gazebo-vnc-video`) and removes it afterwards. Nothing is installed on the host.

## Step by step on Stellar

Check the machine first (`cat /etc/stellar/booking.json`, `nvidia-smi`). The image has to be
rebuilt once for ffmpeg and Playwright: `start-stellar.sh` does that on its own.

**Terminal 1, the sim, recording:**

```bash
docker/start-stellar.sh --record
```

It prints `Recording this run to /workspace/data/recordings/<run>/gz`. Wait for `pxh>`.

**Terminal 2, the UI, recording** (from `ros2_ws/src/swarm_backend/`):

```bash
export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml
./mount-backend.sh ui --record auto
```

It prints `Recording the UI on hive_base's simulation time to .../ui.jsonl`. Open
http://localhost:8090 if you want to watch (not needed for the video).

**Terminal 3, the scenario**, as usual, e.g.:

```bash
./mount-backend.sh spawn          # defenders 1-3 and hostile 4 (SWARM_CONFIG exported above)
./mount-backend.sh attacktest     # one run; --bearing -30 0 30 --runs 3 for three
```

Any scenario works the same way, e.g. `scenarios/trial.sh 1` (one intercept trial; it spawns
the drones itself). The camera angles assume `surveillance.yaml`'s drones.

**When the scenario is over**: Ctrl-C the UI (it prints how many states it saved), then stop
the sim (Ctrl+C in terminal 1; the container is removed, that's fine).

**Pick the start time:**

```bash
recording/video.sh events
```

lists every UI event with its sim time (link up, searching -> intercept, hit, ...). Take
*S* a few seconds before what you want to show, e.g. 10 s before the first sighting.

**Film Gazebo, once per angle** (`recording/video.sh angles` lists them):

```bash
recording/video.sh replay -a hostile_chase --start 312
```

1. Wait for `Replay paused at sim time 312.00 s` and `Camera set to hostile_chase`.
2. Don't touch the time slider (seeking backwards makes models vanish).
3. Click the video recorder button, choose **mp4**.
4. Press **play** (bottom left).
5. Click the record button again before the log ends. Save as
   `/workspace/data/recordings/<run>/hostile_chase.mp4` (the terminal prints the path).
   Close the window.

Use the same window size for every angle: the clip has the size of the 3D view.

**Render the UI** (headless, about 5-10 frames a second, so a 60 s clip takes a few minutes):

```bash
recording/video.sh ui --start 312 --duration 60
recording/video.sh ui --start 312 --duration 60 --collapse-events   # right side free for an inset
```

It writes `data/recordings/<run>/ui_312.mp4` at 1920x1080, 25 fps (`--size`, `--fps`).

**Put them together** (file names are relative to the run folder):

```bash
recording/video.sh compose --ui ui_312.mp4 --gazebo hostile_chase.mp4 --layout pip --out demo.mp4
recording/video.sh compose --ui ui_312.mp4 --gazebo d2_watch.mp4 --layout side --out demo_side.mp4
```

`pip`: the UI full size with Gazebo as an inset (`--pip-width 760 --pip-corner br`). `side`:
side by side, 2 x 1280x720. `stack`: Gazebo above the UI.

**Check the sync** on the hit: in Gazebo the hostile drone jumps away (it is moved to a
graveyard far outside the battle), and in the UI the attacking defender turns to `hit` in the
same frame. If they are off, nudge with `--gz-offset SEC` (the sim seconds the Gazebo clip
starts after the UI video; negative: before) and compose again.

**Compress** to an upload limit:

```bash
recording/video.sh compress demo.mp4 demo_10mb.mp4 10
recording/video.sh compress demo.mp4 demo_10mb.mp4 10 1280    # also scale to 1280 px wide
```

Copy what you keep off the Stellar machine: `data/` is not in git and not backed up.

## On the Mac

The same commands, with `./mount-backend.sh restart --record` instead of `start-stellar.sh
--record`. The replay window opens in the VNC desktop. Rendering is in software and slow, but
the clips still play at the right speed (sim time). Stop Gazebo before `replay`:
`docker exec px4-gazebo-vnc pkill -f 'gz sim'`. Don't use `restart --record` on Stellar: a
restart stops drone 0's PX4 and `start-stellar.sh` then removes the container.

## Camera angles (`recording_config.json`)

Drone entity names are `<model>_<id>`: with `scenarios/surveillance.yaml` the defenders are
`x500_gimbal_1..3` (left to right on the arc, 150 m north of the base at 40 m) and the hostile
is `x500_4`. Fixed positions are Gazebo world ENU (x east, y north, z up).

| Angle | What |
|---|---|
| `hostile_chase`, `hostile_close` | behind the hostile drone, 10 m / 5 m |
| `d1_watch`, `d2_watch`, `d3_watch` | rides behind a defender, keeps turning to the hostile: use the one that attacks |
| `base_tripod` | at the base, looking out at the arc, panning after the hostile |
| `arc_overhead` | high above the arc, panning after the hostile |
| `arc_side` | east of the arc, at defender height |

Offsets are in the followed model's frame (x forward, y left, z up). `-p 0.05` makes the
camera softer, `1.0` rigid. At a hit the hostile is moved far away, so a camera that follows
it jumps there: end the clip at the hit, or use a defender or tripod angle.

## Files

| File | |
|---|---|
| `video.sh` | the host commands above |
| `enable_log_record.py` | adds LogRecord to the container's copy of the world before Gazebo starts |
| `replay_camera.py` | pauses the replay at `--start` and sets the GUI camera to an angle |
| `replay_gui_config.py` | the replay window's layout with a sim-time video recorder |
| `recording_config.json` | world, model paths, camera angles |
| `compose.py` | UI + Gazebo in one video (ffmpeg) |
| `compress_to_size.sh` | two-pass H.264 to a target size |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/replay.py` | the UI recorder and its replay |
| `../ros2_ws/src/swarm_backend/swarm_backend/ui/render.py` | renders the UI to mp4 on sim time |

## Why the odd settings (from the Flatbed Falcon kit, all hit for real)

1. No `<record_resources>` in LogRecord: copying meshes into the log segfaults Gazebo when PX4
   spawns a drone. The replay finds meshes through `GZ_SIM_RESOURCE_PATH` instead.
2. The replay layout goes to `~/.gz/sim/<major>/playback_gui.config`: `--gui-config` with
   `--playback` shows the start menu, and closing it crashes.
3. The replay uses a server config with only UserCommands + SceneBroadcaster: PX4's config
   loads the Contact system, which segfaults on replayed contact sensors.
4. The recorder has `lockstep=false`, `use_sim_time=true`: with lockstep the window freezes
   while the replay is paused or at its end.
5. Never seek backwards in a replay (models vanish). To redo an angle, close and run again.
6. Fully automatic Gazebo filming (seek + screenshot per frame) does not work: the replay
   never advanced. Use the record button. (The UI is different: it's our own page, so its
   clock can be controlled.)
7. A log of only a few KB (`du -sh data/recordings/*/gz`) means the run was not recorded.
