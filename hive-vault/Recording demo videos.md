# Recording demo videos

Demo videos of a scenario (built for `attacktest`) where the Gazebo view and the operator UI show the same moment, frame for frame. Built 18 Nov 2026 from the Gazebo recording kit of Oskar's Flatbed Falcon project. The full guide, with every command, is **`recording/README.md`** in the repo.

Back to [[Home]]. The scenario itself: [[Surveillance attack test - how to run]].

## How it works

- Nothing is filmed during the run, so the run behaves as usual.
- **Gazebo:** `docker/start-stellar.sh --record` (Mac: `./mount-backend.sh restart --record`) adds Gazebo's LogRecord: every pose at 50 Hz to `data/recordings/<run>/gz/`. Afterwards `recording/video.sh replay -a ANGLE --start S` replays the log, pauses it at sim time S and sets the camera; you film with the GUI's recorder, which runs on **sim time** (frame i = S + i/25).
- **Operator UI:** `./mount-backend.sh ui --record auto` saves the page's state 10 times a second, stamped with Gazebo's sim time, to `data/recordings/<run>/ui.jsonl` (`swarm_backend/ui/replay.py`). Afterwards `recording/video.sh ui --start S --duration D` serves that file as the UI node and renders the page in headless Chromium with a controlled clock, one screenshot at exactly S + i/25 per frame (`swarm_backend/ui/render.py`).
- `recording/video.sh compose` puts them together (inset, side by side or stacked), `compress` makes it fit an upload limit.
- Pick S with `recording/video.sh events`: the UI's events with their sim times.

## Decisions (18 Nov)

- Frame-exact sync instead of screen-recording the live UI: the sim runs slower than real time and not steadily, so a wall-clock UI recording would drift against the sim-time Gazebo clips.
- Filming on Stellar (GPU). Playwright, Chromium and ffmpeg are in the Docker image (`docker/Dockerfile`), nothing on the host.
- The page has a replay mode (`index.html?replay=<sim time>`); the live UI is unchanged.
- The drone's onboard camera is not filmed (the kit's live camera recording slows the sim down). If wanted later: record `/px4_N/detection/debug`.

## Checks

The live run and the replay window can only be checked by hand: see [[Manual tests to do]] ("Demo video, Gazebo and UI in sync").
