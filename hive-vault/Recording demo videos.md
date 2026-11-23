# Recording demo videos

Demo videos of a scenario where the operator UI, the defenders' YOLO views and third-person camera angles show the same moment, frame for frame, and a final cut from them. The full guide, with every command, is **`recording/README.md`** in the repo.

Back to [[Home]]. The scenario itself: [[Surveillance attack test - how to run]].

## How it works

- Everything is recorded **during** the run and put on Gazebo's sim time, so any part can be cut out afterwards and the clips line up.
- **UI:** `./mount-backend.sh ui --record auto` saves its states with sim time and the computer's clock (`ui.jsonl`); `recording/video.sh screen` films the browser window. `screen-cut` maps the film's clock onto sim time.
- **YOLO views:** `./mount-backend.sh debugrec` records `/px4_N/detection/debug` (the boxes drawn) per defender.
- **Live cameras:** `./mount-backend.sh camrec` puts invisible cameras in the world (`attacker_watch`, `arc_side`, ...), moves them every sim step and films them; `video.sh cam` adds labels (id, role, status, range) from the true poses.
- `recording/video.sh final` builds the final cut from scenes (`recording/final_cut.py`).
- Gazebo's log (`start-stellar.sh --record`) can still be replayed and filmed by hand from any angle.

## Decisions (18 Nov)

- The UI is **filmed from the screen**, not rendered: headless Chromium hung on Stellar (with the GPU at start; without it on the panels' backdrop blur). The drift of a wall-clock film is removed by mapping the computer's clock to sim time with `ui.jsonl`. The Chromium render (`ui/render.py`, `video.sh ui`, Playwright in the image) was removed on 23 Nov.
- Third-person views come from **live cameras** instead of filming replays by hand. A separate process moves them: Gazebo service calls from a process with Python subscriptions stall the sim.
- The live cameras cost sim speed: 0.55x real time with the YOLO views, 0.34x with two cameras as well. Use one or two.
- First full run with everything: `data/recordings/20261118-145347/` (final_cut.mp4, 51.8 s).

## Checks

The live run can only be checked by hand: see [[Manual tests to do]] ("Demo videos still work after removing the Chromium render").
