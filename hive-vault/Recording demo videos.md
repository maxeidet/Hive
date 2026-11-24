# Recording demo videos

Demo videos of a scenario where the operator UI, the defenders' YOLO views and third-person camera angles show the same moment, frame for frame, and a final cut from them. The full guide, with every command, is **`recording/README.md`** in the repo.

Back to [[Home]]. The scenario itself: [[Surveillance attack test - how to run]].

## How it works (since 23 Nov: step 1 of [[Recording - smoother demo videos (design)]])

- **Record in one command:** `recording/record.sh scenarios/trial.sh 1` starts the sim (headless), the UI recording, the pose logger, the YOLO views and the live cameras in a tmux session, each when the one before is ready, flies the scenario, stops everything in order and checks the run folder. `--screen` also films the UI (kiosk browser, whole screen), `--watch` shows Gazebo. Plan: [[Recording - step 1 plan (record and cut)]].
- **Cut from a story:** `recording/video.sh final [--preview]` builds the film from `recording/stories/default.yaml`: scenes tied to named moments of a take (`first_sighting - 5` to `hit`), so any run cuts itself. `video.sh moments` lists the takes, moments and facts.
- **The tactical map** (`map` source) is drawn afterwards from `poses.jsonl` and `ui.jsonl`: the default overview; the UI film is optional.
- Everything is on Gazebo's sim time, so the clips line up frame by frame.
- The older pieces still work by hand: `video.sh screen` / `screen-cut`, `debugrec` / `video.sh debug`, `camrec` / `video.sh cam`, `compose`, and replaying the Gazebo log.

## Steps 2-4 (built 23-24 Nov, untested on Stellar)

- **Many runs:** `recording/shoot.sh recording/shoots/demo.yaml [--prescreen 30]` flies a list of runs; `compare:` flies each run once per setup (baseline / ours), `--prescreen` picks random seeds in the fast sim. Resumes after a stop. Plan: [[Recording - step 2 plan (comparison and shoot)]].
- **Comparison film:** `recording/stories/compare.yaml`: `compare` scenes (baseline left, ours right, synced on the hostile setting off), a `ghost` map, and an end card with the shoot's and the earlier evaluation's hit rates. Takes are ranked (`take: best`).
- **Map details and insights:** the estimate next to the truth with the error in metres, the commanded velocity, and numbers at the right moment (`insights: true`). Plan: [[Recording - step 3 plan (map details and insights)]].
- **Extras:** `zoom` on the YOLO box with a freeze, graphs (`plot:range`), the `base_orbit` camera (`run: idle 60`), subtitles (`.srt`, `--burn-subtitles`), stills PDF and GIFs (`--stills`). Plan: [[Recording - step 4 plan (extras)]].
- A real-run fact found on the way: the Gazebo scenarios log no "hit the target" event (the patrol is called back first); the hit is the hostile's jump to the graveyard.

## Decisions (18 Nov)

- The UI is **filmed from the screen**, not rendered: headless Chromium hung on Stellar (with the GPU at start; without it on the panels' backdrop blur). The drift of a wall-clock film is removed by mapping the computer's clock to sim time with `ui.jsonl`. The Chromium render (`ui/render.py`, `video.sh ui`, Playwright in the image) was removed on 23 Nov.
- Third-person views come from **live cameras** instead of filming replays by hand. A separate process moves them: Gazebo service calls from a process with Python subscriptions stall the sim.
- The live cameras cost sim speed: 0.55x real time with the YOLO views, 0.34x with two cameras as well. Use one or two.
- First full run with everything: `data/recordings/20261118-145347/` (final_cut.mp4, 51.8 s, made with the old `final_cut.py`).

## Checks

The live runs can only be checked by hand: see [[Manual tests to do]] ("record.sh and final on Stellar", then "shoot.sh with a compare pair on Stellar" and "Map details, insights and extras on a real shoot").
