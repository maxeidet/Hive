# Manual tests to do

Tests that need a person and a running simulation, for the detection work. Claude adds a test here whenever something is built that it could not check fully itself; tick it off (and note what you saw) when done. Newest first.

Back to [[Home]]. How to run things: [[Tutorial - live detector (detect)]], [[Manual - scenario 1, images and prediction]].

---

## To do

- [ ] **shoot.sh with a compare pair, and the compare film** (added 24 Nov; step 2 of [[Recording - smoother demo videos (design)]], tested only on made-up runs and the fast sim. The single-run parts passed on 24 Nov, see Done)
  1. `df -h /` first (54 GB free on 24 Nov; a run is 0.3-0.5 GB). Then `cd ~/Desktop/Hive && tmux new -s shoot 'recording/shoot.sh recording/shoots/demo.yaml --name demo-1 --prescreen 30'` and leave it (9 runs, about 2 h at the sim's 0.35x).
  2. Half-way, Ctrl-C it once, then run the same command again: it should skip the done runs.
  3. `recording/video.sh moments demo-1`, then `recording/video.sh final --story compare --preview demo-1` (side by side and the ghost map).

  **Pass:** runs with the same config follow each other without restarting the sim; the restart carries on; `moments` ranks a hit first; in the compare scenes both sides show the hostile setting off at the same moment, each side's result appears at its own hit, and the ghost shows the baseline in grey at the same time after the release; the `overview` run's orbit camera circles the base smoothly. **Note here:** for the zigzag entry, how many of the pre-screened "ours hits, baseline misses" seeds really did so in Gazebo, and how long the shoot took.

- [ ] **record.sh ends by itself after the last intercept** (added 24 Nov; `recording/record.sh`, quick end. Only the dry run is tested)
  1. `recording/record.sh --name <name> --screen scenarios/trial.sh 1` and let it run to the end.

  **Pass:** after the hit it prints `Last engagement over (sim ...)`, about 15 sim s later `Done: stopping without landing`, then every health line is ✓ and `final --preview` builds with the aftermath scene. **Note here:** how long from the hit to `Ready to cut`.

- [ ] **Why the sim runs at 0.35x in flight** (added 24 Nov; to look into before long shoots)
  Measured over phases (sim time / wall time from `ui.jsonl`): 0.58-0.95x before the drones spawn, 0.34-0.36x during takeoff and flight (run 4, headless, Gazebo on the GPU), 0.23x with `--screen` while other processes were busy, 0.17x with `--watch` (the cameras then render without the GPU). Candidates: Gazebo physics with 5 PX4s in lockstep, the 4 cameras, the screen film (x264 at 2560x1440). **Note here:** the speed with `--no-cameras --no-yolo`, then with each added back.

- [ ] **A defender took another defender for the target** (added 24 Nov, demo-ui2, sim 93.6 s) Drone 1 reported a target at N 160, E 15, 40 m up: about 20 m from drone 2's slot at drone 2's height. The friendly check (`hive/friendlies.py`, box size against the hivemind's positions) did not mark it. **Note here:** whether it happens again (`recording/video.sh events <run>`, a "sees the target" before the release far from the hostile).

- [ ] **YOLO (gazebo-only) and the new range rule in a live chase** (added 6 Nov; range rule fitted offline, see `training/fit_range.py`)
  1. Container and drone 0 up. In `ros2_ws/src/swarm_backend/`: `scenarios/scenario1.sh --duration 120`.
  2. Second terminal: `./mount-backend.sh detect 2 --detector yolo --weights /workspace/models/yolo26n-gazebo-only-1280.onnx --conf 0.25 --save /workspace/detect_frames`. Then the same with `detect 2` (baseline) on another flight to compare.

  **Pass:** it prints the model load line, finds drone 1 in most frames, and the estimated distance on the debug image (`/px4_2/detection/debug`) is within about 40 % of the real one (YOLO now uses 0.8 m / sqrt(w*h), the baseline still 0.5 m / longer side). **Note here:** seconds per frame, how many frames it found the drone in, and the worst range error you see.

- [ ] **New birds look right** (added 18 Oct)
  1. Gazebo with `hive_base`. `./mount-backend.sh boxtest --bird eagle --out /workspace/data/box_test_eagle`, then the same with `--bird seagull` and `--bird soaring_bird`.
  2. Open `003m_boxes.png` and `010m_boxes.png` of each.

  **Pass:** the bird flies away from the camera (we see its back/tail, wings spread across the image, upright), roughly the size its "expected w" says, with a green box on it. **If it is sideways or upside down,** note which; the fix is `yaw`/`roll` in `KINDS` in `birds.py`. Then `./mount-backend.sh gendata --run birds01 --count 20` should show birds in some images, with label lines starting with `1`.

- [ ] **Different kinds and colours of drones in the generated images** (added 18 Oct)
  1. Gazebo with `hive_base`, then `./mount-backend.sh gendata --run kinds01 --count 30 --variants 10`. It prints `Drones this run: x500/original, standard_vtol/red, ...`.
  2. Look through `data/gen/kinds01/images/` (the near ones, 20+ px, show the models best; `meta/*.json` → `placed` says which variant is where).

  **Pass:** the different models show up (small boxy PX4 Vision, tilted-arm omniquad, flying-wing tailsitter, large VTOL plane), upright and the right size relative to each other; repainted ones are visibly that colour; each has a label line. **Note here:** any kind that is missing, invisible, sideways or the wrong size, and whether repainting works on the textured x500 (if it keeps its carbon look, that's fine but note it).

- [ ] **Cut-outs for YOLO on candidates** (added 18 Oct; no simulation needed, only the container)
  1. `./mount-backend.sh makecrops` (in `ros2_ws/src/swarm_backend/`): reads the runs in `data/gen/`, writes scaled cut-outs to `data/crops/`. For the other mode: `./mount-backend.sh makecrops --mode fixed --out /workspace/data/crops_fixed`. (Claude ran both offline on 18 Oct: it works; see the recall in [[Training data and YOLO training]].)
  2. Look at some `data/crops/<run>/images/*.png` and their `labels/`.

  **Pass:** it prints how many cut-outs with a target and how many false-alarm cut-outs it made, and a table of the baseline's candidate recall per pixel size; the cut-outs with a label line show a drone where the label says (about the centre, off by a few pixels), the others show sky/cloud/tree edges/own propellers. **Note here:** the recall table (that is the most the two stages can find), and the counts.

- [ ] **Live generator: the gimbal drone takes the photos** (added 17 Oct; see [[Training data and YOLO training]])
  1. Gazebo with `hive_base`, then `./mount-backend.sh spawn gimbal_labels` (note the id it gets, N) and wait until it shows in Gazebo.
  2. `./mount-backend.sh genlive N --run L01 --count 20 --heights 30 60`. It takes off, climbs, turns between images, and lands at the end.
  3. Look through `data/gen/L01/images/` and the `labels/`.

  **Pass:** the drone takes off and the gimbal visibly moves between images; the images show the drone's own legs/propellers when looking level or up, like scenario 1; the placed drones have boxes that sit on them (check a few against `meta/*.json`). **Note here:** images per second; whether PX4 started with the gimbal working (if `genlive` says the model has no bounding-box cameras, or the gimbal never moves, paste `/tmp/px4_N.log`'s first lines). **Also try** `./mount-backend.sh genlive 2 --run L02 --count 10 --labels projected` on a stock `spawn gimbal` drone and compare the boxes.

- [ ] **Generator shows the camera drone's own body** (added 17 Oct)
  1. `./mount-backend.sh gendata --run own01 --count 20` (hive_base running).
  2. Flip through `data/gen/own01/images/`, especially the ones looking up (`camera.pitch` > 0.3 in `meta/*.json`).

  **Pass:** in images looking level or up, the own x500's legs or propellers show near the top or edges of the image, like in the scenario 1 images from drone 2, and they have no label line in `labels/`. **If they never show,** or cover the whole image, note it: `CAMERA_ON_BODY` in `gen_data.py` is then off.

- [ ] **YOLO weights in our pipeline** (added 17 Oct; after the first Colab training, see `training/README.md`)
  1. Put `best.onnx` (exported with `opset=12`) in `models/yolo26n-test/` in the repo.
  2. `./mount-backend.sh predict yolo`: runs the model on `data/Manualimages/` and draws the boxes in `data/results/yolo/`.
  3. With scenario 1 flying: `./mount-backend.sh detect 2 --detector yolo`.

  **Pass:** it prints `YOLO: .../best.onnx (opencv 4.6.0 ...)` or `(onnxruntime)` and finds drone 0 in some images (with 30 training images, expect many misses). **Note here:** if it says `Could not load ...`, paste the error (then onnxruntime goes into the container); and the time per image `detect` reports, compared with the baseline.

- [ ] **Training data generator: first small run** (added 17 Oct; see [[Training data and YOLO training]])
  1. Gazebo running with `hive_base` (`./mount-backend.sh restart -w hive_base`); nothing has to fly.
  2. `./mount-backend.sh gendata --run test01 --count 20` (in `ros2_ws/src/swarm_backend/`). It prints progress every 10 images and a table of labelled objects by size at the end.
  3. Look through `data/gen/test01/images/` and compare a few with `labels/` (or `meta/` for the boxes in pixels).

  **Pass:** 20 images with different camera heights and directions (sky, trees, ground), some brighter and some dimmer/warmer (dusk), about 3 empty; drones of very different sizes, some only a few pixels; each `.txt` has one line per drone you can see. **Note here:** the images per second it printed (decides how long 1000 images take), whether the light visibly changes (if never: the `light_config` service didn't work; paste any error), and anything odd (drones inside trees, under the ground, cut at the edge). If it looks good, try `--count 200`.

- [ ] **snap saves birds and boxes; YOLO labels from them** (added 17 Oct)
  1. Gazebo with drone 2 in the air looking at drone 0 (e.g. scenario 1), and a bird in view: `./mount-backend.sh bird spawn dove --at 15 5 30` (adjust so drone 2's camera sees it; `view 2` shows it).
  2. Move the old files out of `data/Manualimages/`, then `./mount-backend.sh snap 2 --count 5`. Each printed line should now list the bird too (`bird_1: 18.3 m`).
  3. `./mount-backend.sh yololabels data/Manualimages`, then open one `.json` and its `labels/<name>.txt`.

  **Pass:** the `.json` has an `objects` list with drone 0 (`class: drone`) and `bird_1` (`class: bird`), each with a `box` around where it shows in the image; the `.txt` has one line per object in view (class 0 = drone, 1 = bird). **If the bird is missing** from `objects`: Gazebo's `pose/info` topic did not give its pose in time; note it here. Also check `./mount-backend.sh predict` still runs on the new files.

- [ ] **Box test with the in-flight bird, and the dove's heading** (added 17 Oct)
  1. `./mount-backend.sh boxtest --bird mourning_dove --out /workspace/data/box_test_mourning`, then open `data/box_test_mourning/003m_boxes.png` and `010m_boxes.png`.
  2. `./mount-backend.sh bird spawn dove --name bird_t --at 10 0 40 --yaw 90`, look in Gazebo, then `bird remove bird_t` (try `--yaw -90` if it faces south).

  **Pass:** the mourning dove has spread wings with a green box on it; note whether it faces north (tail towards the camera) or sideways. Note the `--yaw` that makes the dove face north, so it goes into `KINDS` in `birds.py`.

- [x] **Bounding-box camera gives training labels** (added 17 Oct; see [[Training data and YOLO training]]). Run by Claude on 17 Oct in `hive_base`: boxes sit on the x500 and the dove; `empty` has none; `overlap` gives the dove a smaller visible box (7×4) than full box (9×5). Boxes disappear at 2–3 px (drone: box at 60 m, none at 100 m). Our projected drone box matches the full box (95×42 vs 95×48 at 3 m). x500_gimbal camera: clip far 15 000 m, no noise, no anti-aliasing setting. `pose/info` lists static birds at full rate. The dove mesh is a perched bird with folded wings and faces east at heading 0. Images in `data/box_test/`.
  1. Gazebo running, `hive_base` world preferred (trees in the background): `./mount-backend.sh restart -w hive_base`. Drone 0 alone is enough; nothing has to fly.
  2. `./mount-backend.sh boxtest` (in `ros2_ws/src/swarm_backend/`). It first prints the x500_gimbal camera's clip planes, anti-aliasing and noise; then spawns a bare camera 40 m up at the origin looking north, a static x500 (label 1) and a dove (label 2), moves them to 3, 5, 10, 20, 40, 60, 100 and 150 m, prints a table and removes them. Options: `--height`, `--bird mourning_dove`, `--keep`.
  3. Open `data/box_test/*_boxes.png` (green = visible box, red = full box, blue = the box we project ourselves from `HALF_SIZE` in `ground_truth.py`, yellow cross = where the pinhole model puts the model's origin). The table's `proj w x h` column is the blue box.

  **Pass:** all three models spawn; green boxes sit on the drone and the dove in every step where they are visible, down to 2–4 px (60–100 m); in `overlap` the dove's green box is smaller than its red one; `empty` has no boxes. **Note here:** the printed clip far / anti-aliasing / noise of x500_gimbal, the smallest distance at which a box is missing, and whether the box width is close to "expected w", and how far the blue box is from the red one (if it is clearly too big, too small or shifted, note it: `HALF_SIZE` gets adjusted from this). **If the drone fails to spawn:** try `--drone-model x500`. **If there are no boxes at all:** paste the output; then the fallback is projecting 3D boxes ourselves.

- [ ] **Birds look right in Gazebo** (added 14 Oct)
  1. Gazebo running with drone 2 (the gimbal drone) in the air, e.g. scenario 1, and `./mount-backend.sh view 2`.
  2. Put birds near drone 0 one at a time (the old three-bird `scenarios/birds.sh` is now the birds-flying-in-with-the-hostiles scenario, see the next test): `./mount-backend.sh bird spawn dove --at 15 5 30`, or `--near 0 --at 3 0 1` for relative to drone 0.
  3. Look at them in the Gazebo window, next to a drone (0.5 m across).

  **Pass:** each bird is about drone-sized (the flock: five birds about that size), upright with its wings level, and facing its heading. **If not:** remove it (`bird remove bird_1`) and spawn again with `--scale`, `--roll 0` (if it stands on its side) or `--yaw 90/180` (if it flies sideways), and note the values that work here, so they can go into `KINDS` in `birds.py`. Then try `bird wander bird_1 --around 15 5 30` and watch whether `detect 2` picks it up.

- [ ] **Birds scenario: birds fly in with the hostiles** (added 11 Nov)
  1. Gazebo with drone 0 in the hive_base world (`docker/start-stellar.sh`), then `scenarios/birds.sh` (the mpc scenario plus `--birds 10`; `--birds N` for another number, `--bird-seed S` to repeat the start points).
  2. Expect: the log lists ten birds of all seven kinds 120-175 m out. They start around the two hostiles and fly level over the base at about the hostiles' speed, then on past it.
  3. Watch the detector picture (`/px4_1/detection/debug`): do birds get boxes, and does the lock stay on a drone? The summary's "reports matching no hostile" counts reports that were not on a drone.
  4. After the run the birds are gone from Gazebo.
- [ ] **The join: Max's attack flying on our detector** (first runs on 14 Oct: head-on hit, crossing missed; see [[Join test results 2026-10-14]]) (added 14 Oct, after merging both into `dev`)

  This is the most important test now: the "when we meet again" step of [[Baseline attack work plan]]. Run it on `dev`. Use **two terminals**, both in `ros2_ws/src/swarm_backend/`.

  **Step 1. Start a light simulation** (in terminal A). Only drones 0 (the hostile) and 2 (the defender, with the gimbal camera) are started, which keeps Gazebo faster.

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh restart 3
  ```

  Run the same `export` line in terminal B too, before `attacktest`. The `export` tells `mount-backend.sh` to read `scenarios/attack.yaml` instead of `config/swarm.yaml`, and it only lasts in the terminal where it was typed. `restart` and `attacktest` need it; `detect` does not read the config, so it doesn't matter there.

  **Step 2. Fly the attack with Max's fake detector** (terminal B). This checks the attack on its own. **14 Oct: hit** (1.45 m).

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh attacktest --pattern offset
  ```

  `offset` is the easiest path: the hostile flies straight toward the defender, 8 m to its side, at the same height.
  **Pass:** the run ends with `hit`.

  **Step 3. Fly the same attack on our detector.** **14 Oct: `headon` hit** (1.09 m).
  Terminal A:

  ```bash
  ./mount-backend.sh detect 2 --rate 20
  ```

  Terminal B:

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh attacktest --pattern headon --external-detector
  ```

  `--external-detector` turns the fake off, so the attack only sees what `detect 2` publishes. Use `headon` or `crossing` here, not `offset`: in `offset` the hostile flies at the defender's own height, against the horizon and ground, where the baseline can't see it (Max's code says `offset` is for the fake detector only). The 14 Oct `offset` run showed it: found only at 11–14 m and 35–43° to the side, then lost in the first turn.
  **Watch:** terminal A shows `confirmed: T…` with a position while the hostile is in view; terminal B goes SEARCH → ATTACK.
  **Pass:** the run ends with `hit`, or at least gets about as close as with the fake.

  **Step 4. The harder path** (crossing in front of the defender), with the fake and with ours. **This is what is left:** on 14 Oct our detector missed by about 4.4 m with gain 0.5 and with 1.0, because the lock kept dropping; the fake has not flown it yet. Add `--save /workspace/data/detect_frames` to `detect 2` so the frames show why the lock drops:

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh attacktest --pattern crossing
  ./mount-backend.sh attacktest --pattern crossing --external-detector     # with detect 2 running
  ./mount-backend.sh attacktest --pattern crossing --external-detector --gain 1.0
  ```

  `--gain 1.0` is Max's open question: in his simulation the plan's gimbal gain of 0.5 missed crossing targets and 1.0 hit.

  **Results:** every run adds one row to `data/attack_runs.csv` in the repo. The `detector` column says which detector it was (fake or external = ours), so fake and real runs can be compared row by row.

  **If it fails, look at terminal A first:**

  | What you see                                     | Where the problem is                                                                                      |
  | ------------------------------------------------ | --------------------------------------------------------------------------------------------------------- |
  | Never`confirmed`                                 | Detection: the hostile is not against the sky, or too small (far away) or too big (closer than about 5 m) |
  | `confirmed`, but terminal B never goes to ATTACK | The topics don't reach the attack node (topic name or drone number)                                       |
  | ATTACK, but no hit                               | The attack itself, or the gimbal gain                                                                     |

  Send Claude the output of both terminals, or the CSV.

- [ ] **Speed after task 5** (added 14 Oct)
      Scenario 1 and `./mount-backend.sh detect 2` running; read `ms/image`. **Pass:** well under 200 ms (about 30 ms on saved images in the container, against 218 ms before). Then try `--rate 20`: with the detector this fast, the default limit of 10 images per second may be what holds it back.
- [ ] **The lock holds while the gimbal turns** (added 14 Oct, task 4)
  1. Scenario 1 running (the chaser turns its gimbal all the time), and `./mount-backend.sh detect 2`.
  2. Watch the `confirmed: T<number>` part of the lines while the target is in view.

  **Pass:** the same track number stays while the gimbal turns, and target_offset keeps coming. **Fail:** the number keeps changing (T1, T2, T3 …) or `confirmed: none` every time the gimbal moves. Then try `--gate-deg 15`; and compare with `--no-px4` (old pixel matching), which should lose the lock more often.

- [ ] **target_position during a chase** (added 14 Oct, task 3)
  1. Scenario 1 running, and `./mount-backend.sh detect 2`.
  2. Third terminal: `docker exec -it px4-gazebo-vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /px4_2/detection/target_position"`
  3. During the chase, also `./mount-backend.sh snap 2 --count 20 --every 0.5`, then `./mount-backend.sh predict` and read the `target_position (PX4 orientation) off by …` parts.

  **Pass:** messages only while a target is confirmed; the direction off by under 2°; the position off mostly in distance (within about 25 %). Checked by Claude only with both drones hovering (0.2–1.6 m off at 17 m).

- [ ] **Debug picture shows in RViz** (added 14 Oct, after the fix below)
  1. `./mount-backend.sh detect 2` running (restart it, so it has the fix).
  2. `./mount-backend.sh view 2`, then Add → By topic → `/px4_2/detection/debug` → Image.

  **Pass:** the picture shows, with the white centre cross; no "incompatible QoS" warning in the detect terminal. Then redo the sign check: target right of the cross → x positive, above it → y positive, also when the picture is tilted.

- [ ] **Detection flight test with the new detector** (added 13 Oct)
  1. `./mount-backend.sh restart 3` (Gazebo with drones 0–2 from swarm.yaml: drone 2 is the gimbal drone, drone 0 the target). Close the shell and any scenario first.
  2. `./mount-backend.sh detecttest` (it flies drones 0 and 2 itself).

  It was written for Olle's old detector and has not been flown with ours. It times itself on PX4's clock, so PX4's clock jumps (see [[Join test results 2026-10-14]]) can make it time out. **14 Oct attempt:** aborted at the start with "No data from drones [2]", because only drone 0 was running (a plain `restart` starts only drone 0); nothing was measured.

  **Pass:** it runs to the end and prints the table of detection rate per 10 m. **Note** at which distance detection stops.

## Done

Run on Stellar (vultaumar) on 24 Nov by Oskar and Claude: the demo recording, from [[Recording demo videos]].

| Test | Result | Verdict |
| --- | --- | --- |
| record.sh and final, one run (`claude-test4`) | Every part ready in under a minute (sim 20 s, UI 5 s, poses 5 s, YOLO 3 s, cameras 10 s), every health line ✓, hit by drone 3 seen at 72 m, 5.7 s to the hit; preview cut in 34 s, all panels on the same sim time. Fixed on the way: `run.json` was never written (argparse), Ctrl-C waited 300 s for the landing (now 20 s), Gazebo rendered the cameras on the CPU (now `--headless-rendering` on the GPU), the model folder. | **Pass** |
| The UI film (`--screen`, `demo-ui3`) | The kiosk opens on the leftmost monitor and only that one is filmed (it filmed both before). The UI lines up with the cameras and the map in the film. Fixed on the way: false targets during takeoff (the detectors now start at patrol height, drone 0 is a friend), `first_sighting` counted the hostile parked before its release. | **Pass** |
| The final film and extras (`demo-ui3`, `stories/extras.yaml`) | 1920x1080, 55 s, 8 MB in 45 s; zoom on the first locked box with the hostile inside, frozen 1.5 s; the range graph reaches 0 at the hit; insight labels at the sighting; `.srt` matches; stills PDF one page per scene; hit GIF 1.2 MB. `--burn-subtitles` drew the captions a second time, huge: removed (the captions are in the picture already). | **Pass** |
| Demo videos after removing the Chromium render | The image builds, the 83 UI/camera/screen/debug tests pass, `events` prints the table, `cam` with and without `--plain` writes its clip (on `claude-test4`; the 18 Nov run is not on this machine), the UI looks as before. | **Pass** |
| `--watch` (Gazebo's window) | Works, but the sim drops to 0.17x: the cameras then render without the GPU. Use the UI in the browser to watch a run. | **Works, slow** |

Run by Oskar on 14 Oct; checked by Claude the same day.

| Test                                                       | Result                                                                                                    | Verdict                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Camera roll follows the drone in a chase                   | Roll went −2 → +14 → +4° through a turn, and to −19° in another                                           | **Pass**                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Orientation error under motion                             | Where it looks: median 0.42°, worst 2.68°. Roll: median 0.26°, worst 5.48°, over 8 images                 | **Mostly pass.** 6 of 8 images under 0.7°; the two outliers (…087.748: 2.7°, …088.804: 1.9° and 5.5° roll) were during fast turns. Likely cause: PX4 reports the gimbal only 5 times a second and the gimbal jumps between reports, so the value in between is guessed. Effect: target_offset barely changes (5° roll error at a 5° offset ≈ 0.4°); target_position's direction can be ~1 m off at 20 m. Not a blocker; improve later if PX4 can report faster |
| target_offset signs on the debug picture                   | RViz showed "No image"; detect warned "incompatible QoS … RELIABILITY"                                    | **Fail, fixed:** the debug picture was sent best-effort, and RViz asks for reliable. It is now sent reliable (a reliable subscriber received it in a test, no warning). Re-test above                                                                                                                                                                                                                                                                          |
| target_offset signs, from saved frames (`detect 2 --save`) | One detection (…155.96): target above and a little right of the centre cross; published x −2.5°, y +17.5° | **Pass.** In the picture it is +1.3° right and +17.6° up. The camera was rolled about 12° left side down (the horizon slopes down to the right), so in the world "up in the picture" leans left: x −2.5° is the roll correction working. RViz part still to do                                                                                                                                                                                                 |
| Speed with the scenario running                            | About 210 ms per image (164–274)                                                                          | **Below goal** (200 ms): about 4.7 images per second. Task 5 (faster filter) addresses it                                                                                                                                                                                                                                                                                                                                                                      |

### Raw notes from the run

1. seems to be working normally:

\*\* \*\*camera orientation from PX4: on

**Running. Ctrl-C to stop.**

**[70.69] 0 candidates, confirmed: none; camera roll -2 deg** **(233 ms/image)**

**[70.76] 0 candidates, confirmed: none; camera roll +0 deg** **(240 ms/image)**

**[70.95] 0 candidates, confirmed: none; camera roll +4 deg** **(179 ms/image)**

**[71.15] 0 candidates, confirmed: none; camera roll +6 deg** **(210 ms/image)**

**[71.22] 0 candidates, confirmed: none; camera roll +7 deg** **(234 ms/image)**

**[71.48] 0 candidates, confirmed: none; camera roll +12 deg** **(242 ms/image)**

**[71.58] 0 candidates, confirmed: none; camera roll +13 deg** **(164 ms/image)**

**[71.68] 0 candidates, confirmed: none; camera roll +14 deg** **(219 ms/image)**

**[71.88] 0 candidates, confirmed: none; camera roll +11 deg** **(274 ms/image)**

**[71.94] 0 candidates, confirmed: none; camera roll +10 deg** **(209 ms/image)**

**[72.04] 0 candidates, confirmed: none; camera roll +9 deg** **(202 ms/image)**

**[72.11] 0 candidates, confirmed: none; camera roll +7 deg** **(246 ms/image)**

**[72.24] 0 candidates, confirmed: none; camera roll +4 deg** **(231 ms/image)**

2. results:

**PX4 camera orientation, where it looks: median 0.42 deg, worst 2.68 deg over 8 images**

**PX4 camera orientation, roll: median 0.26 deg, worst 5.48 deg over 8 images**

3. Says NO IMAGE in Rviz, also got this in terminal: **[WARN] [1791999065.768667836] [detect_2]: New subscription discovered on topic '/px4_2/detection/debug', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY**
4. around 200ms per image:

**[126.36] 1 candidates, confirmed: none; camera roll -1 deg** **(229 ms/image)**

**[126.46] 0 candidates, confirmed: none; camera roll -5 deg** **(208 ms/image)**

**[126.69] 0 candidates, confirmed: none; camera roll -14 deg** **(216 ms/image)**

**[126.82] 0 candidates, confirmed: none; camera roll -19 deg** **(225 ms/image)**

**[127.02] 0 candidates, confirmed: none; camera roll -19 deg** **(231 ms/image)**

**[127.32] 0 candidates, confirmed: none; camera roll -8 deg** **(212 ms/image)**

**[127.45] 0 candidates, confirmed: none; camera roll -5 deg** **(212 ms/image)**

**[127.65] 0 candidates, confirmed: none; camera roll -2 deg** **(217 ms/image)**

**oskarrask@oskars-air-2 swarm_backend % ./mount-backend.sh detecttest **

**[INFO] [1791999322.603837886] [detection_test.drone0]: Using topics under "/", system id 1**

**[INFO] [1791999322.608481761] [detection_test.drone2]: Using topics under "px4_2", system id 3**

**[INFO] [1791999325.082326179] [detection_test.drone0]: Connected to PX4**

**[INFO] [1791999325.084881971] [detection_test.drone0]: Pre-flight checks PASS**

**[ERROR] [1791999384.899287471] [detection_test]: Aborting (No data from drones [2]. Start them with ./mount-backend.sh spawn)**

**No images recorded.**
