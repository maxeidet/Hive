# Manual tests to do

Tests that need a person and a running simulation, for the detection work. Claude adds a test here whenever something is built that it could not check fully itself; tick it off (and note what you saw) when done. Newest first.

Back to [[Home]]. How to run things: [[Tutorial - live detector (detect)]], [[Manual - scenario 1, images and prediction]].

---

## To do

- [ ] **Birds look right in Gazebo** (added 14 Oct)
  1. Gazebo running with drone 2 (the gimbal drone) in the air, e.g. scenario 1, and `./mount-backend.sh view 2`.
  2. `scenarios/birds.sh` puts all three a few metres around drone 0 (`--remove` takes them out, `--wander` also flies the dove around). Single birds: `./mount-backend.sh bird spawn dove --at 15 5 30`, or `--near 0 --at 3 0 1` for relative to drone 0.
  3. Look at them in the Gazebo window, next to a drone (0.5 m across).

  **Pass:** each bird is about drone-sized (the flock: five birds about that size), upright with its wings level, and facing its heading. **If not:** remove it (`bird remove bird_1`) and spawn again with `--scale`, `--roll 0` (if it stands on its side) or `--yaw 90/180` (if it flies sideways), and note the values that work here, so they can go into `KINDS` in `birds.py`. Then try `bird wander bird_1 --around 15 5 30` and watch whether `detect 2` picks it up.

- [ ] **The join: Max's attack flying on our detector** (first runs on 14 Oct: head-on hit, crossing missed; see [[Join test results 2026-10-14]]) (added 14 Oct, after merging both into `dev`)

  This is the most important test now: the "when we meet again" step of [[Baseline attack work plan]]. Run it on `dev`. Use **two terminals**, both in `ros2_ws/src/swarm_backend/`.

  **Step 1. Start a light simulation** (in terminal A). Only drones 0 (the hostile) and 2 (the defender, with the gimbal camera) are started, which keeps Gazebo faster.

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh restart 3
  ```

  Run the same `export` line in terminal B too, before `attacktest`. The `export` tells `mount-backend.sh` to read `scenarios/attack.yaml` instead of `config/swarm.yaml`, and it only lasts in the terminal where it was typed. `restart` and `attacktest` need it; `detect` does not read the config, so it doesn't matter there.

  **Step 2. Fly the attack with Max's fake detector** (terminal B). This checks the attack on its own; it has only been shown on a simulated drone so far, not in Gazebo.

  ```bash
  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/attack.yaml
  ./mount-backend.sh attacktest --pattern offset
  ```

  `offset` is the easiest path: the hostile flies straight toward the defender, 8 m to its side, at the same height.
  **Pass:** the run ends with `hit`.

  **Step 3. Fly the same attack on our detector.**
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

  **Step 4. If step 3 works, try the harder path** (crossing in front of the defender), with the fake and with ours:

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

  It was written for Olle's old detector and has not been flown with ours. **14 Oct attempt:** aborted at the start with "No data from drones [2]", because only drone 0 was running (a plain `restart` starts only drone 0); nothing was measured.

  **Pass:** it runs to the end and prints the table of detection rate per 10 m. **Note** at which distance detection stops.

## Done

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
