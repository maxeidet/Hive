# Baseline attack work plan

The agreement between Oskar and Max (13 Oct 2026) for building the baseline attack in two halves that fit together without changes. Copied from Max's doc "Baseline attack: work plan", with Oskar's points to settle at the end. **The shared contract is the part that matters most: neither side changes it without telling the other.**

Back to [[Home]]. Related: [[Following the target with detection]], [[Detection implementation plan]].

---

## Who does what

- **Oskar: perception.** Turns camera images into the two topics below. His node never commands the drone; it only reads the camera, the gimbal's attitude and the drone's attitude.
- **Max: attack.** The node that flies the drone into the target with pure pursuit. He starts on a fake detector, so he does not wait for Oskar.
- Only Max's node sends commands to the drone.

## Shared contract

Two standard ROS topics. Oskar publishes them, Max's node reads them. No new message package.

| Topic                              | Type                           | Content                                                                                                                            |
| ---------------------------------- | ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| `/px4_N/detection/target_position` | `geometry_msgs/PointStamped`   | Target position relative to the drone, world NED, metres (= distance × direction)                                                  |
| `/px4_N/detection/target_offset`   | `geometry_msgs/Vector3Stamped` | x = horizontal angle from the image centre (rad, right positive), y = vertical angle (rad, up positive), corrected for camera roll |

Rules for both topics:

- Published **only when the target is confirmed** (seen k images in a row). No message means no confirmed detection.
- **Timestamp = when the image was taken** (sim time from the camera), not when the message was sent. Both topics carry the same timestamp for the same image.
- Conventions: NED (x north, y east, z down), metres, radians.
- **One target per image:** the confirmed track with the highest contrast.

Why two topics: `target_position` steers the drone (which way to fly). `target_offset` steers the gimbal; it is measured directly in the image, so it pulls the target to the centre even if the camera orientation estimate is a few degrees off.

## Oskar: perception tasks

| #   | Task                                                                                                          | Done when                                                                    |
| --- | ------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| 1   | Publish`target_offset` from the box: `atan((u − cx) / f)` and `atan((cy − v) / f)`, corrected for camera roll | The angles go to about 0 when the target is in the centre of the debug image |
| 2   | Camera orientation from PX4 (gimbal read-back + drone attitude) instead of the truth files                    | Under 2° error against the truth saved by`snap`                              |
| 3   | Publish`target_position`: direction from `camera.direction` × distance from `range_from_width`                | Within about 20 % in distance and 2° in direction against the truth          |
| 4   | Publish only confirmed tracks, the best one if there are several                                              | No messages while looking at empty sky                                       |
| 5   | (If time) Faster detection than today's 220 ms per image                                                      | At least 5 images per second                                                 |

**All five tasks done on 14 Oct**, merged into `dev`. How to run it: [[Tutorial - live detector (detect)]].

**Status (14 Oct, task 1):** `./mount-backend.sh detect 2` publishes `target_offset` for the best confirmed target, stamped with the image time, `frame_id` = the drone's Gazebo name, nothing without a confirmed target. Roll-corrected since task 2 (on the three scenario-1 images with a detection: 2–6° off without the correction, 0.5–0.6° with it). The detector is chosen with `--detector baseline|yolo`; everything after it only sees boxes and a score.

**Status (14 Oct, task 2):** built and checked on the running simulation. The camera's orientation is PX4's gimbal status quaternion, which the simulated gimbal reports in the earth frame with roll included; PX4's timestamps count from a different zero than Gazebo's, so they are matched through Gazebo's clock. Against the truth: 0.35° median and 1.6° worst while the chase flew, 0.19° while hovering. `snap` saves PX4's orientation and its error in each `.json` (`px4_camera`), and `predict` prints the error over all images.

**Status (14 Oct, task 3):** `target_position` is published with `target_offset` (same stamp and `frame_id`), only when the camera orientation from PX4 is known. Live test, drone 1 hovering 15 m north of and 8 m above drone 2: published N +14.8, E −0.1, D −8.6 m (17.1 m away); truth 17.0 m. Against the truth on 12 saved images: direction 0.3–0.6° off (2.5° once, in a fast turn), position 0.2–3.2 m off, almost all of it in the distance (±25 %). The distance jumps by 1–2 m from image to image at 17 m because the box size changes by a pixel; Max's pure pursuit only uses the direction, so this matters little for the baseline. It is relative to the camera, which sits 0.26 m below the drone's centre.

**Status (14 Oct, task 4):** only confirmed tracks are published, the highest score if there are several. Detections are now matched from image to image by world direction once PX4's camera orientation is known (gate `--gate-deg`, default 10°), so the gimbal turns by Max's node no longer reset the "k in a row" count; until then they are matched in pixels. Unit tested (a 6° gimbal turn that moves the target 30 px keeps one track); the live check is in the manual tests.

**Status (14 Oct, task 5):** the baseline's black-hat window is now square instead of round (`--round-kernel` brings the old one back): the same detections, about 30 ms per image in the container instead of about 210 ms. The live check is in [[Manual tests to do]].

## Max: attack tasks

| #   | Task                                                                                                                                                                                                             | Done when                                                     |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| 1   | Fake detector: publishes the same two topics from Gazebo's true positions, at 4 Hz, with optional delay (0.25 s), noise (1°, 20 % distance) and a blind zone below 5 m                                           | Its topics look exactly like the real ones                    |
| 2   | Attack node: the states below, with pure pursuit (velocity = max speed × direction to the target)                                                                                                                | Runs on the fake detector                                     |
| 3   | Limits: a minimum height and a speed limit on every command                                                                                                                                                      | Never below the minimum height, never above PX4's speed limit |
| 4   | Test scenario: the defender hovers; the hostile flies above it, through the camera's view, on a line or circle at 4 m/s. Hit = within 1.5 m from the true positions; the run then ends and the drones move apart | Hits with the fake detector, no noise                         |
| 5   | Logging: hit or not, time from first detection to hit, closest distance, time in each state, why the run ended                                                                                                   | One CSV row per run                                           |
| 6   | (If time) The same runs with noise and the blind zone turned on                                                                                                                                                  | The hit rate with realistic errors is known                   |

## Attack states

The gimbal keeps the target in the centre of the image during the attack, so the drone's own tilting and turning does not lose it.

```text
SEARCH ──first confirmed message──▶ ATTACK ──no message 0.5 s──▶ COAST ──none for 1.5 s──▶ LOST ──▶ SEARCH
                                       ▲                            │
                                       └──────message again─────────┘
ATTACK / COAST ──within 1.5 m──▶ HIT (the test ends the run)
```

| State  | Drone                                                  | Gimbal                                                          | Next state                                    |
| ------ | ------------------------------------------------------ | --------------------------------------------------------------- | --------------------------------------------- |
| SEARCH | Hovers at its guard position                           | Fixed, towards where the hostile comes from, tilted slightly up | First confirmed message → ATTACK              |
| ATTACK | Velocity = max speed × direction from`target_position` | Turned by 0.5 ×`target_offset` on each message                  | No message for 0.5 s → COAST                  |
| COAST  | Keeps its last velocity                                | Held still                                                      | Message again → ATTACK; none for 1.5 s → LOST |
| LOST   | Hovers                                                 | Points towards the last known direction                         | → SEARCH                                      |
| HIT    | The test ends the run (within 1.5 m)                   |                                                                 |                                               |

Coasting is needed because the detector cannot see the target closer than about 5 m: it is then too big for the detector's size limit (40 px). The drone covers the last metres on its last velocity.

## When we meet again

**First join done on 14 Oct:** head-on hits on the real detector, crossing misses by about 4 m. See [[Join test results 2026-10-14]].

Joining the halves needs no code changes, because both use the same topics.

1. Stop Max's fake detector and start Oskar's node.
2. Run the same test scenario as with the fake detector.
3. If it fails, compare both topics with the truth from Gazebo. The logs then show whether the problem is the angles, the distance or the attack.

## Open questions

From the plan, settled in the code (`scenarios/attack.yaml`, `tests/attack_test.py`):

- [x] How high the defender hovers, and how far above it the hostile flies: the defender guards at 20 m; in `headon` and `crossing` the hostile flies 10 m above it (`offset` is at the same height and only works with the fake detector).
- [x] Which direction the gimbal points during SEARCH: towards `search_heading_deg` (0 = north), tilted 10° up.
- [x] The drone number N: the defender is drone 2 (`/px4_2/detection/...`), the hostile drone 0.

Added by Oskar after reading it (14 Oct):

- [x] **Drone 0 has no prefix.** Settled as "the drone's own namespace": `/detection/...` for drone 0, `/px4_N/detection/...` otherwise, on both sides.
- [x] **Which clock the timeouts use.** Max's attack times the 0.5 s and 1.5 s timeouts on message arrival, not on the stamps, so PX4's clock jumps don't affect it.
- [x] **Turning the gimbal can break the confirmation.** Fixed in task 4: tracks are matched by world direction.
- [ ] **`frame_id`.** Not specified in the contract. The real detector sends the drone's Gazebo name (`x500_gimbal_2`), Max's fake sends `world_ned`; the attack does not read it, so nothing breaks. Agree on one with Max.
- [ ] **Suggested change: "highest score", not "highest contrast".** Contrast only exists for the baseline detector; YOLO gives a confidence. The detector publishes the confirmed track with the highest score (the baseline's contrast or YOLO's confidence). Topics, types and conventions stay the same. Still to agree with Max.
- [x] **Rate.** Task 5 done: about 30 ms per image, so `--rate` and the camera are now the limit, not the detector.

## The reader class on Max's side

Oskar's suggestion (14 Oct) of a thin class that reads the two topics so the attack logic works on plain numbers without ROS is what Max built: `DetectionInbox` in `intercept/attack.py` keeps the newest message of each topic with its stamp and arrival time, and `nodes/attack_node.py` fills it from the ROS callbacks. The topics stay the contract.
