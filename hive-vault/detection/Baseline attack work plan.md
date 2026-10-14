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

**Status (14 Oct):** task 1 is built and unit tested, not yet tried live (Docker was off). `./mount-backend.sh detect 2` publishes `target_offset` for the best confirmed target, stamped with the image time, `frame_id` = the drone's Gazebo name, nothing without a confirmed target. It is **not roll-corrected yet**: the roll needs task 2, and the node says so when it starts. On the three scenario-1 images with a detection, the offset was 2–6° off without roll correction (camera roll 9–30°) and 0.5–0.6° off with the true roll, so the correction formula is right and task 2 is what is missing. The detector is chosen with `--detector baseline|yolo`; everything after it only sees boxes and a score.

**Status (14 Oct, task 2):** built and checked on the running simulation. The camera's orientation is PX4's gimbal status quaternion, which the simulated gimbal reports in the earth frame with roll included; PX4's timestamps count from a different zero than Gazebo's, so they are matched through Gazebo's clock. Against the truth: 0.35° median and 1.6° worst while the chase flew, 0.19° while hovering. `target_offset` is now roll-corrected. `snap` saves PX4's orientation and its error in each `.json` (`px4_camera`), and `predict` prints the error over all images.

**Status (14 Oct, task 3):** `target_position` is published with `target_offset` (same stamp and `frame_id`), only when the camera orientation from PX4 is known. Live test, drone 1 hovering 15 m north of and 8 m above drone 2: published N +14.8, E −0.1, D −8.6 m (17.1 m away); truth 17.0 m. Against the truth on 12 saved images: direction 0.3–0.6° off (2.5° once, in a fast turn), position 0.2–3.2 m off, almost all of it in the distance (±25 %). The distance jumps by 1–2 m from image to image at 17 m because the box size changes by a pixel; Max's pure pursuit only uses the direction, so this matters little for the baseline. It is relative to the camera, which sits 0.26 m below the drone's centre.

If task 2 is not finished, publish `target_position` with the truth orientation for now and say so in the log, so nobody mistakes it for a real result.

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

Joining the halves needs no code changes, because both use the same topics.

1. Stop Max's fake detector and start Oskar's node.
2. Run the same test scenario as with the fake detector.
3. If it fails, compare both topics with the truth from Gazebo. The logs then show whether the problem is the angles, the distance or the attack.

## Open questions

From the plan, to settle together:

- [ ] How high the defender hovers, and how far above it the hostile flies, so the target always has sky behind it.
- [ ] Which direction the gimbal points during SEARCH.
- [ ] The drone number N in the topic names for the test scenario.

Added by Oskar after reading it (14 Oct), worth settling before coding:

- [ ] **Drone 0 has no prefix.** PX4 drone 0 publishes on `/fmu/...`, not `/px4_0/...`. Either the defender is never drone 0, or the rule is "the drone's own namespace" (`/detection/...` for drone 0).
- [ ] **Which clock the timeouts use.** The stamp is Gazebo's camera time, and PX4's clock can differ from it. If the 0.5 s and 1.5 s timeouts compare stamps against the drone's clock they can fire wrongly. Timing on message arrival is safest.
- [ ] **Turning the gimbal can break the confirmation.** The k-in-a-row check matches detections in pixels with a 20 px gate. When the attack turns the gimbal by half the offset, the target can jump further than that, the count restarts and no messages go out. Perception should match by world direction (or widen the gate) — part of Oskar's task 4.
- [ ] **`frame_id`.** Not specified. Suggestion: the drone's name, with NED stated here, so nobody reads the numbers as ENU.
- [ ] **Suggested change: "highest score", not "highest contrast".** The contract says one target per image, "the confirmed track with the highest contrast". Contrast only exists for the baseline detector; YOLO gives a confidence. Since perception will let the detector be swapped with one option (baseline or YOLO), the rule should read "the confirmed track with the highest score", where the score is the baseline's contrast or YOLO's confidence. The topics, types and conventions stay exactly the same. To agree with Max.
- [ ] **Rate.** The fake detector runs at 4 Hz and the real one at about 4 images a second. Task 5 is cheap (a square black-hat window: 39 ms → 2.5 ms for that step), so Oskar does it early rather than "if time".

## Idea, not agreed yet: a thin reader class on Max's side

_Oskar's suggestion (14 Oct), to discuss with Max. Not part of the contract._

The topics stay the contract. Publish/subscribe fits here: the detector runs in its own process, so a slow image, a crash or a restart never stalls the attack node's setpoint stream (PX4 lands the drone below 2 setpoints a second). The fake and the real detector are interchangeable, and `ros2 topic echo` or a bag shows what passes between them. It is the same pattern the code already uses for the radar (`MockRadar` publishes, `RadarTarget` reads).

The idea is to read the two topics through a small class, like `RadarTarget`, with one method such as `latest()` that returns the newest offset and position (or nothing when the last message is too old). The attack logic then works on plain numbers and never touches ROS, so it can be unit tested without the simulation.
