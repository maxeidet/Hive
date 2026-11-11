# Tutorial: the live detector (`./mount-backend.sh detect`)

How to run the detector on a drone's camera while the simulation flies, see what it finds, and read what it publishes for the attack node. This note is updated as the detector grows. Last updated 17 Oct 2026 (all five tasks of the work plan done).

Back to [[Home]]. Related: [[Baseline attack work plan]] (the topics agreed with Max), [[Manual - scenario 1, images and prediction]] (scenario 1, `snap`, `predict`).

---

## What it does

For every camera image (as fast as it can, up to `--rate` per second):

1. **Detect:** the chosen detector finds candidate targets in the image (boxes with a score).
2. **Confirm:** a candidate seen in `--k` images in a row, moving at most `--gate-deg` degrees in the world between images (`--gate` pixels until PX4's camera orientation is known), becomes a confirmed target.
3. **Publish:** for the best confirmed target (highest score), it publishes the topics below. With no confirmed target it publishes nothing.
4. **Debug picture:** the image with everything drawn on it, on its own topic.

It only reads the camera and never commands the drone, so it can run next to the shell or a scenario.

## Status

| Part | Status |
|---|---|
| `target_offset` (angles from the image centre) | Works, corrected for camera roll (from PX4) |
| `target_position` (where the target is, in metres) | Works. Direction 0.3–0.5° off; distance from the box size, so it jumps by ±1–2 m from image to image at 17 m (the box changes by a pixel) |
| Camera orientation from PX4 | Works: 0.2–0.4° typical, 1.6° worst against Gazebo's truth |
| Detector choice | `baseline` works, `yolo` not trained yet |
| Speed | About 30 ms per image in the container (was 210 ms): the detector is no longer the limit; `--rate` (default 10 per second) and the camera are |

## Run it

Everything runs in the container; nothing is installed on the Mac. From `ros2_ws/src/swarm_backend/`:

1. Start the simulation (`docker/start-dev.sh`) and something that flies a camera drone, for example scenario 1 (`--no-view` keeps it faster):
   ```bash
   scenarios/scenario1.sh --no-view
   ```
2. In a second terminal:
   ```bash
   ./mount-backend.sh detect 2
   ```
   `2` is the drone whose camera is used. It must be a gimbal drone (scenario 1 makes drones 1 and 2 gimbal drones).

When it starts it prints the camera topic, the topics it publishes and whether the camera orientation is read from PX4. Then, once a second, a line like this (from a test on 14 Oct: drone 1 hovering 15 m north of drone 2 and 8 m higher):

```text
[242.75] 1 candidates, confirmed: T1 at (665, 350) 12x10px score 75, offset x +3.5 y +1.4 deg, position N +14.8 E -0.1 D -8.6 m (17.1 m away); camera roll +0 deg  (241 ms/image)
```

| Part | Meaning |
|---|---|
| `[200.64]` | Simulation time of the image, seconds |
| `1 candidates` | Boxes the detector found in this image |
| `confirmed: T4 at (692, 352) 9x7px score 104` | Confirmed track 4: its centre pixel, box size and score. `none` if nothing is confirmed |
| `offset x +4.6 y +0.6 deg` | The published target_offset, in degrees: right and up positive, roll-corrected |
| `position N +14.8 E -0.1 D -8.6 m (17.1 m away)` | The published target_position: metres north, east and down (negative = above) from the drone, and the distance |
| `camera roll +0 deg` | The camera's roll from PX4 (right side down positive). `camera roll unknown` = PX4 has not reported yet (or its clock just jumped, see Known limits), and the offset is sent uncorrected |
| `241 ms/image` | Time per image (this example is from before task 5; now about 30 ms). Under 200 ms = more than 5 images per second |

Ctrl-C stops it.

## Options

| Option | Default | What it does |
|---|---|---|
| `--detector baseline\|yolo` | `baseline` | Which detector runs. Everything else stays the same |
| `--no-px4` | off | Don't read the camera orientation from PX4 (the offset is then not roll-corrected) |
| `--k N` | `3` | Images in a row before a target counts as confirmed. Higher = fewer false alarms, slower to react; `1` = every detection |
| `--gate-deg DEG` | `10` | How far (degrees, in the world) a target may move between images and still be the same target. Matching is by world direction once PX4's camera orientation is known, so turning the gimbal does not break the lock. Raise it for a fast target close by |
| `--gate PX` | `20` | The same in pixels, used only until PX4's camera orientation is known |
| `--rate HZ` | `10` | Most images processed per second |
| `--save DIR` | off | Save the raw image and a `_boxes.png` copy with the drawings, e.g. `--save /workspace/data/detect_frames` (= `data/detect_frames/` in the repo) |
| `--save-every S` | `1.0` | Simulation seconds between saves |
| `--horizon ROW` | off | Also ignore everything below this image row (360 = the middle, for a level camera). Not needed: the detector finds the sky by its colours |
| `--round-kernel` | off | The round black-hat window of the first version: the same detections, about 7 times slower |
| `--kernel PX` | `41` | Baseline detector: targets wider than this are not found |
| `--contrast N` | `60` | Baseline detector: how much darker than the sky around it a target must be (grey levels 0–255) |

For example, stricter confirmation, saving every half second:
```bash
./mount-backend.sh detect 2 --k 5 --save /workspace/data/detect_frames --save-every 0.5
```

## See the debug picture

Open RViz on the drone: `./mount-backend.sh view 2`. Click **Add → By topic → `/px4_2/detection/debug` → Image**.

| Drawn | Meaning |
|---|---|
| Yellow box with a number | Candidate; the number is how many images in a row it has been seen |
| Red box with `T<id> <size>px` | Confirmed target |
| Red text `x … y … deg` | The offset that was published for it |
| White cross | The image centre |
| Grey box `T<id> lost <n>` | ByteTrack: a lost track, drawn where its filter predicts it |
| Green box `bird <score>` | YOLO: a box classed as a bird. Shown only: never tracked, locked or published (added 11 Nov) |

## Read what it publishes

What the attack node gets. In a third terminal:

```bash
docker exec -it px4-gazebo-vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /px4_2/detection/target_offset"
```

An example message:

```yaml
header:
  stamp: {sec: 200, nanosec: 644000000}   # when the image was taken (simulation time)
  frame_id: x500_gimbal_2                 # the camera drone
vector:
  x: 0.128    # radians right of the image centre (7.3 degrees)
  y: 0.0097   # radians above the image centre
  z: 0.0      # unused
```

- Messages only come while a target is confirmed. Silence = no target.
- For drone 0 the topic has no prefix: `/detection/target_offset`.
- Radians to degrees: × 57.3.

Where the target is, in metres (north, east, down from the drone; z negative = above):

```bash
docker exec -it px4-gazebo-vnc bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /px4_2/detection/target_position"
```

```yaml
header: {stamp: {sec: 242, nanosec: 752000000}, frame_id: x500_gimbal_2}   # same stamp as target_offset
point: {x: 14.82, y: -0.13, z: -8.58}   # 14.8 m north, 0.1 m west, 8.6 m above
```

## Things to check

- **Signs:** target right of the white cross → x positive; above it → y positive.
- **No target, no messages:** silence while the target is in front of trees, behind the drone or out of view.
- **Speed:** the `ms/image` number.

## Known limits

- **Sky only.** The baseline finds the target against the sky, not in front of trees or ground. In scenario 1 the chaser flies level with the target, so it is mostly against trees and there are long gaps.
- **Not closer than about 5 m.** The target is then wider than 40 px and dropped.
- **The lock drops when the target crosses the view.** In the 14 Oct crossing runs the attack spent as long coasting or lost as attacking; why is not known yet ([[Join test results 2026-10-14]]).
- **PX4's clock jumps by 60–79 s now and then** ("PX4 clock jumped … (time sync?)"). The camera orientation is then unknown for about one image. Proposed fix: `UXRCE_DDS_SYNCT` = 0 in the container.

## Troubleshooting

| Message | What to do |
|---|---|
| `Drone 2 is not in Gazebo or has no camera` | Start a gimbal drone: run scenario 1, or `./mount-backend.sh spawn gimbal` |
| The camera picture is upside down, or a drone crashed | Restart Gazebo and the drones without restarting the container: `./mount-backend.sh restart`, then the scenario again |
| `The YOLO detector is not here yet` | Use `--detector baseline` (the default) |
| Nothing is ever confirmed | The target is not against the sky, or too far (under 3 px) or too close (over 40 px). Look at the debug picture |
| The count keeps resetting | Raise `--gate-deg` |
