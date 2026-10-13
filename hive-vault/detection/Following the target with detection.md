# Following the target with detection

How the camera detection can steer the drone, so that it follows the hostile drone without the radar telling it where the target is. A design to discuss, written 13 Oct 2026. Nothing here is built yet. It is step 4 of [[Detection implementation plan]] ("lock-on") worked out in more detail, and follows the ideas in [[Lock-on brainstorm]].

Back to [[Home]].

---

## The idea in one paragraph

The detector gives, for each image, where the target is in the picture and how big it is. Two control loops use that. A fast one turns the **gimbal** so the target stays in the middle of the picture. A slower one flies the **drone** toward the direction the camera is looking, and uses the target's size in the picture to keep a set distance. The radar is only used to get the target into view at the start, and as a fallback when the camera loses it.

## What we have and what is missing

| Piece | Status |
|---|---|
| Find the target in an image (`detect/candidates.py`) | Works against the sky |
| Confirm it over several images (`detect/tracking.py`) | Works, in pixels |
| Run on the camera in flight (`detect/live.py`) | Works, about 4 images per second on the Mac |
| Pixel → direction, size → distance (`detect/camera.py`) | Works, but only tested with the camera's true orientation |
| Camera orientation from PX4's gimbal status | Missing |
| Gimbal loop that centres the target | Missing (the chase aims the gimbal from the radar estimate) |
| Guidance that flies on a direction | Missing (the chase flies to a position) |
| Hand-over between radar and camera | Missing |

## The loops

```text
camera image ─▶ detector ─▶ confirmed target: pixel (u, v), size p
                                 │
            ┌────────────────────┴─────────────────────┐
            ▼                                          ▼
   gimbal loop (fast)                         direction filter
   pixel offset from the centre               pixel + camera orientation
   → turn the gimbal toward it                → direction in the world, smoothed,
                                                with its turn rate; predicts
                                                through short losses
                                                       │
                                                       ▼
                                              guidance (slower)
                                              fly along the direction;
                                              speed from the distance error
                                              (distance from the size p)
```

**Gimbal loop.** The offset of the box from the image centre is an angle: `atan((u − cx) / f)` sideways and the same for up and down. The loop adds a share of that angle to the gimbal's commanded yaw and pitch each image (a P or PI controller). The picture is rolled when the drone banks, so the offset has to be turned by the camera's roll before it is split into yaw and pitch. This loop is what the report means by "two PID controllers turn the gimbal so that the target stays in the centre of the image".

**Direction filter.** A small Kalman filter on the direction (azimuth and elevation and how fast they change). It smooths the detections, bridges a few missed images, and tells the detector where to expect the target next, so the right spot is picked when there are several. The detection is about 0.25 s old when it arrives, so the filter predicts forward to "now".

**Guidance.** Two ways, simplest first.

| | A. Fly on the direction (recommended first) | B. Feed the existing chase |
|---|---|---|
| How | Velocity command along the filtered direction. Speed = gain × (estimated distance − wanted distance), limited to the top speed. Distance from the box size, `f · 0.5 m / p` | Give the tracker a new "direction + rough distance" measurement next to the radar's. The chase code stays as it is |
| Needs | Only the camera | The tracker extended to 3D and to direction measurements |
| Good | Simple, no radar needed, is the report's baseline (pure pursuit) | Keeps the lead point and the "stay behind the target" logic; fuses radar range with camera direction |
| Weak | Chases where the target is, not where it is going; distance is rough (about ±20 %) | More work; the tracker today is flat (fixed altitude) and radar-shaped |

A is the right first version: it is small, it can be tested on its own, and it is what the report calls the baseline. B is the route to the later methods (proportional navigation, MPC), which need the turn rate of the direction or a position.

## States

```text
CUE ──target confirmed in the image──▶ LOCKED ──not seen for a moment──▶ COAST
 ▲                                        ▲                                │
 │                                        └────────seen again──────────────┤
 └──────────────────not seen for N seconds─────────────────────────────────┘
```

- **CUE:** fly and aim on the radar, as the chase does today. The detector runs and waits for a confirmed target near where the radar says it is.
- **LOCKED:** the camera drives the gimbal and the guidance. The radar is ignored, or only used for distance.
- **COAST:** the target was missed in the last images. Keep turning the gimbal and flying on the filter's prediction. After N seconds (start with 2), go back to CUE.

This is also the report's lock-on measure: target in view, confirmed, within 20 m, for 3 s without a break.

## Problems we already know about

- **The detector only works against the sky.** In scenario 1 the chaser flew level with the target and mostly saw it against trees. Follow from **behind and a few metres below**, so the camera looks slightly up and the target has sky behind it. This is a setting in the guidance (a height offset), and the biggest single thing for making the baseline usable.
- **Too close is also bad.** Above 40 px (closer than about 5 m) the detector drops the target. Keep the wanted distance at 10–15 m, where the target is 14–20 px.
- **Tracking in pixels breaks when the gimbal turns.** The confirmation should match detections by world direction instead. `tracking.py` already allows this; it needs the camera orientation.
- **The camera orientation comes from PX4, not from the truth.** First measure how good it is: compare the orientation PX4 reports with the one in the `.json` files that `snap` saves.
- **Speed.** About 4 images per second on the Mac. Enough for following at a distance, slow for a target crossing close by. The gimbal can be turned by the filter between images.
- **The drone's own propellers are in the picture** when the gimbal looks up. Candidates touching the image edge are already ignored; a fixed mask for the own arms may be needed.
- **Distance from size is rough.** It is good enough to hold a distance, not for a position. Radar range, or a second drone (triangulation), fixes that later.

## Order of work

| Step | What | Done when |
|---|---|---|
| 1 | Camera orientation from PX4's gimbal status, as forward / right / down vectors | The direction error against the truth files is known (aim: under 2°) |
| 2 | Gimbal loop on the detection | Target hovering in view: the gimbal brings it to the image centre and holds it while the camera drone is moved by hand |
| 3 | Direction filter, and tracking by world direction | The target stays confirmed while the gimbal turns; short losses are bridged |
| 4 | Guidance A with a distance and a height offset | Drone 2 follows drone 1 flying a circle at 4 m/s on the camera alone, 10–15 m behind and below |
| 5 | States: radar cue → locked → coast → cue | Scenario 1 with hand-over: starts on radar, switches to the camera, falls back when the target is hidden |
| 6 | Score it | Share of the time locked, longest lock, distance kept, against the radar-only chase on the same scenario |

Steps 1–3 do not fly the drone on the camera, so they are safe to try early. Each can be checked with what exists: `snap` truth files for step 1, the debug image of `detect` for steps 2–3.

## Open questions

- [ ] A first (camera only), then B, or straight to B because the tracker is needed anyway for the base station?
- [ ] Should LOCKED still use the radar's distance when there is one?
- [ ] Follow from below: how far below, and does that fit the report's fixed patrol altitude?
