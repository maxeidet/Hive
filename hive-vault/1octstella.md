# Intercept and lock-on: design notes (1 Oct 2026, Stellar)

Notes from a design session about the chase/intercept action in `swarm_backend`: what was built, what the simulations showed, and what the real scenario means for guidance and sensors. The code lives in `ros2_ws/src/swarm_backend/swarm_backend/intercept/`; how to run it is in the package README (section "Intercept").

---

## 1. What exists now

One friendly drone chases one hostile drone. Both are real PX4 drones in Gazebo, at a fixed altitude of 50 m. The interceptor only knows what the base station's (mock) radar reports. Every run is scored against the hostile drone's true position.

```
hostile PX4 drone ──► MockRadar ──radar_msgs/RadarScan /base/radar/hostile (2 Hz)──►
   RadarTarget ──► KalmanTracker (x, y, vx, vy + uncertainty) ──► ChaseAction.step() @ 10 Hz
        ├─ phases: LOST → INTERCEPT → SHADOW
        ├─ position setpoint + velocity feed-forward (fixed altitude, nose toward target)
        └─ gimbal aimed at the estimate (pitch relative to the horizon)
```

| Part | What it does |
|---|---|
| `paths.py` | Hostile patterns at constant speed: `line` (racetrack), `circle`, `figure8`, `random` (seeded, minimum turn radius 8 m). No instant reversals, which no real drone can fly. |
| `mock.py` | `HostileFlight` flies the hostile drone along a pattern; `radar_fix` is what the radar measures. |
| `radar.py` | Mock radar publisher and the `RadarTarget` source (ROS). |
| `target.py` | `TargetSource` interface and fix types: `RadarFix` (polar) and `PositionFix` (for the drone's own sensors later). The chase uses the first source with a fresh fix, so a sensor source can be put in front of the radar. |
| `tracker.py` | Kalman filter; radar returns go in as measured (EKF on range, azimuth, radial velocity). |
| `guidance.py` | Lead point, shadow point, braking approach, phases, gimbal angles. Pure math. |
| `chase.py` | `ChaseAction`, one step per setpoint tick. |
| `tests/intercept_test.py` | Flight test in Gazebo with a summary (time to catch up, standoff error, closest approach, radar and tracker error, gimbal pointing error) and an optional CSV. |
| `tests/test_intercept.py` | Unit tests, including a closed-loop chase simulation (speed- and acceleration-limited drones). |

Run it: `./start-stellar.sh`, then `./mount-backend.sh spawn` and `./mount-backend.sh intercept [--pattern figure8 --csv /tmp/run.csv]`. In the shell: `takeoff(50)`, `hostile()`, `chase()`, `status()`, `stop_chase()`.

Timing uses each PX4's own clock (simulation time), so speeds and rates stay right when Gazebo runs slower than real time.

**Troubleshooting:** "No data from PX4 for drones [0, 2]" while the drones are running means Gazebo is **paused**. PX4 runs in lockstep with the simulation clock and never finishes starting. Press ▶ in Gazebo, or:
`docker exec px4-gazebo-vnc gz service -s /world/hive_base/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'`

---

## 2. The follow algorithm

**Predictive pursuit to a standoff point** (lead pursuit plus following a virtual target). This is not proportional navigation.

1. **Estimate.** The Kalman filter turns noisy radar returns into position, velocity and uncertainty, and predicts between returns.
2. **Where to go.**
   - Far away (more than 3 × standoff): aim behind the predicted meeting point. The meeting time solves `t = |target + v·t − me| / my_speed`.
   - Close in: aim behind the target itself. The aim point is limited to 45° round the target from where we are, so if the target turns back we circle round it instead of cutting across.
   - Never closer than the minimum separation.
3. **How to get there.** The position setpoint is placed so the closing speed is `min(max_speed, √(2·a·d), K·d)` (a = 2 m/s²), with the target's velocity as feed-forward. The drone brakes in time instead of overshooting.
4. **Phases.** INTERCEPT → SHADOW within standoff + 3 m (back to INTERCEPT only beyond 2 × standoff). LOST (hover) after 3 s without a return.

Problems found in simulation and fixed: overshoot at capture, cutting in front of the target's path, and patterns with instant 180° turns.

**Limitation:** shadowing from behind only works against a target slower than the interceptor (see section 5).

---

## 3. Radar model

| | 1 σ |
|---|---|
| Azimuth (horizontal angle) | 1° (35 m across the line of sight at 2 km) |
| Range | 2–5 m at 2 km (3.5 m used) |
| Elevation | 2° (±70 m at 2 km; unused while altitude is fixed) |
| Radial velocity | 0.5 m/s, along the line of sight only |

- The error is a long, thin ellipse across the radar's line of sight, not a ball. The tracker models that directly.
- `radar_position` in `swarm.yaml` places the radar. The default `[-2000, 0, 0]` puts it virtually 2 km away, so the errors are realistic while the drones fly near the origin.
- Messages: `radar_msgs/RadarScan` (added to the Dockerfile as `ros-jazzy-radar-msgs`), frame `base_radar_ned`, azimuth clockwise from north.

**Results (closed-loop simulation):**
- Radar 200 m away: about 10 m standoff, at least 4 m separation, mean standoff error 1.2–1.8 m.
- Radar 2 km away: the filtered estimate is still about ±15 m (1σ). The standoff is scaled to 3σ (about 45 m), but a target that turns back still sometimes passes within metres.

**Conclusion:** the radar alone cannot keep a same-altitude chase safe. Getting close needs the drone's own sensor. An open option is a vertical offset (15–20 m above the target) while only the radar is available.

---

## 4. Detection and lock-on with onboard sensors

### Sensors (as modelled in our Gazebo)

| Sensor | Specs | Verdict |
|---|---|---|
| 3D lidar (`x500_lidar_3d`) | 16 beams, ±15°, 2° apart (none at 0°), 0.4° horizontal, 100 m | Unreliable on a small drone at the same altitude: beams pass just above and below it |
| Depth camera (OAK-D, `x500_depth`) | 640×480, 73° FOV, 0.2–19.1 m, 30 Hz | Excellent up close; fixed to the nose |
| Gimbal RGB (`x500_gimbal`) | 1280×720, 115° FOV, 30 Hz | Bearing only; good for ML detection later |

### Depth camera range

- The simulator cuts off at 19.1 m (the far clip; can be changed in our own model).
- Pixels: f ≈ 433 px, so a 0.5 m drone is 22 px at 10 m, 11 px at 20 m, 5 px at 40 m, about 3 px at 70 m.
- Real stereo error grows with distance squared: σ_z ≈ z²·σ_d / (f·B). For an OAK-D Lite (7.5 cm baseline): ±0.8 m at 10 m, ±1.7 m at 15 m, ±3 m at 20 m. **Realistic useful range is about 10–15 m.**
- At 50 m altitude the ground is beyond the depth range, so the sky is empty and every valid depth pixel is a target. A first detector needs no ML: find the group of valid pixels and project it to 3D.

### Proposed: depth camera on the gimbal

- A custom model `x500_gimbal_depth`: a gimbal with a depth sensor on the camera link.
- The gimbal keeps the depth camera pointed at the target however the drone tilts.

### Proposed state: LOCKED (lock and follow)

```
LOST ──► INTERCEPT ──► SHADOW ──(3 consistent detections in 1 s)──► LOCKED
                         ▲                                            │
                         └────────── no detection for ~1 s ◄──────────┘
```

- **Getting in:** a detection only counts if it is close to the radar track and not where one of our own drones is.
- **In LOCKED:** radar ignored, the tracker restarted with the sensor (much tighter), gimbal aimed from the detections.
- **Wiring:** gz depth image → ros_gz_bridge → per-drone detector → `/px4_N/perception/detections` → `SensorTarget` in the chase's source list.
- **Care needed:** matching image timestamps to drone poses (different clocks); sensor render cost on the sim.

---

## 5. The real scenario

- Hostile drones are detected by the base-station radar at 2 km or more and fly in toward the base.
- Friendly drones are spread thinly on a **1 km perimeter**.
- The friendlies have about 1 km of the hostile drone's flight to intercept and lock on.
- Speeds: the interceptor is an x500 at **12 m/s** (PX4 default `MPC_XY_VEL_MAX`); the hostile drone is 5 m/s faster at **17 m/s**.

### Meeting geometry (hostile drone flies straight in from 2 km; d = how far sideways the nearest friendly is)

| d | Meeting after | Meeting at (from base) | Closing speed | Interceptor heading |
|---|---|---|---|---|
| 0 m | 34.5 s | 1414 m | 29.0 m/s | straight out (head-on) |
| 200 m | 36.2 s | 1385 m | 28.2 m/s | 27° |
| 400 m | 41.5 s | 1295 m | 26.0 m/s | 54° |
| 600 m | 51.2 s | 1130 m | 22.8 m/s | 78° |
| 700 m | 58.3 s | 1008 m | 20.9 m/s | 90° (crossing) |

Beyond about 700 m to the side, the hostile drone crosses the ring first. **The ring needs a friendly drone at least every ~1.4 km: 5 or more on the 6.3 km circle, with more for margin.**

### Time for lock-on (first-detection range ÷ closing speed of 21–29 m/s)

| Sensor | First detection of a 0.5 m drone | Time before the meeting |
|---|---|---|
| Depth camera | ~20 m | 0.7–1.0 s |
| 3D lidar | ~30–50 m | 1–2.5 s |
| Gimbal RGB, wide (115°) | ~50–70 m | 2–3 s |
| **Gimbal RGB, narrow (~20°, 1920 px)** | **~400–500 m** | **14–24 s** |

### Consequences

1. **Head-on or crossing intercept, not a chase from behind.** The hostile drone is faster, so we can't get behind it and shadow. Guidance must steer onto its path; proportional navigation applies once locked.
2. **Only a narrow-FOV gimbal camera gives a real lock-on window.** The radar points it: at the meeting point (about 1.3–1.4 km from the radar), the cross-range error is σ ≈ 23–25 m, which is ±10° at 3σ from 400 m, inside a 20° view.
3. **The camera and radar complement each other.** The camera gives a precise direction (≈0.1°, under 1 m at 400 m), which fixes the radar's weak cross-range axis; the radar gives range. Together: metre-level in every axis.
4. **There is no second chance.** After the pass, turning round takes the x500 about 8 s. The hostile drone is then about 100 m ahead and pulling away by 5 m/s. Only the long-range camera keeps it in view.
5. **Depth camera and lidar only matter for the last ~20 m** (terminal guidance, measuring the closest pass).
6. **Timeline from radar detection:** about 20–40 s flying out on radar, about 15–20 s of camera lock, then the pass.

### Proposed state machine for the real scenario

```
ASSIGNED ──► INTERCEPT (radar: fly to the predicted meeting point)
         ──► ACQUIRE (gimbal points along the radar estimate; narrow camera searches)
         ──► LOCKED (camera bearing + radar range; PN onto the target)
         ──► TERMINAL (< 20 m: depth / lidar)
```

The base station assigns the friendly drone with the earliest feasible meeting time; that's the geometry above, run per drone.

---

## 6. Next steps / open questions

- [ ] Head-on intercept guidance (meeting point on the hostile drone's path; PN once locked), alongside the current shadow mode.
- [ ] Hostile drone at 17 m/s in the sim: raise `MPC_XY_VEL_MAX` on the hostile PX4 only (check the x500 handles the tilt), or move a kinematic Gazebo model along its path (no autopilot needed).
- [ ] A narrow-FOV (~20°) gimbal camera model, plus a detector (a simple sky-background blob first, ML later).
- [ ] Fuse camera bearing with radar range in the tracker (a bearing-only `Fix` type).
- [ ] Gimbal depth camera model for the terminal phase.
- [ ] A 2+ km world or scenario for full-scale runs.
- [ ] Base-station assignment: earliest feasible meeting time per drone.
- [ ] Decide: vertical offset during radar-only phases?
- [ ] Decide: what an intercept ends with (lock and track/escort, or neutralization). That chooses between PN to contact and holding a standoff.
- [ ] Hostile altitude: fixed for now; once unknown, the radar's 2° elevation (±70 m at 2 km) becomes the weak axis.
- [ ] Not yet verified in Gazebo: the polar radar + Kalman tracker version (needs a container restart for `radar_msgs`), and whether the gimbal fully removes the drone's tilt during hard acceleration.

Related: [[Home]], [[Codebase map]], [[ai_components]], [[px4_ros2_drone_swarm_development_roadmap]], [[Lock-on brainstorm]]
