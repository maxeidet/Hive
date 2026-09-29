# AI Components — Counter-UAS Drone Swarm

The project has three AI components, one per pair:

| # | Component | Runs on | Question it answers |
|---|---|---|---|
| **1** | **Base station** — sensor fusion and decisions | Base station | *Where is the hostile drone, and which of our drones should do what?* |
| **2** | **Lidar detection** of the hostile drone | Each drone | *Is there a hostile drone in my lidar data, and where?* |
| **3** | **Path to the hostile drone** — planning | Each drone | *How do I get to the hostile drone (or search for it) quickly and without hitting anything?* |

Every component has a **non-AI baseline** that is built first. The baseline keeps the whole system running end to end from early on, and it is what the AI version is measured against.

---

# How the parts work together

```text
        Simulation (Gazebo for validation / fast Python env for training)
  ┌──────────── scenario runner: spawn hostile drone, reset, score ────────────┐
  │ hostile ground truth (evaluation only)      lidar per drone    base radar   │
  └──────┬──────────────────────────────────────────┬──────────────────┬───────┘
         │                                          ▼                  │
         │              ┌── 2. LIDAR DETECTION (each drone) ──┐        │
         │              │ points → clusters → classifier      │        │
         │              └──────────────┬──────────────────────┘        │
         │                             ▼ /<drone>/perception/detections│
         │              ┌── 1. BASE STATION ───────────────────▼───────┐
         │              │ fusion: detections + radar → tracks          │
         │              │ decision: roles, interceptors, formation     │
         │              └──────────────┬───────────────────────────────┘
         │                             ▼ /base/assignments
         │              ┌── 3. PATH PLANNING (each drone) ────────────┐
         │              │ search / intercept path, avoids trees        │
         │              └──────────────┬──────────────────────────────┘
         │                             ▼ /<drone>/planning/path
         │              ┌── swarm_backend (exists) ───────────────────┐
         └─────────────▶│ PX4Drone.set_position + safety layer         │
                        └──────────────┬──────────────────────────────┘
                                       ▼
                                  PX4 → Gazebo
```

## Where things run, and why

- **Drones process their own lidar.** A lidar produces several MB/s of points; a drone radio link cannot carry that for a whole swarm, and a round trip to the base adds delay. Drones send compact **detections** (~100 bytes each, ~10 Hz) instead.
- **The base station fuses and decides.** It has the full picture: detections from every drone, the long-range radar and every drone's state. Fusing sensors gives better positions (several viewpoints) and fewer false alarms (a "drone" only one sensor sees is probably a bird or a tree).
- **The drones plan their own paths.** Planning needs the drone's current state and must keep working if the link drops.
- **The intercepting drone guides itself on the final approach.** The base's track gets it close; its own lidar track guides the last part, so a lost link late in the chase does not matter.
- **A safety layer in `swarm_backend` checks all AI output** (minimum distance between drones, geofence, altitude limits) on every tick, so a bad model cannot crash the swarm.

## Shared interfaces (`hive_interfaces` package)

Agreed by all three pairs in week 1. Changes need everyone's agreement. All positions use the swarm world frame from `swarm_backend`: **NED** (x north, y east, z down) with the origin at the Gazebo world origin.

```text
Detection.msg          # 2 → 1, one per detected object
  std_msgs/Header header         # stamp = time of the lidar scan
  string   sensor_id             # "px4_1/lidar", "base/radar"
  uint8    source                # LIDAR=0, RADAR=1
  uint8    object_class          # UNKNOWN=0, DRONE=1, BIRD=2, CLUTTER=3
  float32  confidence            # 0..1
  geometry_msgs/Point   position
  geometry_msgs/Vector3 velocity # NaN if unknown
  float32[9] position_covariance

Track.msg              # 1 → 3 (and to the scoring tools)
  std_msgs/Header header
  uint32   track_id
  float32  confidence
  geometry_msgs/Point   position
  geometry_msgs/Vector3 velocity
  float32[36] covariance         # position + velocity
  builtin_interfaces/Time first_seen
  builtin_interfaces/Time last_updated

Assignment.msg         # 1 → 3, one per drone
  uint32   drone_id
  uint8    role                  # PATROL=0, INTERCEPT=1, SHADOW=2, RETURN=3, HOLD=4
  uint32   track_id              # for INTERCEPT / SHADOW
  geometry_msgs/Point   goal     # patrol point or formation slot
  float32  priority

Path.msg               # 3 → swarm_backend
  std_msgs/Header header
  uint32   drone_id
  geometry_msgs/Point[] waypoints
  float32[] speeds               # m/s per waypoint
  builtin_interfaces/Time valid_until
```

## Developing in parallel

No pair waits for another:

- **Stand-in publishers:** each pair gets a small fake node for the topics it depends on, for example fake tracks made from the ground truth plus noise, or fake assignments ("drone 2: INTERCEPT track 1").
- **Recorded runs:** scenario runs are recorded with `ros2 bag` and shared, so the three pairs can work offline without starting Gazebo.
- **One scoring setup:** a fixed set of seeded scenarios and the same measures for everyone (see Evaluation below).
- **The main branch always runs:** every merge keeps the full chain working, even with stand-ins in place.
- **Full-chain run every two weeks:** all three parts together in Gazebo.

## Train offline, deploy small

- Train on GPU machines (Stellar); export learned models to **ONNX**; run them in the ROS nodes with `onnxruntime` on CPU.
- Keep models small (a few MB, int8 quantization if needed) so one inference takes **1–10 ms** on a drone-class computer (Jetson Orin Nano / Raspberry Pi 5).
- Check this by running the AI nodes in a container limited to drone-like resources, e.g. `docker run --cpus=2 --memory=4g`, and log per-stage timing.

**Latency budget, from lidar scan to the drone moving:**

| Stage | Budget |
|---|---|
| 2. Lidar detection | ≤ 50 ms |
| Radio link to base | ≤ 20 ms |
| 1. Fusion + decision | ≤ 10 ms |
| 3. First path | ≤ 50 ms |
| **Total** | **≈ 100–150 ms** |

A hostile drone at 30–50 m/s moves about 5 m in that time, which is fine when detection happens hundreds of metres out.

---

# 1. Base station

**Owner:** pair 1 · **Runs on:** base station · **Inputs:** detections from all drones, simulated radar, drone states · **Outputs:** tracks, assignments

The base station has two jobs: **fusion** (turn many noisy detections into one track per hostile drone) and **decision** (give every drone a role, choose interceptors, re-form the rest).

## 1a. Simulated radar

Gazebo has no radar, so this pair builds a simulated one:

- Long range (e.g. 2 km), coarse position (tens of metres error), a few Hz update rate.
- Uses the hostile drone's ground truth; adds noise, misses (detection probability drops with range) and clutter (false returns).
- Publishes `Detection` messages with `source = RADAR`.

## 1b. Fusion: detections → tracks

| | Approach |
|---|---|
| **Baseline** | One Kalman filter per track (constant velocity); nearest-neighbour matching of detections to tracks; start a track after N hits, drop it after M misses |
| **AI v1** | Learned track confidence: a small classifier over track features (speed, acceleration, how many sensors see it, lidar class votes) that separates real drones from birds and clutter |
| **AI v2** (optional) | Learned data association (which detection belongs to which track), or a learned motion model that predicts the hostile drone's next seconds better than constant velocity |

Must handle: the same drone seen by several drones and the radar (one track, not three); false alarms seen by only one sensor; detections arriving late or out of order over the radio link.

## 1c. Decision: roles, interceptors, formation

When a track appears, decide **how many** drones to send, **which** ones, and **how the rest re-form** to keep covering the area.

| | Approach |
|---|---|
| **Baseline** | Nearest drone intercepts; the rest keep the current formation, recomputed for N−1 drones with `Formation` + `assign_targets()` from `swarm_backend` |
| **AI v1** | Utility-based or auction-based assignment (e.g. CBBA): score each drone on time to intercept, battery, and the coverage gap it would leave; choose the formation that closes the gap |
| **AI v2** | Learned policy (multi-agent RL or a graph neural network over drones and tracks) trained in the fast Python environment against varied attacks |

**Staying instant:**
- Assignment for 3–20 drones is tiny: optimal (Hungarian) or greedy auction runs in well under 1 ms.
- During patrol, keep an up-to-date table of *"who intercepts if a threat appears in each sector"*. When a detection arrives, the decision is a lookup.
- A learned policy only runs a forward pass at runtime (< 5 ms).

## Deliverables by phase

| Phase | Weeks | Deliverable |
|---|---|---|
| 0 | 1–2 | `hive_interfaces` drafted; radar simulator; fake-track stand-in for pair 3 |
| 1 | 3–6 | Kalman fusion + nearest-drone assignment + N−1 re-forming, running in Gazebo |
| 2 | 7–12 | Learned track confidence; utility/auction assignment; coverage-aware formation choice |
| 3 | 13–16 | Handles lost drones and lost links; latency measured under drone-like limits |
| 4 | 17+ | Evaluation against the baseline |

## Metrics

Track accuracy (position error vs ground truth), false-track rate, time from first detection to confirmed track, interceptor choice vs best possible (computed afterwards from ground truth), coverage left during an intercept.

---

# 2. Lidar detection of the hostile drone

**Owner:** pair 2 · **Runs on:** each drone · **Inputs:** the drone's lidar point cloud and own position · **Outputs:** `/<drone>/perception/detections`

## Sensor setup

- Gazebo's lidar sensor on the drone; the PX4 image already includes `gz_x500_lidar_front`. A 3D lidar model (more beams) may be needed. Check what the image provides, or add a custom model.
- Lidar needs GPU rendering and the sim is already slow: start with lidar on **one or two drones**.
- Small drones return very few points beyond roughly 50–100 m. Lidar is the **short-range, precise** sensor; the radar handles long range.
- Convert detections from the drone's body frame to the world frame using the drone's position and attitude (`PX4Drone.position`, `.attitude`).

## Pipeline

```text
point cloud → remove ground & known static obstacles → cluster (e.g. DBSCAN)
           → per-cluster features / points → classifier → drone? → Detection
```

| | Approach |
|---|---|
| **Baseline** | Clustering + hand-written rules: cluster size, extent, height above ground, not matching the static tree map |
| **AI v1** | Classifier on hand-picked cluster features (size, shape, point spread, intensity, motion between scans): gradient boosting or a small MLP |
| **AI v2** | Point-cloud network on the raw cluster points (a small PointNet), plus using several scans for motion |

Classes: `DRONE`, `BIRD`, `CLUTTER` (trees, noise). **Birds are the hard case.** Pair 1 or the scenario runner should spawn bird-like distractors so the classifier learns to reject them.

## Training data

- **Labelled automatically from the simulation:** the ground-truth position of the hostile drone (and the distractors) labels every cluster. No manual labelling.
- Vary distance, angle, speed, hostile drone model/size, background (trees, sky, ground), and sensor noise, so the model does not overfit to one scene.
- Record point clouds with `ros2 bag` and train offline; keep a separate test set of scenarios never used for training.

## Staying fast

- Clustering + a small classifier takes a few ms per scan. A small PointNet exported to ONNX takes about 5–10 ms on a drone-class CPU/GPU.
- Only classify clusters that survive the cheap filters (removing ground and known trees), not the whole point cloud.
- Target: ≤ 50 ms from scan to published detection, at 10 Hz.

## Deliverables by phase

| Phase | Weeks | Deliverable |
|---|---|---|
| 0 | 1–2 | Lidar drone spawning in Gazebo; point clouds bridged to ROS 2; recording tools |
| 1 | 3–6 | Rule-based baseline publishing `Detection`s; automatic labelling pipeline |
| 2 | 7–12 | Feature classifier, then a point-cloud network; distractors in the dataset |
| 3 | 13–16 | ONNX export, timing under drone-like limits, robustness to noise |
| 4 | 17+ | Evaluation against the baseline |

## Metrics

Precision and recall per range band (0–25 m, 25–50 m, 50–100 m), maximum reliable detection range, false detections per minute, position error, inference time per scan.

---

# 3. Path to the hostile drone

**Owner:** pair 3 · **Runs on:** each drone · **Inputs:** the drone's assignment, tracks, own state, static obstacle map · **Outputs:** `/<drone>/planning/path`, executed by `swarm_backend`

This component decides **how each drone flies**: where to search while patrolling, and the collision-free path to intercept a hostile drone that is moving.

## 3a. Search: where to look

| | Approach |
|---|---|
| **Baseline** | Fixed patterns per sector: back-and-forth sweeps, or a ring around the base |
| **AI** | Belief-based search: keep a probability map of where a hostile drone could be (raised where no one has looked recently and along likely approach routes), and plan paths that maximize the probability of detection per second |

## 3b. Intercept: getting to a moving target

| | Approach |
|---|---|
| **Baseline** | Fly straight at the track's current position; **proportional navigation** as the steering rule |
| **AI v1** | A* on a 3D grid, or RRT*, through the precomputed obstacle map, aimed at the **predicted intercept point** (where the drone and the hostile drone can meet), replanned as the track updates |
| **AI v2** | Time-aware planning (kinodynamic RRT* or search over time), or a learned policy / learned cost-to-go that speeds up the planner |

## 3c. Paths for the hostile drone

The same planner generates the hostile drone's routes for the scenario runner: random start far from the base, goal at the base, plus a cost for being inside our sensor coverage. This gives varied, realistic attack paths (not the same route every time) that try to avoid detection, which is better training data for pairs 1 and 2.

## Staying instant

- **Time-limited planning:** give RRT* a fixed budget (e.g. 50 ms) and use the best path found so far.
- **Plan a short distance ahead:** plan a few seconds ahead and replan at 5–10 Hz, rather than solving the whole route once.
- **Instant fallback:** proportional navigation costs microseconds, so the drone steers by it while a new plan is being computed and is never waiting.
- **Precompute the world:** the occupancy map of trees and buildings is built once, offline, from the world file. Nothing processes the world at runtime.

## Deliverables by phase

| Phase | Weeks | Deliverable |
|---|---|---|
| 0 | 1–2 | Occupancy map from the Gazebo world; path follower in `swarm_backend`; fake-assignment stand-in |
| 1 | 3–6 | Sweep patrol + proportional-navigation intercept in Gazebo; hostile drone path generator (random start, straight to base) |
| 2 | 7–12 | A*/RRT* intercept to the predicted point; belief-based search; detection-avoiding hostile routes |
| 3 | 13–16 | Time-limited replanning, timing under drone-like limits, handling lost tracks |
| 4 | 17+ | Evaluation against the baseline |

## Metrics

Time to intercept, success rate, minimum distance to obstacles and to other drones, planning time per cycle, time to first detection during search.

---

# Evaluation

All three pairs use the same fixed set of **seeded scenarios** (easy → hard: slow and straight → fast, low, evasive), so AI and baseline are compared on exactly the same attacks.

| Measure | Mainly tests |
|---|---|
| Detection range (distance of hostile drone when first confirmed) | 2, 1 |
| Time from first detection to intercept | 1, 3 |
| Intercept success rate / hostile drone reached base | All |
| False alarms per hour | 2, 1 |
| Coverage lost during intercepts | 1, 3 |
| End-to-end latency, scan → movement | All |

Each pair reports **AI vs baseline** on this set; together that is the evaluation for the final report.

# Timeline overview

| Phase | Weeks | Whole group |
|---|---|---|
| 0. Contracts | 1–2 | `hive_interfaces`, stand-in publishers, scenario runner (spawn, reset, score), faster sim |
| 1. Baselines | 3–6 | Full chain in Gazebo with all three baselines; full-chain run every two weeks from here on |
| 2. AI v1 | 7–12 | Fast Python training environment; each pair trains and swaps in its AI version |
| 3. Integration | 13–16 | Latency under drone-like limits; robustness (noise, lost link, lost drone) |
| 4. Evaluation | 17+ | AI vs baseline on the fixed scenarios; report |
