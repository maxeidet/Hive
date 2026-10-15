# Base surveillance plan

How the defenders watch the sky in front of the base, what happens to the coverage when one of them leaves to attack, and how the base station and the drones talk to each other. Brainstorm from 15 Oct 2026, written up for the team. Nothing here is built yet.

Back to [[Home]]. Related: [[Baseline attack work plan]] (the attack a defender flies when it leaves), [[Join test results 2026-10-14]].

---

## In short

- **3 surveillance drones on a quarter circle** (90° arc) in front of the base, facing the direction the threat comes from. This replaces the full ring of N drones that the report and Oscar's evaluation harness still describe.
- **Baseline for covering the gap (option A):** when one drone leaves to attack, the two that remain stay where they are and widen their gimbal sweeps to cover the empty slice. No drone moves.
- **Central control by the base station:** the drones report to the base station, and the base station gives the orders. The message types are below.
- **Open:** exactly how the base station makes its decisions. **Jostien decides this.** The section on decisions only lists suggestions.

## 1. The setup: 3 drones on a quarter circle

- The arc only makes sense if we know roughly where the threat comes from. That direction becomes a setting, the *threat heading*, like `search_heading_deg` in `scenarios/attack.yaml`.
- The 3 drones hold fixed points on an arc of radius R around the base, at altitude h. Each owns a 30° slice of the 90° arc and sweeps its gimbal camera across that slice. The narrow camera needs a few steps per slice.
- Two ways to place them (to decide):

| Placement | Slots (from the threat heading) | Good | Bad |
|---|---|---|---|
| Centre of each slice | −30°, 0°, +30° | Each drone sits in the middle of what it watches | The arc's edges are 15° from the nearest drone |
| Ends and middle | −45°, 0°, +45° | The edges are covered from right next to them | The outer drones look half outside the arc |

- **The measure that matters is the revisit time:** how long a given direction goes unwatched before a camera looks at it again. With 3 drones the slices are small, so revisits are fast. With 2 drones each slice is 50 % bigger.

## 2. Covering the gap while one drone attacks

Our attack flies into the hostile drone, so **the attacking defender is probably lost**. The gap it leaves is permanent, not temporary.

| Option | What happens | Good | Bad |
|---|---|---|---|
| **A. Widen the sweep (baseline)** | The 2 remaining drones stay put and widen their sweeps to cover the empty slice between them | Instant, no flying, simplest | The middle of the gap is far from both drones, so the camera sees less far there. Longer revisit time |
| B. Redistribution (from the report) | The remaining drones move to new, evenly spaced points (e.g. ±22.5°) and take 45° each | Even coverage again | Worse coverage while they are moving |
| C. Replacement (from the report) | A reserve drone from the base takes the empty point | Full coverage again | Needs a 4th drone; we have max 3 |
| D. A, then B | Widen the sweeps at once, move once it is clear the attacker will not come back | Best of A and B | Two changes of state to get right |

**We start with A as the baseline.** B and D are candidates for the improvement we compare against it.

Also to handle: a missed attack. The attacker could fly back to its point, and the other two then go back to their normal slices.

### The evaluation test this gives

Two hostile drones, where the second arrives **while the first is being attacked**. Measured: is the second one detected, at what range, and how much of the arc goes unwatched (revisit time) while a defender is away. Run the same seeded scenarios for every option, so A, B and D compare directly. This is the report's "coverage lost while a defender is away from the ring".

## 3. Central control by the base station

The base station does not fly and has no camera. It gets everything the drones send and gives the orders. With 3 drones this is the simplest choice and matches the report's base station section. Control where each drone only listens to its neighbours (as in Kingston et al., in the report) can be a later improvement.

All drones are already in the same ROS 2 (DDS) network, so the protocol is a few ROS topics and a base station node.

### Messages

| Message | Direction | Content |
|---|---|---|
| `Status` (heartbeat, e.g. 2 Hz) | drone → base | id, position, state (PATROL / ENGAGE / RETURN / LOST), its current slice, battery |
| `Detection` | drone → base | id, direction to the target, confidence, time. Later also image crops (report, Base Station) |
| `Assignment` | base → drone | PATROL (point, slice to sweep), ENGAGE (target), RETURN. Numbered, so an old order can be told from a new one |
| `Ack` | drone → base | Which assignment number the drone now follows |

### Drone states

```
PATROL --(ENGAGE from base)--> ENGAGE --(hit)--> lost
   ^                              |
   |                         (miss / called back)
   +-------- RETURN <-------------+
```

### When the link fails (suggestion)

- Base station: no `Status` from a drone for a few seconds → treat its slice as empty and give the others wider slices (option A).
- Drone: nothing from the base station → keep following its last order. It does not stop or land.

Later, for realism: add delay and lost messages, like the fake detector's errors, and measure how much worse it gets.

## 4. How the base station decides — open, Jostien decides

**Jostien owns this section.** The points below are suggestions from the brainstorm, not decisions.

1. **Who starts the attack?**
   - *Suggestion 1:* the drone that sees the target turns toward it at once (as the report says) and the base station confirms or calls it back. Fast, but two drones can see the same target.
   - *Suggestion 2:* the drone only reports, and the base station sends the ENGAGE order. Slower, but the base station is always in control.
2. **Two drones see the same target.** The first report wins, or the drone closest to the target's path wins.
3. **How many may be away at once?** Triangulation needs two drones to see the target, but sending two away leaves one to watch the whole arc. Suggestion: at most one attacking, a second only when the target is confirmed and close.
4. **How the new slices are split in option A.** Each of the two remaining drones takes half of the empty slice, or the drone nearer the gap takes all of it.
5. **When to call an attack off** (target lost, low confidence it is a drone, too far from the arc) and what the drone does then.

*Jostien: write your decisions here and remove the suggestions you don't use.*

## 5. To agree on as a team

1. Is a defender always lost on a hit? That decides whether this is a temporary gap or a permanent loss.
2. Is the threat heading known in advance, and is the arc fixed or can it turn?
3. Radius and altitude in Gazebo. The `hive_base` ground is 1000 × 1000 m, and Gazebo slows down with several camera drones, so start small (e.g. R = 60–100 m, h = 20 m) and keep both as settings.
4. Update the report (Base Surveillance, Baseline System) and tell Oscar: the report and his `SectorScan` placeholder (`detect/scan.py` on `feat-baseline-eval`) still describe a full ring of N drones, each sweeping 180°.

## What already exists to build on

| Piece | Where |
|---|---|
| Evenly spaced points on a circle | `Formation.circle()` in `swarm/formation.py` (needs an arc version) |
| Fly to a world position with a heading | `drone.set_position(x, y, z, yaw)` |
| Hold the gimbal at a pitch/yaw | `drone.aim_gimbal(pitch, yaw)` |
| Take off, "reached" checks and landing for many drones | `SwarmManager` |
| The attack a defender flies | `nodes/attack_node.py`, [[Baseline attack work plan]] |
| Interface for the camera sweep | `GimbalScan` in `detect/scan.py` (on `feat-baseline-eval`) |
