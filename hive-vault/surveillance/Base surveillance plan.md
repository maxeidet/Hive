# Base surveillance plan

How the defenders watch the sky in front of the base, what happens to the coverage when one of them leaves to attack, and how the base station and the drones talk to each other. Brainstorm from 15 Oct 2026, written up for the team. Nothing here is built yet.

Back to [[Home]]. Related: [[Baseline attack work plan]] (the attack a defender flies when it leaves), [[Join test results 2026-10-14]].

---

## In short

- **3 surveillance drones on a quarter circle** (90° arc) in front of the base, facing the direction the threat comes from. This replaces the full ring of N drones that the report and Oscar's evaluation harness still describe.
- **Baseline for covering the gap (option A):** when one drone leaves to attack, the two that remain stay where they are and widen their gimbal sweeps to cover the empty slice. No drone moves.
- **Central control by the base station:** the drones report to the base station, and the base station gives the orders. The message types are below.
- **One drone attacks, not all three:** the drone that detects the target goes, and the other two stay. The rules are in section 4.
- **Rules decided and proposed in section 4:** the detecting drone turns at once, the first report on a target wins, one drone is held in reserve, and an attack is called off at 1000 m from the base or when the target is lost. Still open: what "target lost" means and the "close to the base" distance.

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

## 4. How the base station decides

**Jostein owns this section.**

### Decided

1. **Who goes:** the drone that detects the target.
2. **The two that stay:** they hold their points and widen their sweeps from 30° to 45° each (option A). The surveillance always covers 90° in total, split over the drones that are left (90° / number on patrol).
3. **On a miss:** the drone is sent back if it still exists. A new drone is sent only if the target is still on its way in.
4. **The last drone:** it is held back until the target is confirmed and close to the base. At that point it is better to use it than to save it.
5. **Several targets:** each target gets one drone.
6. **Who starts the attack:** the drone that sees the target turns toward it at once (as the report says), and the base station confirms or calls it back.
7. **Call-off:** an attacker turns back when it is more than 1000 m from the base or when it loses the target.

### Proposal: the rules in detail

These close the gaps between the decisions above. Numbers are settings, not fixed values.

| Setting | Start value | Meaning |
|---|---|---|
| `sector_deg` | 30° / 45° | 90° divided by the drones on patrol |
| `leash_m` | 1000 | Max distance from the base for an attacker (decision 7) |
| `close_radius_m` | 1500 | "Close to the base": the last drone may go for a confirmed target inside this |
| `reserve` | 1 | Drones that are never sent unless the target is confirmed and inside `close_radius_m` |
| `resweep_delay_s` | 10 | Wait before sectors go back to 30° after an attacker returns |

| Situation | Rule |
|---|---|
| A drone detects a new target | It locks the target ID in the base station's target table and goes to ENGAGE at once. The base station confirms or calls it back |
| Two drones see the same target | First report wins. Ties go to the lowest drone id. The other stays on PATROL (it never leaves, the lock only stops it from starting) |
| Free drones on patrol are above `reserve` | The detecting drone may start an attack on its own |
| Only `reserve` drones are left on patrol | The drone may not start on its own. It starts only when the base station says the target is confirmed and inside `close_radius_m` |
| Several targets | One drone per target. When no drone may go, the target waits. Targets inside `close_radius_m` are served first, closest to the base first |
| Attacker passes `leash_m` or loses the target | RETURN. The target lock is released |
| Miss, attacker still exists | RETURN to its own slot. Sectors go back to 30° after `resweep_delay_s` |
| Miss, attacker lost, target still inbound | A new drone is sent from those on patrol, under the same `reserve` rule |
| Miss, attacker lost, target gone | Nothing is sent. Sectors stay widened |

Check before building: `leash_m` must be long enough to reach where an intercept can happen, and short enough that the arc is not left open for long. With defenders at 5 m/s and the hostile at 1 m/s, a return from 1000 m takes about 200 s.

### Still open

1. **Is the 1000 m measured from the base to the attacker or to the target?** The proposal uses the attacker.
2. **What counts as "target lost"** (no detection for how many seconds, or low confidence it is a drone).
3. **Value of `close_radius_m`**, which depends on the detection range we measure.

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
