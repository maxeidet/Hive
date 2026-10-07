# Final state

A possible end state for the project: what we could show at the end of the course and how it answers the planning report's two questions. This is a target to steer by, not a promise. We start small (section "Stages") and expand this note as decisions are made in [[Lock-on brainstorm]].

Back to [[Home]].

---

## The final demo, in one paragraph

In Gazebo, a ring of friendly drones patrols about 1 km around the `hive_base` camp. A hostile drone comes in from 2 km at 17 m/s. The base station's radar picks it up and assigns the friendly drone that can meet it earliest. That drone flies toward the predicted meeting point while the rest of the ring closes the gap. Its narrow gimbal camera, pointed by the radar, finds a small dark blob against the sky. The drone's fast model flags it, the base station's stronger model calls it a drone, and the interceptor **locks on**: camera bearing plus radar range, with the gimbal keeping the target centred. It then guides itself to the intercept. Birds fly in the same scenarios: they get investigated and released, but no hostile drone is let through. Every run is a seeded scenario, scored automatically.

## Answers to the research questions

| Question (planning report) | What the final state shows |
|---|---|
| **1. How can a hostile drone near a protected asset be detected?** | Layered sensing. Base radar at long range (2 km), a wide camera on each drone for short range (fills the radar's gaps), and a narrow gimbal camera pointed by either of them (*slew-to-cue*). A two-tier classifier decides: a fast model on the drone that rarely misses, then a strong model at the base that uses sequences of crops, several drones and the radar. Measured as detection range and **missed drones**, with false pursuits as the price. |
| **2. How can it be intercepted, with a planning-based or an RL-based controller?** | A **planning-based** controller as the baseline: fly to the predicted meeting point on radar, then proportional navigation (PN) once locked (`intercept/guidance.py` grows into this). An **RL controller** trained in a fast Python simulator and run through the same `Drone` interface, compared with the baseline on the same scenarios. Measured as intercept success, time to intercept and miss distance. |

## The system at the end

```text
base station:  radar → tracks → classifier (strong, tracklets) → assignment (who intercepts, who re-forms)
                  ▲ crops + bearings              │ roles: PATROL / INVESTIGATE / INTERCEPT
                  │                               ▼
each drone:    wide cam + narrow gimbal cam → fast detector → lock-on (Cue → Detect → Acquire → Locked)
                                                             → controller (planning or RL) → PX4 → Gazebo
```

## Stages: start small, expand

Each stage is a working demo on its own; later stages only add to it.

| Stage | What works | Builds on |
|---|---|---|
| **0. Now** | One interceptor chases one hostile drone at 50 m using a mock radar, and shadows it at a standoff | exists ([[1octstella]]) |
| **1. Lock-on v1** | Narrow gimbal camera model; sky-blob detector on the drone; camera bearing + radar range in the tracker; LOCKED state. One hostile drone, sky background | Lock-on brainstorm: Cue, Acquire, Locked |
| **2. Real intercept** | Hostile drone at 17 m/s, interceptor at 12 m/s; head-on meeting point + PN once locked; a ring of 5+ drones and the base assigns the interceptor | [[1octstella]] section 5 |
| **3. Classification** | Crops sent to the base, tracklet classifier, birds as distractors, `INVESTIGATE` role, no-miss tuning | Lock-on brainstorm: Detect and classify |
| **4. Wide coverage** | Wide camera per drone + slew-to-cue scheduler; CNN detector for tree and ground backgrounds | Lock-on brainstorm: Two cameras |
| **5. RL controller** (stretch) | RL interception policy trained in a fast simulator, compared with the planning baseline; several hostile drones and decoys | [[ai_components]] |

## How we judge it

- **Missed hostile drones** per scenario (the main measure; close to 0).
- Intercept success rate, time from first detection to lock and to intercept.
- False pursuits per hour, and coverage lost while drones are away.
- Planning vs RL controller on the same seeded scenarios.

## Not in scope

- Real drones and real sensors: everything is in simulation.
- What an intercept ends with (escort or neutralization) is still open; the final state stops at "locked and within intercept distance".

## To expand next

- [ ] Timeline: which stage by which week (match [[ai_components]] phases).
- [ ] Who owns which stage (the three pairs).
- [ ] What the RL controller observes and outputs, and which fast simulator.
- [ ] The scenario set: easy → hard (slow and straight → fast, low, evasive, with birds).

Related: [[Home]], [[Lock-on brainstorm]], [[1octstella]], [[ai_components]], [[Codebase map]]
