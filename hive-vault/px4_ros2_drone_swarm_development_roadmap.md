# PX4 + Gazebo + ROS 2 Drone Swarm Development Roadmap

## Development Order

Build the system bottom-up: first make one drone reliable, then abstract its interface, then introduce the swarm layer, and finally implement formations and missions.

| # | File | What to build | What to research |
|---|---|---|---|
| **1** | `config.yaml` / `config.py` | Drone IDs, namespaces, rates, limits, initial positions | `ROS 2 Python parameters YAML configuration` |
| **2** | `px4/topics.py` | Centralized PX4/ROS 2 topic-name construction | `PX4 ROS 2 multi vehicle namespaces` |
| **3** | `px4/messages.py` *(optional)* | Helpers for constructing PX4 messages | `PX4 ROS 2 Python VehicleCommand TrajectorySetpoint` |
| **4** | `px4/drone.py` | Generic drone state and high-level operations | `Python ROS 2 class create publisher subscriber` |
| **5** | `px4/px4_drone.py` | PX4-specific publishers/subscribers | `PX4 ROS 2 Python offboard control example` |
| **6** | `tests/single_drone_test.py` | Arm → takeoff → move → land one drone | `PX4 ROS 2 offboard Python example` |
| **7** | `swarm/swarm.py` | Collection of `Drone`/`PX4Drone` objects | `Python collection of objects design` |
| **8** | `swarm/manager.py` | Commands affecting all/selected drones | `ROS 2 multi robot architecture` |
| **9** | `swarm/formation.py` | Formation geometry and target positions | `multi UAV formation control` |
| **10** | `mission/mission.py` | High-level swarm state machine | `ROS 2 state machine Python` |
| **11** | `nodes/swarm_node.py` | Main ROS 2 entry point and dependency wiring | `ROS 2 Python Node architecture` |
| **12** | `launch/*.py` | Start multiple vehicles/nodes | `PX4 ROS2 multi vehicle simulation launch Gazebo` |

---

## 1. Configuration

### File

```text
config/swarm.yaml
```

Start by making the swarm configurable rather than hard-coded.

Example:

```yaml
drones:
  - id: 0
    namespace: px4_0
    position: [0, 0, 0]

  - id: 1
    namespace: px4_1
    position: [5, 0, 0]

  - id: 2
    namespace: px4_2
    position: [0, 5, 0]
```

### Research

Search for:

- `ROS 2 Python parameters YAML configuration`
- `ROS 2 parameters Python`
- `ROS 2 YAML parameters`

### Goal

The number of drones, their IDs, namespaces, and configuration should not be hard-coded throughout the application.

---

## 2. PX4 Topic Abstraction

### File

```text
swarm_backend/px4/topics.py
```

Make one place responsible for constructing PX4 topic names.

Example:

```python
class PX4Topics:
    def __init__(self, namespace):
        self.namespace = namespace

    @property
    def vehicle_command(self):
        return f"{self.namespace}/fmu/in/vehicle_command"

    @property
    def trajectory_setpoint(self):
        return f"{self.namespace}/fmu/in/trajectory_setpoint"

    @property
    def vehicle_local_position(self):
        return f"{self.namespace}/fmu/out/vehicle_local_position"
```

### Research

Search for:

- `PX4 ROS 2 multi vehicle namespaces`
- `PX4 ROS 2 DDS namespace multi vehicle`
- `PX4 ROS 2 topics`
- `PX4 ROS 2 multi vehicle`

### Goal

Going from:

```text
px4_0
```

to:

```text
px4_1
px4_2
px4_3
```

should require changing configuration, not copying topic strings throughout the code.

---

## 3. PX4 Message Helpers

### File

```text
swarm_backend/px4/messages.py
```

This is optional initially.

Instead of repeatedly constructing messages:

```python
msg = VehicleCommand()
msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
msg.param1 = 1.0
```

you could eventually have helpers:

```python
msg = arm_command()
```

### Research

Search for:

- `PX4 VehicleCommand ROS 2`
- `PX4 TrajectorySetpoint ROS 2`
- `PX4 OffboardControlMode ROS 2`
- `PX4 ROS 2 Python messages`

### Goal

Keep PX4 message-construction details out of the higher-level swarm code.

Don't over-engineer this file at the beginning.

---

## 4. Drone Abstraction

### File

```text
swarm_backend/px4/drone.py
```

Define what a drone should expose to the rest of the program.

For example:

```python
class Drone:

    def __init__(self, drone_id):
        self.id = drone_id

        self.position = None
        self.velocity = None
        self.attitude = None

    def arm(self):
        ...

    def takeoff(self, altitude):
        ...

    def land(self):
        ...

    def set_position(self, x, y, z):
        ...

    def set_velocity(self, vx, vy, vz):
        ...
```

### Research

Search for:

- `Python composition vs inheritance`
- `Python class encapsulation`
- `ROS 2 Python Node class publisher subscriber`
- `object oriented robotics software architecture`

### Goal

The rest of the application should be able to interact with a drone without knowing about:

- ROS 2 topic names
- PX4 messages
- QoS settings
- publisher implementation
- subscriber implementation

---

## 5. PX4 Drone Implementation

### File

```text
swarm_backend/px4/px4_drone.py
```

Implement the PX4-specific version of the drone interface.

Conceptually:

```text
Drone
  ↑
  │
PX4Drone
  │
  ├── publishers
  ├── subscribers
  ├── PX4 messages
  └── PX4-specific logic
```

For example:

```python
class PX4Drone(Drone):

    def __init__(self, node, config):
        ...
        self.command_pub = node.create_publisher(...)
        self.position_sub = node.create_subscription(...)
```

### Research

Search for:

- `ROS 2 Python publisher subscriber class design`
- `PX4 ROS 2 Python offboard control`
- `PX4 ROS 2 offboard example`
- `PX4 OffboardControlMode TrajectorySetpoint`

### Goal

At the end of this stage, you should be able to control **one simulated PX4 drone** through a clean Python interface.

---

## 6. Single Drone Test

### File

```text
tests/single_drone_test.py
```

Before implementing the swarm, make one drone reliable.

The basic sequence should be:

```text
connect
   ↓
wait for PX4
   ↓
arm
   ↓
enter offboard
   ↓
takeoff
   ↓
move
   ↓
hover
   ↓
land
   ↓
disarm
```

Ideally your test code eventually looks something like:

```python
drone.arm()
drone.takeoff(-5.0)
drone.set_position(5.0, 0.0, -5.0)
drone.land()
```

The exact coordinate convention depends on your PX4 setup.

### Research

Search for:

- `PX4 ROS 2 Offboard Control`
- `PX4 ROS 2 OffboardControlMode`
- `PX4 ROS 2 TrajectorySetpoint`
- `PX4 ROS 2 Python offboard control`

### Goal

**Do not move on to swarm behavior until one drone works reliably.**

---

## 7. Swarm Object

### File

```text
swarm_backend/swarm/swarm.py
```

Once one drone works, make the swarm a collection of drones.

Example:

```python
class Swarm:

    def __init__(self, drones):
        self.drones = drones

    def arm_all(self):
        for drone in self.drones:
            drone.arm()

    def land_all(self):
        for drone in self.drones:
            drone.land()
```

Now you can do:

```python
swarm.arm_all()
```

instead of:

```python
drone0.arm()
drone1.arm()
drone2.arm()
```

### Research

Search for:

- `Python collection of objects design`
- `Python object aggregation`
- `multi robot software architecture`

### Goal

Going from one drone to multiple drones should require creating more `PX4Drone` objects, not duplicating code.

---

## 8. Swarm Manager

### File

```text
swarm_backend/swarm/manager.py
```

Introduce coordination at the swarm level.

Possible interface:

```python
class SwarmManager:

    def takeoff(self, altitude):
        ...

    def move_all(self, targets):
        ...

    def emergency_land(self):
        ...
```

The distinction should be:

### `Swarm`

What drones exist?

### `SwarmManager`

What should the swarm do?

### Research

Search for:

- `ROS 2 multi robot architecture`
- `multi robot software architecture`
- `robot swarm software architecture`
- `ROS 2 multi robot systems`

### Goal

Keep individual drone communication separate from swarm-level coordination.

---

## 9. Formation Controller

### File

```text
swarm_backend/swarm/formation.py
```

Now implement formation geometry.

For example:

```python
class Formation:

    def grid(self, center, spacing):
        ...

    def line(self, center, spacing):
        ...

    def circle(self, center, radius):
        ...
```

A formation function should return target positions rather than ROS/PX4 messages.

For example:

```python
targets = formation.grid(
    center=[0, 0, -10],
    spacing=5
)
```

Could return:

```python
[
    [0, 0, -10],
    [5, 0, -10],
    [0, 5, -10],
    [5, 5, -10]
]
```

### Important

`formation.py` should **not know that PX4 or ROS 2 exists**.

It should operate on:

- positions
- velocities
- target positions
- formation parameters

### Research

Start with:

- `UAV formation control`
- `multi UAV formation geometry`
- `multi agent formation control`

Then, if you want more advanced swarm behavior:

- `consensus based formation control UAV`
- `potential field formation control UAV`
- `leader follower formation control UAV`
- `distributed formation control UAV`

### Goal

Separate the mathematical swarm behavior from the communication layer.

---

## 10. Mission Controller

### File

```text
swarm_backend/mission/mission.py
```

Represent an autonomous mission as a state machine.

For example:

```text
TAKEOFF
   ↓
FORM_GRID
   ↓
FLY_TO_WAYPOINT
   ↓
FORM_LINE
   ↓
FLY_TO_WAYPOINT
   ↓
LAND
```

Possible states:

```python
TAKEOFF
FORMATION
NAVIGATE
HOVER
LAND
EMERGENCY
```

### Research

Search for:

- `Python state machine pattern robotics`
- `ROS 2 state machine Python`
- `robotics finite state machine`
- `finite state machine autonomous robot`

### Goal

Avoid putting a large mission sequence into one giant `if/elif` block.

---

## 11. Main ROS 2 Node

### File

```text
swarm_backend/nodes/swarm_node.py
```

This should eventually be relatively small.

Its main purpose should be to **wire everything together**.

Conceptually:

```python
class SwarmNode(Node):

    def __init__(self):
        super().__init__("swarm")

        configs = load_config()

        drones = [
            PX4Drone(self, config)
            for config in configs
        ]

        self.swarm = Swarm(drones)
        self.manager = SwarmManager(self.swarm)
        self.mission = Mission(self.manager)
```

### Research

Search for:

- `ROS 2 Python Node architecture`
- `ROS 2 Python package structure`
- `ROS 2 dependency injection`
- `ROS 2 Python launch node`

### Goal

The node should primarily connect components together.

The actual swarm algorithms should live in their respective modules.

---

## 12. ROS 2 Launch

### Files

```text
launch/
├── swarm.launch.py
├── drone.launch.py
└── simulation.launch.py
```

Eventually you want to be able to start the swarm cleanly.

For example:

```bash
ros2 launch swarm_backend simulation.launch.py num_drones:=5
```

### Research

Search for:

- `ROS 2 Python launch multiple nodes`
- `ROS 2 launch arguments Python`
- `ROS 2 launch multiple robots`
- `PX4 ROS 2 multi vehicle simulation`
- `PX4 Gazebo multi vehicle simulation`
- `PX4 ROS 2 multi vehicle launch`

### Goal

Make starting the complete simulation reproducible.

---

# Recommended Development Milestones

## Milestone 1 — One Drone

Build:

```text
config
   ↓
PX4 topics
   ↓
PX4Drone
   ↓
single_drone_test
```

Get:

```text
ARM
 ↓
OFFBOARD
 ↓
TAKEOFF
 ↓
MOVEMENT
 ↓
HOVER
 ↓
LAND
```

working reliably.

---

## Milestone 2 — Multiple Independent Drones

Build:

```text
config
    ↓
PX4Drone × N
    ↓
Swarm
    ↓
SwarmManager
```

Get something like this working:

```text
drone 0 ──┐
drone 1 ──┤
drone 2 ──┼── ARM / TAKEOFF / LAND
drone 3 ──┘
```

At this stage the drones don't need sophisticated swarm behavior yet.

---

## Milestone 3 — Formation Behavior

Build:

```text
Formation
     ↓
SwarmManager
     ↓
Drone 0
Drone 1
Drone 2
...
```

Start with:

1. Static grid
2. Static line
3. Move as a formation
4. Change formation
5. Waypoint navigation

---

## Milestone 4 — Autonomous Swarm

Eventually:

```text
                    Mission
                       ↓
                Formation Controller
                       ↓
                 Swarm Controller
                       ↓
              Individual Drone Control
                       ↓
                     PX4
                       ↓
                    Gazebo
```

This is where you can start investigating:

- Collision avoidance
- Leader/follower control
- Consensus algorithms
- Distributed control
- Trajectory generation
- Dynamic formations
- Inter-drone communication

---

# Final Architecture

The main architectural goal is to keep the dependency direction clean:

```text
                    ┌────────────────┐
                    │    mission     │
                    └───────┬────────┘
                            ↓
                    ┌────────────────┐
                    │   formation    │
                    └───────┬────────┘
                            ↓
                    ┌────────────────┐
                    │ swarm_manager  │
                    └───────┬────────┘
                            ↓
                       ┌─────────┐
                       │  swarm  │
                       └────┬────┘
                            ↓
                 ┌──────────┴──────────┐
                 ↓          ↓          ↓
              PX4Drone   PX4Drone   PX4Drone
                 ↓          ↓          ↓
               PX4 #0     PX4 #1     PX4 #2
```

The most important rule is:

> **`formation.py` and mission logic should not know that PX4 or ROS 2 exists.**

ROS 2/PX4 should be the communication and execution layer underneath the swarm logic.

---

# Suggested Initial File Tree

Don't create every file immediately. Start small:

```text
swarm_backend/
│
├── package.xml
├── setup.py
├── setup.cfg
│
├── resource/
│   └── swarm_backend
│
├── config/
│   └── swarm.yaml
│
└── swarm_backend/
    │
    ├── __init__.py
    │
    ├── px4/
    │   ├── __init__.py
    │   ├── drone.py
    │   └── topics.py
    │
    ├── swarm/
    │   ├── __init__.py
    │   └── swarm.py
    │
    └── nodes/
        ├── __init__.py
        └── swarm_node.py
```

Add:

```text
px4/messages.py
px4/px4_drone.py
swarm/manager.py
swarm/formation.py
mission/mission.py
launch/
tests/
```

when each becomes necessary.

---

# Recommended Research Order

If you want to learn while implementing, research in this order:

1. `ROS 2 Python package structure`
2. `ROS 2 Python Node publisher subscriber`
3. `PX4 ROS 2 architecture`
4. `PX4 ROS 2 Offboard Control`
5. `PX4 ROS 2 VehicleCommand`
6. `PX4 ROS 2 TrajectorySetpoint`
7. `PX4 ROS 2 multi vehicle`
8. `ROS 2 namespaces multi robot`
9. `ROS 2 QoS`
10. `Python object oriented design composition inheritance`
11. `multi robot software architecture`
12. `UAV formation control`
13. `multi agent formation control`
14. `robotics finite state machine`
15. `ROS 2 Python launch multiple robots`
16. `PX4 Gazebo multi vehicle simulation`

**Don't start with formation-control papers/code.** First understand exactly how one PX4 vehicle communicates through ROS 2 and how PX4 identifies/names multiple vehicles. That foundation will make the swarm architecture much easier to get right.
