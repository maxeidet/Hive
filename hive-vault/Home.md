# Hive

Counter-UAS drone swarm: a swarm of PX4 drones in Gazebo, controlled from a Python ROS 2 backend, that detects, intercepts and follows hostile drones. Repo: `github.com/maxeidet/Hive`.

Start here, then follow the links.

## Where things are

- [[Codebase map]]: what each folder and file in the repo is, how to run things, and what is built so far.
- [[Testing]]: kinds of tests, the rules every evaluation follows, the standard run output, and a checklist for adding a test.

## Plans and design

| Note | What it is |
|---|---|
| [[Final state]] | Possible end state of the project: the final demo, answers to the report's two questions, stages from small to full |
| [[ai_components]] | The three AI components (base station fusion and decisions, lidar detection, path planning), shared interfaces, timeline and evaluation |
| [[px4_ros2_drone_swarm_development_roadmap]] | Bottom-up build order for `swarm_backend` (one drone → swarm → formations → missions), with current status |
| [[1octstella]] | Intercept and lock-on design notes (1 Oct 2026): chase algorithm, radar model, sensor ranges, the real scenario |
| [[Lock-on brainstorm]] | Mindmap: how the drone finds, confirms and keeps a lock on the hostile drone (cue → acquire → confirm → locked → lost) |

## Environment and setup

| Note | What it is |
|---|---|
| [[StackV2]] | Tech stack and dev environment (Swedish): Docker, ROS 2, PX4, Gazebo, Mac/Windows setup, VNC, multi-drone, HPC |
| [[Stack]] | First version of the stack description (Swedish). Replaced by [[StackV2]] |
| [[WSL2 Docker setup]] | Manual PX4 + Gazebo + ROS 2 setup on Windows/WSL2 |
| [[GHCR How-To]] | Link: publishing images to GitHub Container Registry |

Setup docs that live in the repo, outside the vault:
- `docker/README.md`: the dev container, the `start-dev.sh` flags, worlds and drone sensor variants
- `docker/README-stellar.md`: running on the Stellar lab computers (rules, `start-stellar.sh`, cleanup)
- `ros2_ws/src/swarm_backend/README.md`: the backend, `mount-backend.sh` commands, intercept

## Research

| Note | What it is |
|---|---|
| [[UAS - Initial Landscape]] | Early questions about UAS and the main technical problems (stub) |
| [[CarlaAir]] | Link: CarlaAir simulator |
| `Research/UAV swarms research.pdf` | Background reading |
