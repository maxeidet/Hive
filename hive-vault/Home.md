# Hive

Counter-UAS drone swarm: a swarm of PX4 drones in Gazebo, controlled from a Python ROS 2 backend, that detects, intercepts and follows hostile drones. Repo: `github.com/maxeidet/Hive`.

Start here, then follow the links.

## Where things are

- [[Codebase map]]: what each folder and file in the repo is, how to run things, and what is built so far.
- [[Testing]]: kinds of tests, the rules every evaluation follows, the standard run output, and a checklist for adding a test.

## Plans and design

| Note                                           | What it is                                                                                                                                          |
| ---------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| [[Final state]]                                | Possible end state of the project: the final demo, answers to the report's two questions, stages from small to full                                 |
| [[ai_components]]                              | The three AI components (base station fusion and decisions, lidar detection, path planning), shared interfaces, timeline and evaluation             |
| [[px4_ros2_drone_swarm_development_roadmap]]   | Bottom-up build order for `swarm_backend` (one drone → swarm → formations → missions), with current status                                          |
| [[1octstella]]                                 | Intercept and lock-on design notes (1 Oct 2026): chase algorithm, radar model, sensor ranges, the real scenario                                     |
| [[Detection implementation plan]]              | How to build the report's detection section: camera math, dark-object baseline, YOLO26 + ByteTrack, Gazebo training data, evaluation, order of work |
| [[Progress 2026-10-13]]                        | Detection, day 1: what was built (image capture with truth, scenario 1, baseline detector, predict), first results and what we learned              |
| [[Manual - scenario 1, images and prediction]] | Step by step: fly scenario 1, take camera images with the truth saved, run the detector on them                                                     |
| [[Tutorial - live detector (detect)]]          | How to run `./mount-backend.sh detect`: options, the debug picture, what it publishes (target_offset), known limits. Updated as it grows            |
| [[Manual tests to do]]                         | Checklist of tests that need a person and the running simulation, kept up to date as things are built                                               |
| [[Following the target with detection]]        | Design: how the detection steers the gimbal and the drone so it follows the target on the camera, with the radar as cue and fallback                |
| [[Baseline attack work plan]]                  | Oskar/Max agreement (13 Oct): the two shared ROS topics (target_position, target_offset), who builds what, the attack states, open points           |
| [[Baseline attack - how to run]] | The baseline attack (pure pursuit on the detection topics), the fake detector and the attack flight test: how to run them, the contract, the files, first results |
| [[Join test results 2026-10-14]] | First runs of Max's attack on the real detector: head-on hits, crossing misses by ~4 m, what went wrong and what's next |
| [[Baseline evaluation - how to run]] | `./mount-backend.sh baseline-eval`: the report's 30-seed intercept and detection scenarios and metrics, flown in Gazebo by the 3-defender arc with real detectors |
| [[Surveillance attack test - how to run]] | `./mount-backend.sh attacktest`: 3 defenders on the arc, base station, real detectors; a hit removes the hostile and the defender returns to its slot |
| [[Progress 2026-10-14 evaluation harness]] | Evaluation harness for the report's intercept and detection baselines: what was built, the merge with dev, what is still a placeholder |
| [[Base surveillance plan]] | 3 drones on a quarter circle, option A (widen the sweeps) for the gap while one attacks, central control by the base station with its messages. Decisions open for Jostien |
| [[Baseline attack - how to run]] | The baseline attack (pure pursuit on the detection topics), the fake detector and the attack flight test: how to run them, the contract, the files, first results |
| [[Progress 2026-10-14 evaluation harness]] | Evaluation harness for the report's intercept and detection baselines: what was built, the merge with dev, what is still a placeholder |
| [[Progress 2026-11-05 3D mapping]] | 3D mapping, day 1: DAEP builds on Jazzy, `mount-backend.sh map` builds an OctoMap from a depth-camera drone, the `maptest` flight test, results and what is next |
| [[3D mapping plan]] | On command (centre + radius), one drone explores the area with the DAEP planner and builds a 3D model (OctoMap): pipeline, where the private DAEP code lives, what we must build, order of work |
| [[Lock-on brainstorm]]                         | Mindmap: how the drone finds, confirms and keeps a lock on the hostile drone (cue → acquire → confirm → locked → lost)                              |

## Environment and setup

| Note                  | What it is                                                                                                     |
| --------------------- | -------------------------------------------------------------------------------------------------------------- |
| [[StackV2]]           | Tech stack and dev environment (Swedish): Docker, ROS 2, PX4, Gazebo, Mac/Windows setup, VNC, multi-drone, HPC |
| [[Stack]]             | First version of the stack description (Swedish). Replaced by [[StackV2]]                                      |
| [[WSL2 Docker setup]] | Manual PX4 + Gazebo + ROS 2 setup on Windows/WSL2                                                              |
| [[GHCR How-To]]       | Link: publishing images to GitHub Container Registry                                                           |

Setup docs that live in the repo, outside the vault:

- `docker/README.md`: the dev container, the `start-dev.sh` flags, worlds and drone sensor variants
- `docker/README-stellar.md`: running on the Stellar lab computers (rules, `start-stellar.sh`, cleanup)
- `ros2_ws/src/swarm_backend/README.md`: the backend, `mount-backend.sh` commands, intercept

## Research

| Note                               | What it is                                                       |
| ---------------------------------- | ---------------------------------------------------------------- |
| [[UAS - Initial Landscape]]        | Early questions about UAS and the main technical problems (stub) |
| [[CarlaAir]]                       | Link: CarlaAir simulator                                         |
| `Research/UAV swarms research.pdf` | Background reading                                               |
