# AI & Agent Instructions (Hive Project)

These instructions are automatically loaded by Gemini/Antigravity and its sub-agents whenever they work in this project.

## 1. Project Context
* **Environment:** We are using ROS 2 , PX4 Autopilot, and Gazebo.
* **Build Environment:** 6 Developers on different OS, has the be compatibale for all developers
* **Goal:** Develop a simulation-based, self-healing cooperative drone swarm ("the hive") in collaboration with the Swedish Armed Forces (Försvarsmakten). The swarm acts as a single entity to protect a home base and scout surroundings.
* **Our Subproblems:**
  1. Base Surveillance: Persistent monitoring and situational awareness over the base.
  2. Counter-UAS: Detect and intercept hostile drones near protected assets (comparing planning-based vs Reinforcement Learning controllers).
* **Team Context:** Part of the LiU InDefense specialization. We are one of several groups sharing the same swarm system, so code must adhere to shared conventions and common interfaces.

## 2. Technical Standards
* **Python* 
* **Submodules:** PX4 is included as a git submodule in `PX4-Autopilot`. Never edit the core code inside PX4 unless explicitly requested. Instead, write our own code in ROS 2 nodes.
* **Build Commands:** The main compilation for the simulator is done via `make px4_sitl ign_x500` from inside the container.

## 3. Agent Behavior
* Before running heavy build commands in the terminal, always ask for permission first to save processing power.
* When proposing architectural changes, always consider that the code must work for the entire team (Mac, Windows, Linux).
