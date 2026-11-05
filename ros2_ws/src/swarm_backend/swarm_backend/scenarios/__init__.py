"""Scenarios: a scripted situation flown from start to end and scored from the truth
(not pass/fail tests; those are tests/test_*.py, and tests/test_scenarios.py runs every
scenario that has a fast simulation).

    python3 -m swarm_backend.scenarios NAME [--sim] [args]     (NAME --help for its args)
    ./mount-backend.sh scenario NAME [--sim] [args]             (in the container)

Without --sim a scenario flies PX4 drones in Gazebo (start the container, and the drones
its config needs, first; with or without the Gazebo GUI). With --sim it runs in the fast
simulation (fast_sim.py): no ROS, no PX4, no Gazebo, seconds per run, so it also runs on
the host with only PyYAML and numpy.

Their configs (YAML) and shell launchers are in scenarios/ next to the package.
"""

# name -> (module, has a fast simulation)
SCENARIOS = {
    'attack': ('swarm_backend.scenarios.attack', True),
    'baseline_eval': ('swarm_backend.scenarios.baseline_eval', True),
    'intercept_baseline': ('swarm_backend.scenarios.intercept_baseline', True),
}
