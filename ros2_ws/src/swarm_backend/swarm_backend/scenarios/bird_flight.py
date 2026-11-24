"""Planned birds flying in their own process (`python3 -m swarm_backend.birds fly`).

Their own process on purpose: a gz-transport call in a scenario's process holds Python's
interpreter lock while Gazebo answers and starves the setpoint stream, and PX4 leaves
offboard (birds.fly_plan). Used by scenario 2 (`--birds`) and the 3-defender runs
(`baseline_eval --birds`).
"""
import dataclasses
import json
import subprocess
import sys


class BirdFlight:
    def __init__(self, world: str, log_path: str = '/tmp/birds.log'):
        self.world, self.log_path = world, log_path
        self._proc: subprocess.Popen | None = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, plans: list, t0: float) -> None:
        """Spawn `plans` (intercept/inbound.py BirdPlan) and fly them from sim time `t0`."""
        plan = {'world': self.world, 't0': t0,
                'birds': [dataclasses.asdict(b) for b in plans]}
        self._proc = subprocess.Popen(
            [sys.executable, '-m', 'swarm_backend.birds', 'fly'], stdin=subprocess.PIPE,
            stdout=open(self.log_path, 'w'), stderr=subprocess.STDOUT, text=True)
        self._proc.stdin.write(json.dumps(plan))
        self._proc.stdin.close()

    def stop(self) -> bool:
        """Stop the process, which removes the birds from Gazebo. False if it had to be
        killed (birds may be left in Gazebo)."""
        proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return True
        proc.terminate()
        try:
            proc.wait(60)
        except subprocess.TimeoutExpired:
            proc.kill()
            return False
        return True
