"""MpcIntercept (intercept/mpc_action.py): its tracker, its states, and whole runs
through the evaluation harness (intercept/scenario.py) in the fast simulation, scored
with intercept/metrics.py. No ROS.

These tests bring their own detector:
the true position, optionally with noise, whenever the target is within detect_range.
"""
import dataclasses
import functools
import math
import random

import numpy as np

from swarm_backend.config import InterceptConfig
from swarm_backend.intercept.interceptor import Status
from swarm_backend.intercept.metrics import Outcome, score_run
from swarm_backend.intercept.mpc import MpcWeights
from swarm_backend.intercept.mpc_action import MpcIntercept, TargetTracker3D
from swarm_backend.intercept.scenario import make_scenario, run_scenario
from swarm_backend.intercept.target import PositionFix
from swarm_backend.intercept.target_detector import Detector

from swarm_backend.fast_sim import FastSim, SimDrone

# A camera that sees the target 700 m away (the report's 20 deg camera sees ~680 m):
# with the platform's 51 m there is too little time for any controller at speed.
CONFIG = dataclasses.replace(InterceptConfig(), detect_range=700.0, eval_timeout=150.0)
ZHENG = MpcWeights(tracking=1.0, terminal=0.0, horizon_s=8.0, time_to_go=False)


class TruthDetector(Detector):
    """Sees the target whenever it is within detect_range horizontally (the scenario
    places it there at the start), reporting the truth plus `noise` x range, 1 sigma."""

    def __init__(self, sim, config, rng, noise=0.0):
        self.sim, self.config, self.rng, self.noise = sim, config, rng, noise
        self.silent = False  # tests can switch it off
        self._fix = None
        self._seen = False

    def update(self, now):
        me, target = self.sim.interceptor.position, self.sim.hostile.position
        self._seen = (not self.silent
                      and math.hypot(me[0] - target[0], me[1] - target[1]) <= self.config.detect_range)
        if self._seen:
            std = self.noise * math.dist(me, target)
            self._fix = PositionFix(time=now, source='truth', std=max(std, 0.1),
                                    position=tuple(c + self.rng.gauss(0.0, std) for c in target))

    def latest(self):
        return self._fix

    @property
    def in_view(self):
        return self._seen

    @property
    def detected(self):
        return self._seen


def run(aspect, speed, seed, noise=0.0, **action_options):
    scenario = make_scenario(aspect, speed, seed, CONFIG)
    sim = FastSim(scenario, interceptor_max_speed=CONFIG.interceptor_max_speed)
    detector = lambda s, sc, c: TruthDetector(s, c, random.Random(sc.seed), noise)  # noqa: E731
    action = functools.partial(MpcIntercept, **action_options)
    return score_run(run_scenario(scenario, sim, action, detector, CONFIG), scenario, CONFIG)


# --- the tracker ----------------------------------------------------------------

def test_tracker_estimates_velocity_and_height_from_noisy_positions():
    rng = random.Random(1)
    tracker = TargetTracker3D()
    start, velocity = np.array([500.0, -200.0, -90.0]), np.array([-12.0, 9.0, 1.0])
    for i in range(100):  # 10 s at 10 Hz, 3 m noise
        t = 0.1 * i
        true = start + velocity * t
        tracker.update(PositionFix(time=t, source='test', std=3.0,
                                   position=tuple(true + [rng.gauss(0, 3.0) for _ in range(3)])))
    position, estimated = tracker.estimate(9.9)
    # About 1 m/s off: the filter assumes the target can manoeuvre (ACCEL_NOISE), so it
    # follows a turn quickly rather than averaging the noise away completely.
    assert np.linalg.norm(np.subtract(estimated, velocity)) < 2.0
    assert abs(position[2] - (start[2] + velocity[2] * 9.9)) < 3.0


def test_tracker_predicts_between_fixes_and_restarts_on_a_far_fix():
    tracker = TargetTracker3D()
    for i in range(20):
        tracker.update(PositionFix(time=0.1 * i, source='test', std=0.5,
                                   position=(10.0 * 0.1 * i, 0.0, -50.0)))
    position, _ = tracker.estimate(2.9)  # one second after the last fix at 1.9 s
    assert abs(position[0] - 29.0) < 1.0
    tracker.update(PositionFix(time=3.0, source='test', std=0.5, position=(-500.0, 0.0, -50.0)))
    position, velocity = tracker.estimate(3.0)
    assert position == (-500.0, 0.0, -50.0) and velocity == (0.0, 0.0, 0.0)


# --- states ---------------------------------------------------------------------

class OneDroneSim:
    def __init__(self, target):
        self.interceptor = SimDrone(1, (0.0, 0.0, -50.0))
        self.hostile = SimDrone(0, target)
        self.time = 0.0


def test_status_follows_the_detections():
    sim = OneDroneSim((300.0, 0.0, -50.0))
    detector = TruthDetector(sim, CONFIG, random.Random(0))
    action = MpcIntercept(sim.interceptor, detector, CONFIG, lambda: sim.time)
    assert action.status == Status.IDLE
    action.start()
    action.step()
    assert action.status == Status.LOST  # nothing seen yet
    detector.update(sim.time)
    action.step()
    assert action.status == Status.TRACKING and sim.interceptor._velocity_setpoint is not None
    detector.silent = True
    sim.time += CONFIG.track_timeout + 0.1
    action.step()
    assert action.status == Status.LOST and sim.interceptor._velocity_setpoint is None  # holds
    action.stop()
    assert action.status == Status.IDLE


def test_gimbal_looks_up_at_a_target_above():
    sim = OneDroneSim((300.0, 0.0, -110.0))  # 60 m above the interceptor
    detector = TruthDetector(sim, CONFIG, random.Random(0))
    action = MpcIntercept(sim.interceptor, detector, CONFIG, lambda: sim.time)
    action.start()
    detector.update(0.0)
    action.step()
    pitch, yaw = sim.interceptor.gimbal_aim
    assert abs(pitch - math.degrees(math.atan2(60.0, 300.0))) < 1e-6 and abs(yaw) < 1e-6


# --- whole runs ------------------------------------------------------------------

# Every scenario here can be met, also at the hostile's 17 m/s; e.g. 60 deg and 17 m/s
# cannot: even at top speed from the start the interceptor would be 15 s late.
FEASIBLE = [(0.0, 17.0), (30.0, 17.0), (60.0, 12.0), (90.0, 8.0)]


def test_hits_every_feasible_scenario_with_perfect_detections():
    for aspect, speed in FEASIBLE:
        for seed in range(3):
            result = run(aspect, speed, seed)
            assert result.outcome == Outcome.HIT, (aspect, speed, seed, result.outcome)


def test_the_report_mpc_all_the_way_misses_a_fast_head_on_target_that_this_one_hits():
    """Zheng et al.'s cost with a fixed horizon from first detection (see
    intercept/mpc.py). As the terminal controller after the midcourse it hits more often
    (with the PX4-like drone of fast_sim.py: 2 of 5 all the way, 4 of 5 after it)."""
    alone = [run(0.0, 17.0, seed, weights=ZHENG, terminal_s=None).outcome for seed in range(5)]
    after = [run(0.0, 17.0, seed, weights=ZHENG).outcome for seed in range(5)]
    assert alone.count(Outcome.HIT) < after.count(Outcome.HIT)
    assert after.count(Outcome.HIT) >= 4


def test_the_mpc_all_the_way_falls_behind_where_the_midcourse_does_not():
    """See intercept/mpc_action.py: minimum effort starts slowly and the target gets
    away, though flying straight at full speed would have met it."""
    assert run(60.0, 12.0, 0, terminal_s=None).outcome != Outcome.HIT
    assert run(60.0, 12.0, 0).outcome == Outcome.HIT


def test_mostly_hits_with_noisy_detections():
    """10 % range noise, 1 sigma: 70 m at first detection."""
    hits = sum(run(aspect, speed, seed, noise=0.1).outcome == Outcome.HIT
               for aspect, speed in [(0.0, 17.0), (30.0, 12.0), (60.0, 8.0)] for seed in range(3))
    assert hits >= 8


def test_accelerates_like_pursuit_on_a_px4_like_drone():
    """11 Nov 2026 in Gazebo: v + u dt setpoints gave PX4 so little lead that the MPC took
    5-18 s to reach 4 m/s (pursuit: 1 s). On a drone that follows setpoints like PX4's
    velocity controller, from hover: the midcourse reaches 4 m/s as fast as pursuit
    (1.1 s), the MPC phase in 2.8 s (it was 8.1 s, and missed). The MPC stays a little
    gentler by design: it plans the least acceleration that still meets the target."""
    from swarm_backend.fast_sim import SimDrone

    from swarm_backend.intercept.mpc import PX4_VELOCITY_GAIN
    from swarm_backend.intercept.target import PositionFix

    class Fixed:  # a detector that always sees a target 60 m north, coming in at 5 m/s
        def __init__(self, clock):
            self.clock = clock

        def latest(self):
            t = self.clock()
            return PositionFix(time=t, source='test', std=0.5,
                               position=(60.0 - 5.0 * t, 0.0, -30.0))

    for terminal_s, limit_s in ((5.0, 1.5), (None, 3.5)):  # midcourse / the MPC from the start
        drone = SimDrone(1, (0.0, 0.0, -30.0), max_speed=5.0, max_accel=4.0,
                         velocity_gain=PX4_VELOCITY_GAIN)
        clock = [0.0]
        config = dataclasses.replace(CONFIG, interceptor_max_speed=5.0)
        action = MpcIntercept(drone, Fixed(lambda: clock[0]), config, lambda: clock[0],
                              terminal_s=terminal_s)
        action.start()
        reached = None
        closest = math.inf
        while clock[0] < 10.0:
            action.step()
            drone.advance(0.1)
            clock[0] += 0.1
            if reached is None and math.hypot(*drone.velocity[:2]) >= 4.0:
                reached = clock[0]
            closest = min(closest, math.dist(drone.position, (60.0 - 5.0 * clock[0], 0.0, -30.0)))
        assert reached is not None and reached <= limit_s, (terminal_s, reached)
        assert closest < 1.5, (terminal_s, closest)  # and it gets there
