"""The MPC's maths (intercept/mpc.py) on its own, then closed loop against the fast
simulation's drone, with the target's true position and velocity (the report's
"perfect detections"). No ROS."""
import math

import numpy as np

from swarm_backend.intercept.mpc import (Mpc, MpcWeights, closest_approach_time,
                                         free_positions, prediction_matrix, time_to_go,
                                         velocity_command)

from sim import SimDrone

DT = 0.1
ALT = -50.0
V_I = 12.0  # interceptor top speed, InterceptConfig.interceptor_max_speed
A_MAX = SimDrone.MAX_ACCEL
HIT = 1.5
# The report's MPC (Zheng et al.): every step weighted, fixed horizon.
ZHENG = MpcWeights(tracking=1.0, terminal=0.0, horizon_s=8.0, time_to_go=False)


# --- the maths --------------------------------------------------------------

def test_prediction_matrix_matches_constant_acceleration_kinematics():
    steps, a = 30, np.array([1.0, -2.0, 0.5])
    p0, v0 = np.array([3.0, 4.0, ALT]), np.array([1.0, 0.0, -1.0])
    positions = free_positions(p0, v0, steps, DT) + prediction_matrix(steps, DT) @ np.tile(a, (steps, 1))
    t = DT * np.arange(1, steps + 1)[:, None]
    assert np.allclose(positions, p0 + v0 * t + 0.5 * a * t * t)


def test_no_acceleration_when_already_flying_with_the_target():
    mpc = Mpc()
    u = mpc.first_acceleration((0.0, 0.0, ALT), (5.0, 1.0, 0.0), (0.0, 0.0, ALT), (5.0, 1.0, 0.0))
    assert math.hypot(*u) < 1e-9


def test_accelerates_toward_a_still_target_including_up():
    mpc = Mpc()
    ux, uy, uz = mpc.first_acceleration((0.0, 0.0, ALT), (0.0, 0.0, 0.0),
                                        (100.0, 0.0, ALT - 30.0), (0.0, 0.0, 0.0))
    assert ux > 0 and abs(uy) < 1e-9 and uz < 0  # NED: up is negative z
    assert abs(math.atan2(-uz, ux) - math.atan2(30.0, 100.0)) < 1e-9  # straight at it


def test_plan_ends_at_the_predicted_meeting_point():
    """With a heavy terminal weight the whole plan, flown unlimited, ends at the target."""
    mpc = Mpc(MpcWeights(terminal=1e6))
    me, v, tgt, tv = (0.0, 0.0, ALT), (0.0, 0.0, 0.0), (-200.0, 150.0, ALT), (10.0, -14.0, 0.0)
    plan = mpc.plan(me, v, tgt, tv)
    end = mpc.trajectory(me, v, plan)[-1]
    met = free_positions(tgt, tv, len(plan), DT)[-1]
    assert np.linalg.norm(end - met) < 0.1


def test_time_to_go():
    # Head-on: 290 m apart, closing at 12 + 17 m/s.
    assert abs(time_to_go((0.0, 0.0, 0.0), (290.0, 0.0, 0.0), (-17.0, 0.0, 0.0), V_I) - 10.0) < 1e-9
    # Still target: distance / speed.
    assert abs(time_to_go((0.0, 0.0, 0.0), (0.0, 120.0, 0.0), (0.0, 0.0, 0.0), V_I) - 10.0) < 1e-9
    # A faster target flying away is never met; its closest approach is now.
    assert time_to_go((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (17.0, 0.0, 0.0), V_I) is None
    assert closest_approach_time((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (17.0, 0.0, 0.0)) == 0.0


def test_velocity_command_keeps_the_limits():
    # A huge request is cut to A_MAX ...
    v = velocity_command((5.0, 0.0, 0.0), (500.0, 500.0, 0.0), DT, A_MAX, V_I)
    assert abs(math.hypot(v[0] - 5.0, v[1]) - A_MAX * DT) < 1e-9
    # ... and the result to V_I.
    v = velocity_command((V_I, 0.0, 0.0), (A_MAX, 0.0, 0.0), DT, A_MAX, V_I)
    assert abs(math.hypot(*v) - V_I) < 1e-9


# --- closed loop --------------------------------------------------------------

def straight(start, direction, speed):
    n = math.hypot(*direction)
    velocity = tuple(speed * c / n for c in direction)
    return lambda t: (tuple(p + v * t for p, v in zip(start, velocity)), velocity)


def circle(centre, radius, speed):
    w = speed / radius
    return lambda t: ((centre[0] + radius * math.cos(w * t), centre[1] + radius * math.sin(w * t),
                       centre[2]), (-speed * math.sin(w * t), speed * math.cos(w * t), 0.0))


def mpc_steer(weights):
    mpc = Mpc(weights, max_speed=V_I)

    def steer(drone, target, target_velocity):
        u = mpc.first_acceleration(drone.position, drone.velocity, target, target_velocity)
        drone.set_velocity(*velocity_command(drone.velocity, u, DT, A_MAX, V_I))
    return steer


def pursuit_steer(drone, target, _):
    r = [t - p for t, p in zip(target, drone.position)]
    n = math.hypot(*r)
    drone.set_velocity(*(V_I * c / n for c in r))


def fly(steer, target_at, start=(0.0, 0.0, ALT), seconds=60.0):
    """Seconds to a hit (within HIT, checked along each step, as the harness does), or
    None, and the closest distance."""
    drone = SimDrone(1, start, max_speed=V_I)
    closest = math.inf
    for i in range(round(seconds / DT)):
        t = i * DT
        target, target_velocity = target_at(t)
        steer(drone, target, target_velocity)
        before = np.subtract(drone.position, target)
        drone.advance(DT)
        after = np.subtract(drone.position, target_at(t + DT)[0])
        d = after - before
        s = 0.0 if d @ d == 0 else min(1.0, max(0.0, -(before @ d) / (d @ d)))
        closest = min(closest, float(np.linalg.norm(before + s * d)))
        if closest <= HIT:
            return t + DT, closest
    return None, closest


CROSSING = ((-220.0, 220.0, ALT), (280.0, -520.0, 0.0))  # passes 89 m from the interceptor


def test_hits_a_crossing_target_at_every_speed_including_faster_than_itself():
    for speed in (4.0, 8.0, 12.0, 17.0):
        hit, closest = fly(mpc_steer(MpcWeights()), straight(*CROSSING, speed))
        assert hit is not None, f'{speed} m/s: missed by {closest:.1f} m'


def test_hits_a_faster_target_that_pure_pursuit_misses():
    target = straight(*CROSSING, 17.0)
    assert fly(pursuit_steer, target)[0] is None
    assert fly(mpc_steer(MpcWeights()), target)[0] is not None


def test_hits_earlier_than_pure_pursuit_when_both_hit():
    target = straight(*CROSSING, 8.0)
    pursuit, _ = fly(pursuit_steer, target)
    mpc, _ = fly(mpc_steer(MpcWeights()), target)
    assert pursuit is not None and mpc is not None and mpc < pursuit


def test_hits_a_target_at_another_altitude():
    hit, closest = fly(mpc_steer(MpcWeights()),
                       straight((200.0, 250.0, ALT - 40.0), (-0.6, -1.0, 0.0), 17.0))
    assert hit is not None, f'missed by {closest:.1f} m'


def test_hits_a_circling_target():
    """Zheng et al.'s outdoor test: a target on a circle at 4 m/s."""
    hit, closest = fly(mpc_steer(MpcWeights()), circle((60.0, 40.0, ALT), 20.0, 4.0))
    assert hit is not None, f'missed by {closest:.1f} m'


def test_the_report_mpc_with_a_fixed_horizon_misses_a_faster_target():
    """Why the defaults differ from Zheng et al.: weighting every step with a fixed
    horizon flies toward where the target is, like pursuit, and a faster target gets
    past. It still hits a slow one."""
    assert fly(mpc_steer(ZHENG), straight(*CROSSING, 17.0))[0] is None
    assert fly(mpc_steer(ZHENG), straight(*CROSSING, 4.0))[0] is not None
