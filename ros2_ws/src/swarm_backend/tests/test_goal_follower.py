"""Unit tests for swarm_backend/explore/goal_follower.py (no ROS needed)."""
import math

import pytest

from swarm_backend.explore.goal_follower import (
    ACTIVE, DONE, IGNORED, NEW, GoalFollower, carrot, enu_quaternion, enu_to_ned, ned_to_enu,
    roll_pitch_yaw, route_home, swap_yaw, wrap)


def heading(yaw: float):
    """PX4 attitude (w, x, y, z) of a level drone with NED yaw `yaw`."""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class FakeDrone:
    """Flies to its setpoint at once when fly() is called."""

    def __init__(self, position=(0.0, 0.0, -2.0), yaw=0.0):
        self.position, self.attitude = position, heading(yaw)
        self.setpoints = []

    def set_position(self, x, y, z, yaw=None, velocity=None):
        self.setpoints.append(((x, y, z), yaw))

    def fly(self):
        position, yaw = self.setpoints[-1]
        self.position, self.attitude = position, heading(yaw)


def run(follower, drone, max_steps=100):
    """Step until the goal is reached; the number of steps it took."""
    for n in range(1, max_steps + 1):
        if follower.step():
            return n
        drone.fly()
    raise AssertionError('goal not reached')


def test_frames():
    assert enu_to_ned((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
    assert ned_to_enu(enu_to_ned((1.0, 2.0, 3.0))) == (1.0, 2.0, 3.0)
    assert swap_yaw(0.0) == pytest.approx(math.pi / 2)  # east is 90 deg from north
    assert swap_yaw(math.pi / 2) == pytest.approx(0.0)  # north
    assert swap_yaw(swap_yaw(1.0)) == pytest.approx(1.0)
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap(math.pi) == pytest.approx(math.pi)


def test_enu_quaternion():
    # Level, nose north: yaw 90 deg in ENU.
    assert enu_quaternion(heading(0.0)) == pytest.approx(
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    # Level, nose east: the identity in ENU.
    assert enu_quaternion(heading(math.pi / 2)) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    # Nose east and up 0.2 rad: positive pitch in NED, negative about ENU's y (left) axis.
    yaw, pitch = math.pi / 2, 0.2
    q_ned = (math.cos(yaw / 2) * math.cos(pitch / 2), -math.sin(yaw / 2) * math.sin(pitch / 2),
             math.cos(yaw / 2) * math.sin(pitch / 2), math.sin(yaw / 2) * math.cos(pitch / 2))
    assert roll_pitch_yaw(q_ned) == pytest.approx((0.0, pitch, yaw))
    assert enu_quaternion(q_ned) == pytest.approx(
        (0.0, -math.sin(pitch / 2), 0.0, math.cos(pitch / 2)))


def test_carrot():
    start, goal = (0.0, 0.0, -2.0), (10.0, 0.0, -2.0)
    assert carrot(start, goal, start, 1.0) == pytest.approx((1.0, 0.0, -2.0))
    # Off the line: the carrot is still on it.
    assert carrot(start, goal, (4.0, 3.0, -1.0), 1.0) == pytest.approx((5.0, 0.0, -2.0))
    assert carrot(start, goal, (9.5, 0.0, -2.0), 1.0) == pytest.approx(goal)
    assert carrot(start, goal, (-3.0, 0.0, -2.0), 1.0) == pytest.approx((1.0, 0.0, -2.0))
    assert carrot(start, (0.5, 0.0, -2.0), start, 1.0) == (0.5, 0.0, -2.0)  # shorter than lead


def test_flies_to_goal_in_ned_and_reports_once():
    drone = FakeDrone()
    follower = GoalFollower(drone, lead=1.0)
    assert not follower.step()  # no goal: commands nothing
    assert drone.setpoints == []

    # 3 m east, 4 m north, 3 m up, facing north (ENU yaw 90 deg).
    assert follower.on_goal('a', (3.0, 4.0, 3.0), math.pi / 2) == NEW
    assert follower.on_goal('a', (3.0, 4.0, 3.0), math.pi / 2) == ACTIVE
    assert not follower.step()
    target, yaw = drone.setpoints[0]
    assert math.dist(target, drone.position) == pytest.approx(1.0)  # one lead ahead
    assert yaw == pytest.approx(0.0)

    run(follower, drone)
    assert math.dist(drone.position, (4.0, 3.0, -3.0)) < follower.position_tolerance
    assert follower.on_goal('a', (3.0, 4.0, 3.0), math.pi / 2) == DONE
    assert not follower.step()  # reported once
    assert follower.on_goal('b', (0.0, 0.0, 2.0), 0.0) == NEW


def test_waits_for_the_yaw():
    drone = FakeDrone(yaw=0.0)
    follower = GoalFollower(drone)
    follower.on_goal('a', (0.0, 0.0, 2.0), 0.0)  # where it is, but facing east
    assert not follower.step()
    drone.fly()
    assert follower.step()
    assert roll_pitch_yaw(drone.attitude)[2] == pytest.approx(math.pi / 2)


def test_initial_motion_is_one_turn_on_the_spot():
    drone = FakeDrone(position=(0.0, 0.0, -2.0), yaw=1.0)
    follower = GoalFollower(drone, turn_step=math.radians(30.0))
    # DAEP sends x = -1 here when the robot is at the origin; it must not move.
    assert follower.on_goal('init', (-1.0, 0.0, 2.0), 2.0, initial_motion=True) == NEW
    assert run(follower, drone) == 13  # 12 steps of 30 deg, each flown before the next
    yaws = [yaw for _, yaw in drone.setpoints]
    assert all(target == (0.0, 0.0, -2.0) for target, _ in drone.setpoints)
    turned = sum(wrap(b - a) for a, b in zip([1.0] + yaws, yaws))
    assert turned == pytest.approx(2 * math.pi)
    assert yaws[-1] == pytest.approx(1.0)  # back where it started


def test_ignores_goals_without_a_position():
    drone = FakeDrone()
    drone.position = None
    follower = GoalFollower(drone)
    assert follower.on_goal('a', (1.0, 0.0, 2.0), 0.0) == IGNORED
    assert not follower.step()


def test_route_home_climbs_first_then_goes_across():
    # Low in the area (z -2 NED = 2 m up), cruise at 23 m: up on the spot, then across.
    assert route_home((10.0, -30.0, -2.0), (0.0, 5.0, -0.1), -23.0) == [
        (10.0, -30.0, -23.0), (0.0, 5.0, -23.0)]


def test_route_home_never_descends_on_the_way():
    # Already above the cruise height: straight across at its own height.
    assert route_home((10.0, -30.0, -25.0), (0.0, 5.0, -0.1), -23.0) == [(0.0, 5.0, -25.0)]
