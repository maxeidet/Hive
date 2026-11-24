import math

import pytest

from swarm_backend.detect.orientation import (
    AttitudeHistory, ClockOffset, axes_error_deg, camera_axes, multiply, rotate, slerp)

LEVEL = (1.0, 0.0, 0.0, 0.0)


def yaw(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))  # about z (down): turns north toward east


def pitch(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, math.sin(h), 0.0)  # about y (right): positive = nose up


def roll(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)  # about x (forward): positive = right side down


def test_level_camera_looks_north_right_is_east_down_is_down():
    forward, right, down = camera_axes(LEVEL, earth_frame=True)
    assert forward == pytest.approx((1, 0, 0))
    assert right == pytest.approx((0, 1, 0))
    assert down == pytest.approx((0, 0, 1))


def test_yaw_east_and_pitch_up():
    forward, _, _ = camera_axes(yaw(90), earth_frame=True)
    assert forward == pytest.approx((0, 1, 0), abs=1e-9)
    forward, _, _ = camera_axes(pitch(30), earth_frame=True)
    assert forward == pytest.approx((math.cos(math.radians(30)), 0, -0.5), abs=1e-9)  # NED: up = -z


def test_roll_shows_in_right_and_down():
    from swarm_backend.detect.camera import camera_roll
    forward, right, _ = camera_axes(roll(25), earth_frame=True)
    assert math.degrees(camera_roll(forward, right)) == pytest.approx(25)


def test_gimbal_relative_to_the_drone_adds_the_drones_attitude():
    forward, _, _ = camera_axes(yaw(30), earth_frame=False, drone_q=yaw(60))
    assert forward == pytest.approx((0, 1, 0), abs=1e-9)
    assert camera_axes(yaw(30), earth_frame=False, drone_q=None) is None


def test_multiply_order_and_rotate():
    q = multiply(yaw(90), pitch(30))  # pitch first, then yaw
    assert rotate(q, (1, 0, 0)) == pytest.approx((0, math.cos(math.radians(30)), -0.5), abs=1e-9)


def test_slerp_goes_halfway_and_takes_the_short_way():
    half = slerp(yaw(0), yaw(90), 0.5)
    assert rotate(half, (1, 0, 0)) == pytest.approx(rotate(yaw(45), (1, 0, 0)))
    flipped = tuple(-c for c in yaw(90))  # the same rotation written the other way
    assert rotate(slerp(yaw(0), flipped, 0.5), (1, 0, 0)) == pytest.approx(
        rotate(yaw(45), (1, 0, 0)))


def test_history_interpolates_between_reports_and_refuses_far_times():
    history = AttitudeHistory()
    history.add(10.0, yaw(0))
    history.add(10.2, yaw(20))
    q = history.at(10.1)
    assert rotate(q, (1, 0, 0)) == pytest.approx(rotate(yaw(10), (1, 0, 0)))
    assert history.at(10.25) is not None  # just after the last one: the nearest
    assert history.at(20.0) is None
    history.add(10.1, yaw(5))  # older than the newest: ignored
    assert rotate(history.at(10.2), (1, 0, 0)) == pytest.approx(rotate(yaw(20), (1, 0, 0)))


def test_clock_offset_takes_the_least_delayed_pair():
    clock = ClockOffset()
    assert clock.to_px4(5.0) is None
    clock.update(1000.10, 0.20)  # arrived 0.1 s late
    clock.update(1000.50, 0.52)  # arrived 0.02 s late
    assert clock.value == pytest.approx(999.98)
    assert clock.to_px4(1.0) == pytest.approx(1000.98)


def test_axes_error():
    a = camera_axes(LEVEL, True)
    b = camera_axes(multiply(yaw(2), roll(5)), True)
    forward, roll_error = axes_error_deg(a, b)
    assert forward == pytest.approx(2, abs=0.01)
    assert roll_error == pytest.approx(5, abs=0.01)


def test_the_position_history_interpolates_and_restarts():
    from swarm_backend.detect.orientation import PositionHistory
    h = PositionHistory()
    assert h.at(1.0) is None
    h.add(1.0, (0.0, 0.0, -30.0))
    h.add(1.1, (1.5, 0.0, -30.0))
    assert h.at(1.04) == pytest.approx((0.6, 0.0, -30.0))
    assert h.at(1.3) == (1.5, 0.0, -30.0)  # after the last, within the tolerance
    assert h.at(2.0) is None
    h.add(1.05, (9.0, 9.0, 9.0))  # out of order: ignored
    assert h.at(1.05) == pytest.approx((0.75, 0.0, -30.0))
    h.add(0.0, (0.0, 0.0, 0.0))  # the simulation restarted
    assert h.at(1.0) is None and h.at(0.0) == (0.0, 0.0, 0.0)
