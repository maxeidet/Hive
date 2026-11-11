"""The friendly-drone check (hive/friendlies.py): pointing at a friendly, with a fitting
distance, means friendly; the broadcast is predicted on and expires."""
import math

import pytest

from swarm_backend.hive.friendlies import (MAX_AGE_S, Friendly, FriendlyBook, decode, encode,
                                           friendly_for, gate_deg)

CAMERA = (0.0, 0.0, -30.0)  # the defender, 30 m up above the base


def unit(v):
    n = math.hypot(*v)
    return tuple(c / n for c in v)


def towards(point, off_deg=0.0):
    """The direction from CAMERA to `point`, turned `off_deg` to the right (about down)."""
    n, e, d = (p - c for p, c in zip(point, CAMERA))
    a = math.radians(off_deg)
    return unit((n * math.cos(a) - e * math.sin(a), n * math.sin(a) + e * math.cos(a), d))


def test_the_message_round_trips():
    friendlies = [Friendly(0, (0.0, 0.0, -0.2), (0.0, 0.0, 0.0)),
                  Friendly(1, (1.5, -2.0, -30.0), (3.0, 0.5, 0.0))]
    stamp, back = decode(encode(12.3456, friendlies))
    assert stamp == pytest.approx(12.346) and back == friendlies


def test_a_box_on_the_parked_drone_is_friendly():
    parked = (20.0, 0.0, -0.2)  # 36 m away, below
    d = math.dist(parked, CAMERA)
    assert friendly_for(towards(parked), d * 1.3, CAMERA, {0: parked}) == 0
    assert friendly_for(towards(parked, 2.0), d, CAMERA, {0: parked}) == 0  # within the gate
    assert friendly_for(towards(parked, 8.0), d, CAMERA, {0: parked}) is None  # elsewhere


def test_a_hostile_behind_a_friendly_is_not_friendly():
    friend = (10.0, 0.0, -30.0)  # 10 m ahead
    # Same direction, but the box says about 80 m: a hostile far behind it.
    assert friendly_for(towards(friend), 80.0, CAMERA, {1: friend}) is None
    assert friendly_for(towards(friend), 14.0, CAMERA, {1: friend}) == 1
    assert friendly_for(towards(friend), None, CAMERA, {1: friend}) == 1  # no size: careful


def test_the_gate_is_wider_close_in_and_when_predicted_further():
    assert gate_deg(10.0) > gate_deg(50.0) > gate_deg(500.0) > 2.0
    assert gate_deg(30.0, age_s=1.0) > gate_deg(30.0)


def test_the_camera_s_own_drone_is_skipped():
    assert friendly_for(towards((10.0, 0.0, -30.0)), 10.0, CAMERA, {1: CAMERA}) is None


def test_the_book_predicts_with_velocity_and_expires():
    book = FriendlyBook()
    assert book.at(0.0) == {}
    book.update(10.0, [Friendly(1, (0.0, 0.0, -30.0), (2.0, 0.0, 0.0))])
    assert book.at(10.5)[1] == pytest.approx((1.0, 0.0, -30.0))
    assert book.at(10.0 + MAX_AGE_S + 0.1) == {}
    book.update(9.0, [])  # an older message does not replace a newer one
    assert 1 in book.at(10.5)
