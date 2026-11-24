"""The friendly-drone check (swarm/friendlies.py): pointing at a friendly, with a fitting
distance, means friendly; the broadcast is predicted on and expires."""
import math

import pytest

from swarm_backend.swarm.friendlies import (CONFIRM_FRAMES, MAX_AGE_S, RELEASE_FRAMES, Friendly,
                                           FriendlyBook, FriendlyMarks, decode, encode,
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
                  Friendly(1, (1.5, -2.0, -30.0), (3.0, 0.5, 0.0), 12.25)]
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


def test_each_friendly_is_predicted_from_its_own_time():
    """Drone 1's odometry arrived 0.1 s before the broadcast: at 15 m/s that is 1.5 m the
    message's time alone would leave out."""
    book = FriendlyBook()
    book.update(10.1, [Friendly(1, (0.0, 0.0, -30.0), (15.0, 0.0, 0.0), 10.0),
                       Friendly(2, (5.0, 0.0, -30.0), (15.0, 0.0, 0.0))])  # no own time
    assert book.at(10.2)[1] == pytest.approx((3.0, 0.0, -30.0))
    assert book.at(10.2)[2] == pytest.approx((6.5, 0.0, -30.0))
    assert book.ages(10.2) == pytest.approx({1: 0.2, 2: 0.1})
    assert book.get(1)[0] == 10.0 and book.get(2)[0] == 10.1 and book.get(3) is None
    assert 1 not in book.at(10.0 + MAX_AGE_S + 0.05)  # its own time is too old
    assert 2 in book.at(10.0 + MAX_AGE_S + 0.05)


def test_the_gate_uses_each_friendly_s_own_age():
    friend = (20.0, 0.0, -30.0)
    off = gate_deg(20.0) + 1.0  # just outside the gate of a fresh position
    assert friendly_for(towards(friend, off), 20.0, CAMERA, {1: friend}, {1: 0.0}) is None
    assert friendly_for(towards(friend, off), 20.0, CAMERA, {1: friend}, {1: 1.0}) == 1


def test_a_track_is_friendly_after_frames_in_a_row_and_stops_after_misses():
    marks = FriendlyMarks()
    for _ in range(CONFIRM_FRAMES - 1):
        assert marks.update({5: 0}, {5}) == {}
    assert marks.update({5: None}, {5}) == {}  # a miss starts it over
    for _ in range(CONFIRM_FRAMES - 1):
        assert marks.update({5: 0}, {5}) == {}
    assert marks.update({5: 1}, {5}) == {}  # another friendly: starts over too
    for _ in range(CONFIRM_FRAMES - 1):
        marks.update({5: 1}, {5})
    assert marks.update({5: 1}, {5}) == {5: 1}
    for _ in range(RELEASE_FRAMES - 1):
        assert marks.update({5: None}, {5}) == {5: 1}
    assert marks.update({5: 1}, {5}) == {5: 1}  # fits again: the misses start over
    assert marks.update({}, {5}) == {5: 1}  # no box in this image: unchanged
    for _ in range(RELEASE_FRAMES - 1):
        marks.update({5: None}, {5})
    assert marks.update({5: None}, {5}) == {}
    marks.update({6: 0}, {6})
    assert marks.update({}, set()) == {} and not marks._hits  # gone tracks are forgotten


def test_the_book_follows_a_restarted_simulation():
    book = FriendlyBook()
    book.update(500.0, [Friendly(1, (0.0, 0.0, -30.0), (0.0, 0.0, 0.0))])
    book.update(499.5, [])  # a late message: ignored
    assert 1 in book.at(500.0)
    book.update(3.0, [Friendly(2, (1.0, 0.0, -30.0), (0.0, 0.0, 0.0))])  # clock back to 0
    assert set(book.at(3.1)) == {2}
