import math

from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.live import TargetLock, best_target, offset_topic
from swarm_backend.detect.tracking import Track


def track(id_, score, confirmed=True, seen=True):
    detection = Detection(100, 100, 6, 6, score) if seen else None
    return Track(id_, (103, 103), detection, confirmed=confirmed)


def test_publishes_the_confirmed_track_with_the_highest_score():
    tracks = [track(1, 80), track(2, 120), track(3, 200, confirmed=False)]
    assert best_target(tracks).id == 2


def test_nothing_without_a_confirmed_track_seen_in_this_image():
    assert best_target([track(1, 80, confirmed=False), track(2, 90, seen=False)]) is None
    assert best_target([]) is None


# --- --lock (TargetLock) ------------------------------------------------------------

def bearing(deg):
    """A world direction `deg` degrees east of north, level."""
    return (math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0)


def lock_tracks(*specs):
    """Tracks from (id, score, bearing deg, seen), and their directions."""
    tracks, where = [], {}
    for id_, score, deg, seen in specs:
        tracks.append(track(id_, score, seen=seen))
        where[id_] = bearing(deg)
    return tracks, lambda t, predicted: where[t.id]


def test_lock_takes_the_best_target_first():
    tracks, direction = lock_tracks((1, 80, -20, True), (2, 120, 20, True))
    lock = TargetLock()
    assert lock.update(tracks, 0.0, direction).id == 2
    assert lock.locked_id == 2 and 'T2 locked' in lock.event


def test_lock_stays_on_its_drone_when_another_scores_higher():
    lock = TargetLock()
    lock.update(*_with(lock_tracks((1, 80, -20, True)), 0.0))
    tracks, direction = lock_tracks((1, 80, -20, True), (2, 300, 20, True))
    assert lock.update(tracks, 0.1, direction).id == 1
    assert lock.event is None


def test_lock_hands_over_to_a_new_id_on_the_same_drone():
    lock = TargetLock()
    lock.update(*_with(lock_tracks((1, 80, -20, True)), 0.0))
    # T1 lost (ByteTrack predicts it on at -19 deg), the same drone comes back as T5 at
    # -18 deg, the other drone T2 scores higher 40 deg away.
    tracks, direction = lock_tracks((1, 80, -19, False), (5, 70, -18, True), (2, 300, 20, True))
    assert lock.update(tracks, 0.3, direction).id == 5
    assert lock.locked_id == 5 and 'handover' in lock.event


def test_lock_is_silent_through_a_short_gap_then_moves_on():
    lock = TargetLock(timeout=1.0)
    lock.update(*_with(lock_tracks((1, 80, -20, True)), 0.0))
    tracks, direction = lock_tracks((1, 80, -20, False), (2, 300, 20, True))
    assert lock.update(tracks, 0.5, direction) is None  # T2 is far from where T1 should be
    assert lock.locked_id == 1
    assert lock.update(tracks, 1.2, direction).id == 2  # 1.2 s unseen: released
    assert 'T1 released' in lock.event and 'T2 locked' in lock.event


def test_lock_released_when_the_tracker_drops_it_and_nothing_is_there():
    lock = TargetLock(timeout=1.0)
    lock.update(*_with(lock_tracks((1, 80, -20, True)), 0.0))
    assert lock.update([], 0.5, lambda t, p: None) is None and lock.locked_id == 1
    assert lock.update([], 2.0, lambda t, p: None) is None and lock.locked_id is None


def test_lock_gate_widens_while_unseen_but_never_reaches_the_other_drone():
    lock = TargetLock()
    assert lock.gate(0.0) == 4.0
    assert lock.gate(0.5) == 6.5
    assert lock.gate(10.0) == 10.0  # capped: scenario 2's hostiles are ~40 deg apart


def test_lock_without_directions_still_holds_and_times_out():
    lock = TargetLock(timeout=1.0)
    lock.update([track(1, 80)], 0.0, lambda t, p: None)
    assert lock.update([track(1, 80, seen=False), track(2, 300)], 0.5,
                       lambda t, p: None) is None
    assert lock.update([track(2, 300)], 1.5, lambda t, p: None).id == 2


def _with(tracks_and_direction, stamp):
    tracks, direction = tracks_and_direction
    return tracks, stamp, direction


def test_topic_names_follow_the_drones_namespace():
    assert offset_topic(2, 'target_offset') == '/px4_2/detection/target_offset'
    assert offset_topic(0, 'target_offset') == '/detection/target_offset'  # drone 0: no prefix


def test_a_gimbal_turn_keeps_the_track_when_matching_by_world_direction():
    import math

    from swarm_backend.detect.camera import CameraModel
    from swarm_backend.detect.live import directions, world_confirmer
    from swarm_backend.detect.orientation import camera_axes

    camera = CameraModel()
    target = (math.cos(0.2), 0.0, -math.sin(0.2))  # a still target, north and a bit up
    confirmer = world_confirmer(k=3, gate_deg=10)
    pixel_jumps = []
    for i, yaw_deg in enumerate((0, 6, 12)):  # the gimbal turns 6 degrees between images
        h = math.radians(yaw_deg) / 2
        axes = camera_axes((math.cos(h), 0.0, 0.0, math.sin(h)), earth_frame=True)
        forward, right, down = axes
        depth = sum(a * b for a, b in zip(target, forward))
        u = camera.cx + camera.f * sum(a * b for a, b in zip(target, right)) / depth
        v = camera.cy + camera.f * sum(a * b for a, b in zip(target, down)) / depth
        box = Detection(round(u) - 3, round(v) - 3, 6, 6, 100.0)
        pixel_jumps.append(u)
        tracks = confirmer.update(directions([box], camera, axes), [box])
    assert pixel_jumps[0] - pixel_jumps[1] > 20  # in pixels it jumped past the 20 px gate
    assert len(tracks) == 1 and tracks[0].confirmed  # in the world it stayed one track


# --- ByteTrack in the live detector ----------------------------------------------------

def _args(argv):
    import argparse

    from swarm_backend.detect import detectors
    from swarm_backend.detect.live import add_tracker_options

    parser = argparse.ArgumentParser()
    parser.add_argument('--detector', default='baseline')
    parser.add_argument('--k', type=int, default=3)
    detectors.add_options(parser)
    add_tracker_options(parser)
    return parser.parse_args(argv)


def test_confirmer_is_still_the_default():
    from swarm_backend.detect.live import make_tracker

    assert make_tracker(_args([]), []) is None


def test_bytetrack_with_the_baseline_lowers_its_contrast_to_low():
    from swarm_backend.detect.live import TRACK_SCORES, make_tracker

    argv = ['--tracker', 'bytetrack']
    args = _args(argv)
    tracker = make_tracker(args, argv)
    high, low, new = TRACK_SCORES['baseline']
    s = tracker.settings
    assert (s.high, s.low, s.new, s.k) == (high, low, new, 3)
    assert args.contrast == low  # the detector now reports the weak candidates too
    argv = ['--tracker', 'bytetrack', '--contrast', '45', '--high', '70']
    args = _args(argv)
    tracker = make_tracker(args, argv)
    assert args.contrast == 45 and tracker.settings.high == 70  # given: kept


def test_bytetrack_with_yolo_lowers_its_confidence_to_low():
    from swarm_backend.detect.live import TRACK_SCORES, make_tracker

    argv = ['--detector', 'yolo', '--tracker', 'bytetrack']
    args = _args(argv)
    tracker = make_tracker(args, argv)
    assert args.conf == TRACK_SCORES['yolo'][1] == tracker.settings.low  # 0.1, not 0.25
    argv = ['--detector', 'yolo', '--tracker', 'bytetrack', '--conf', '0.3']
    args = _args(argv)
    make_tracker(args, argv)
    assert args.conf == 0.3  # given: kept
    argv = ['--detector', 'yolo']
    args = _args(argv)
    assert make_tracker(args, argv) is None and args.conf == 0.25  # the Confirmer: unchanged


def test_live_bytetrack_follows_a_target_while_the_gimbal_turns():
    from swarm_backend.detect.bytetrack import ByteTracker, Settings
    from swarm_backend.detect.camera import CameraModel
    from swarm_backend.detect.fake_detections import camera_axes
    from swarm_backend.detect.kalman import rotation_homography, warp
    from swarm_backend.detect.live import LiveDetector

    live = LiveDetector.__new__(LiveDetector)  # without ROS: only the tracking part
    live.tracker, live._last_axes = ByteTracker(Settings(high=60, low=30, new=60)), None
    camera = CameraModel()
    still = camera_axes(0.0, 10.0)
    for i, yaw in enumerate([0, 3, 6, 9, 12, 15]):  # 3 degrees (~22 px) per image
        axes = camera_axes(yaw, 10.0)
        u, v = warp(rotation_homography(camera, still, axes), 700.0, 300.0)
        tracks, matching = live._byte_track([Detection(round(u) - 3, round(v) - 3, 6, 6, 80.0)],
                                            camera, axes)
        assert matching == ('pixels' if i == 0 else 'pixels+gimbal')
    assert [t.id for t in tracks if t.confirmed] == [1]


def test_annotate_draws_bytetrack_tracks_with_lost_ones():
    import numpy as np

    from swarm_backend.detect.bytetrack import ByteTracker, Settings, State
    from swarm_backend.detect.live import Result, annotate

    tracker = ByteTracker(Settings(k=2, high=60, low=30, new=60))
    for i in range(3):
        tracker.update([Detection(100 + 5 * i, 100, 6, 6, 80.0)])
    tracker.update([])
    assert tracker.tracks[0].state == State.LOST
    img = np.full((240, 320, 3), 200, np.uint8)
    out = annotate(img, Result(0.0, 320, 240, [], list(tracker.tracks), None, 1.0))
    assert out.shape == img.shape and (out != img).any()  # the lost box was drawn


def test_birds_are_kept_out_of_tracking_and_drawn_green():
    import numpy as np

    from swarm_backend.detect.live import BIRD, Result, annotate, result_record, split_birds

    drone = Detection(50, 50, 8, 8, 0.7)
    bird = Detection(200, 100, 10, 6, 0.6, 'bird')
    boxes, birds = split_birds([drone, bird])
    assert boxes == [drone] and birds == [bird]
    img = np.full((240, 320, 3), 200, np.uint8)
    out = annotate(img, Result(0.0, 320, 240, boxes, [], None, 1.0, birds=birds))
    assert (out[bird.y - 6, bird.x:bird.x + 10] == BIRD).all()  # the green frame above it
    rec = result_record(Result(0.0, 320, 240, boxes, [], None, 1.0, birds=birds))
    assert len(rec['raw']) == 1 and len(rec['birds']) == 1  # the replay sees drones only


def test_a_track_on_a_friendly_drone_is_marked_and_kept_from_the_lock():
    """The hivemind says drone 0 is parked on the base, below the defender (drone 1): the
    box that points at it is friendly for its whole life, the other one is not."""
    import numpy as np

    from swarm_backend.detect.camera import CameraModel
    from swarm_backend.detect.live import FRIEND, LiveDetector, Result, annotate
    from swarm_backend.detect.tracking import Track
    from swarm_backend.swarm.friendlies import Friendly, FriendlyBook

    camera = CameraModel()
    # Camera 30 m up, looking north and 45 deg down: drone 0 at 30 m north on the ground
    # is straight ahead, in the middle of the image.
    s = math.sqrt(0.5)
    axes = ((s, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, s))  # forward, right, down
    me, parked = (0.0, 0.0, -30.0), (30.0, 0.0, 0.0)
    live = LiveDetector.__new__(LiveDetector)
    live.drone_id, live.detector_name, live.friendly = 1, 'yolo', {}
    live.friend_book = FriendlyBook()
    live.friend_book.update(5.0, [Friendly(0, parked, (0.0, 0.0, 0.0)),
                                  Friendly(1, me, (0.0, 0.0, 0.0))])
    width = round(camera.f * 0.8 / math.dist(me, parked))  # a drone's box at that distance
    on_friend = Track(1, (camera.cx, camera.height / 2))
    on_friend.detection = Detection(round(camera.cx - width / 2),
                                    round(camera.height / 2 - width / 2), width, width, 0.9)
    elsewhere = Track(2, (200.0, 100.0))
    elsewhere.detection = Detection(196, 96, 8, 8, 0.5)
    live._mark_friendlies([on_friend, elsewhere], camera, axes, 5.1)
    assert live.friendly == {1: 0}
    on_friend.detection = None  # unseen for a while: still friendly
    live._mark_friendlies([on_friend, elsewhere], camera, axes, 5.2)
    assert live.friendly == {1: 0}
    live._mark_friendlies([elsewhere], camera, axes, 5.3)  # its track is gone
    assert live.friendly == {}

    img = np.full((camera.height, camera.width, 3), 200, np.uint8)
    on_friend.detection = Detection(600, 300, 10, 10, 0.9)
    out = annotate(img, Result(0.0, camera.width, camera.height, [], [on_friend], None, 1.0,
                               friendly={1: 0}))
    assert (out[300 - 6, 600:610] == FRIEND).all()  # drawn blue, not red


def test_without_the_hivemind_nothing_is_friendly():
    from swarm_backend.detect.camera import CameraModel
    from swarm_backend.detect.live import LiveDetector
    from swarm_backend.detect.tracking import Track
    from swarm_backend.swarm.friendlies import FriendlyBook

    live = LiveDetector.__new__(LiveDetector)
    live.drone_id, live.detector_name, live.friendly = 1, 'yolo', {}
    live.friend_book = FriendlyBook()
    track = Track(1, (640.0, 360.0))
    track.detection = Detection(636, 356, 8, 8, 0.9)
    live._mark_friendlies([track], CameraModel(), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                                  (0.0, 0.0, 1.0)), 1.0)
    assert live.friendly == {}
