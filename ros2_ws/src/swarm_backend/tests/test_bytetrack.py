from swarm_backend.detect.bytetrack import ByteTracker, Settings, State, iou
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.fake_detections import Errors, crossing_targets, panning_camera, run
from swarm_backend.detect.kalman import rotation_homography


def box(u: float, v: float, score: float, size: int = 10) -> Detection:
    return Detection(round(u - size / 2), round(v - size / 2), size, size, score)


def ids_per_target(tracker, images, axes=None) -> dict[int, list[int]]:
    """The confirmed track id each target's detections ended up in, image by image."""
    camera, seen = CameraModel(), {}
    for i, image in enumerate(images):
        h = rotation_homography(camera, axes[i - 1], axes[i]) if axes and i else None
        tracker.update(image.detections, h)
        for t in tracker.confirmed:
            source = next(s for d, s in zip(image.detections, image.sources) if d is t.detection)
            if source is not None:
                seen.setdefault(source, []).append(t.id)
    return seen


def switches(ids: list[int]) -> int:
    return sum(a != b for a, b in zip(ids, ids[1:]))


def test_confirmed_after_k_images_and_keeps_its_id():
    tracker = ByteTracker(Settings(k=3))
    for i in range(6):
        tracker.update([box(100 + 5 * i, 100, 0.9)])
        assert [t.id for t in tracker.confirmed] == ([1] if i >= 2 else [])


def test_a_weak_detection_continues_a_track_but_never_starts_one():
    tracker = ByteTracker(Settings(k=2))
    for i in range(3):
        tracker.update([box(100 + 5 * i, 100, 0.9)])
    tracker.update([box(115, 100, 0.3), box(500, 400, 0.3)])  # target weak + clutter
    assert [t.id for t in tracker.confirmed] == [1]
    assert len(tracker.tracks) == 1  # the clutter did not start anything


def test_a_lost_track_comes_back_with_its_id():
    tracker = ByteTracker(Settings(k=2, max_lost=10))
    for i in range(5):
        tracker.update([box(100 + 5 * i, 100, 0.9)])
    for _ in range(3):
        tracker.update([])
    assert tracker.confirmed == [] and tracker.tracks[0].state == State.LOST
    tracker.update([box(140, 100, 0.9)])  # where it would be by now: 100 + 5 * 8
    assert [t.id for t in tracker.confirmed] == [1]


def test_lost_too_long_is_deleted_and_a_tentative_miss_too():
    tracker = ByteTracker(Settings(k=2, max_lost=2))
    for i in range(3):
        tracker.update([box(100, 100, 0.9)])
    for _ in range(3):
        tracker.update([])
    assert tracker.tracks == []
    tracker.update([box(100, 100, 0.9)])
    tracker.update([])
    assert tracker.tracks == []  # one image and gone: probably clutter


def test_crossing_targets_keep_their_ids_with_errors():
    images = run(crossing_targets(), 1280, 720, Errors(), seed=0)
    seen = ids_per_target(ByteTracker(), images)
    assert set(seen) == {1, 2, 3}
    assert all(switches(ids) == 0 for ids in seen.values())


def test_the_swinging_gimbal_needs_the_compensation():
    frames, axes = panning_camera(crossing_targets())
    images = run(frames, 1280, 720, Errors(), seed=0)
    good = ids_per_target(ByteTracker(), images, axes)
    assert set(good) == {1, 2, 3} and all(switches(ids) == 0 for ids in good.values())
    bad = ids_per_target(ByteTracker(), images)  # orientation not used
    assert sum(len(set(ids)) for ids in bad.values()) > 3  # tracks broke and restarted


def test_iou_cost_on_big_boxes_and_greedy_matching():
    for settings in (Settings(k=1, cost='iou'), Settings(k=1, hungarian=False)):
        tracker = ByteTracker(settings)
        for i in range(5):
            tracker.update([box(100 + 3 * i, 100, 0.9, 40), box(300 - 3 * i, 100, 0.9, 40)])
        assert sorted(t.id for t in tracker.confirmed) == [1, 2]


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert iou((0, 0, 10, 10), (20, 0, 10, 10)) == 0.0
    assert abs(iou((0, 0, 10, 10), (5, 0, 10, 10)) - 1 / 3) < 1e-9
