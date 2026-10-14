from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.live import best_target, offset_topic
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


def test_topic_names_follow_the_drones_namespace():
    assert offset_topic(2, 'target_offset') == '/px4_2/detection/target_offset'
    assert offset_topic(0, 'target_offset') == '/detection/target_offset'  # drone 0: no prefix
