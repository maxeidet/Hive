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
