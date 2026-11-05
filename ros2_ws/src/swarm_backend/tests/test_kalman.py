import math
import random

from swarm_backend.detect.fake_detections import Errors, crossing_targets, run
from swarm_backend.detect.kalman import BoxKalman


def test_learns_the_velocity_from_noisy_boxes():
    rng = random.Random(0)
    kf = BoxKalman((100.0, 200.0, 10.0, 10.0))
    for i in range(1, 20):
        kf.predict()
        kf.update((100 + 12 * i + rng.gauss(0, 1.5), 200 - 3 * i + rng.gauss(0, 1.5),
                   10.0, 10.0))
    du, dv = kf.velocity
    assert abs(du - 12) < 1 and abs(dv + 3) < 1
    u, v, _, _ = kf.predict()
    assert math.dist((u, v), (100 + 12 * 20, 200 - 3 * 20)) < 3


def test_without_detections_it_coasts_and_grows_uncertain():
    kf = BoxKalman((0.0, 0.0, 10.0, 10.0))
    for i in range(1, 6):
        kf.predict()
        kf.update((5.0 * i, 0.0, 10.0, 10.0))
    before = kf.centre_std
    for _ in range(3):
        u, _, _, _ = kf.predict()
    assert 35 < u < 45  # kept going at ~5 px per image: 25 + 3 * 5
    assert kf.centre_std > before


def test_a_detection_makes_it_more_certain():
    kf = BoxKalman((0.0, 0.0, 10.0, 10.0))
    kf.predict()
    before = kf.centre_std
    kf.update((1.0, 0.0, 10.0, 10.0))
    assert kf.centre_std < before


def test_predicts_better_than_the_last_position_on_the_crossing_targets():
    """What the old Confirmer does (compare with the last position) against the filter's
    prediction, on the fake detections of the made-up scenario."""
    frames = crossing_targets()
    images = run(frames, 1280, 720, Errors(miss=0.0, weak=0.0, false_per_image=0.0), seed=3)
    filters, last, kalman_errors, last_errors = {}, {}, [], []
    for image in images:
        for d, target in zip(image.detections, image.sources):
            box = (d.u, d.v, d.width, d.height)
            if target in filters:
                predicted = filters[target].predict()
                kalman_errors.append(math.dist(predicted[:2], box[:2]))
                last_errors.append(math.dist(last[target], box[:2]))
                filters[target].update(box)
            else:
                filters[target] = BoxKalman(box)
            last[target] = box[:2]
    later = len(kalman_errors) // 3  # after the filters have learnt the velocities
    kalman = sum(kalman_errors[later:]) / len(kalman_errors[later:])
    old = sum(last_errors[later:]) / len(last_errors[later:])
    assert kalman < 4 < 8 < old
