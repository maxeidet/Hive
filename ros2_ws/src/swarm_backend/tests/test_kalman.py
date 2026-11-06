import math
import random

import numpy as np

from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.fake_detections import (
    Errors, camera_axes, crossing_targets, panning_camera, run)
from swarm_backend.detect.kalman import BoxKalman, rotation_homography, warp


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


# --- camera motion (the gimbal turning) ------------------------------------------------

def _prediction_errors(images, axes, compensated: bool) -> list[float]:
    """Distance from each track's prediction to its target's next detection."""
    camera = CameraModel()
    filters, errors = {}, []
    for i, image in enumerate(images):
        h = rotation_homography(camera, axes[i - 1], axes[i]) if i else None
        if compensated and h is not None:
            for kf in filters.values():
                kf.compensate(h)
        for d, target in zip(image.detections, image.sources):
            box = (d.u, d.v, d.width, d.height)
            if target in filters:
                errors.append(math.dist(filters[target].predict()[:2], box[:2]))
                filters[target].update(box)
            else:
                filters[target] = BoxKalman(box)
    return errors[len(errors) // 3:]  # after the velocities are learnt


def test_same_orientation_changes_nothing():
    camera, axes = CameraModel(), camera_axes(30.0, 5.0)
    h = rotation_homography(camera, axes, axes)
    assert np.allclose(h, np.eye(3))


def test_turning_right_moves_the_image_left():
    camera = CameraModel()
    h = rotation_homography(camera, camera_axes(0.0), camera_axes(5.0))
    u, v = warp(h, camera.cx, camera.cy)
    assert math.isclose(u, camera.cx - camera.f * math.tan(math.radians(5)), abs_tol=1e-6)
    assert math.isclose(v, camera.cy, abs_tol=1e-6)


def test_compensate_moves_the_track_but_keeps_its_own_velocity():
    camera = CameraModel()
    kf = BoxKalman((camera.cx, camera.cy, 10.0, 10.0))
    for i in range(1, 10):
        kf.predict()
        kf.update((camera.cx + 5.0 * i, camera.cy, 10.0, 10.0))
    u_before = kf.box[0]
    assert kf.compensate(rotation_homography(camera, camera_axes(0.0), camera_axes(2.0)))
    assert kf.box[0] < u_before - 12  # 2 degrees ~ 14 px to the left
    assert abs(kf.velocity[0] - 5.0) < 0.5  # the target's own motion is unchanged


def test_turning_away_from_the_track_is_refused():
    camera = CameraModel()
    kf = BoxKalman((camera.cx, camera.cy, 10.0, 10.0))
    assert not kf.compensate(rotation_homography(camera, camera_axes(0.0), camera_axes(180.0)))
    assert kf.box[:2] == (camera.cx, camera.cy)


def test_a_swinging_gimbal_breaks_the_prediction_unless_compensated():
    frames, axes = panning_camera(crossing_targets())
    images = run(frames, 1280, 720, Errors(miss=0.0, weak=0.0, false_per_image=0.0), seed=3)
    without = _prediction_errors(images, axes, compensated=False)
    with_ = _prediction_errors(images, axes, compensated=True)
    mean = lambda e: sum(e) / len(e)
    assert mean(with_) < 4 < 10 < mean(without)


# --- real time between images ---------------------------------------------------------

def test_predict_moves_on_by_dt_steps_and_grows_more_uncertain():
    def learnt():
        kf = BoxKalman((0.0, 0.0, 10.0, 10.0))
        for i in range(1, 15):
            kf.predict()
            kf.update((10.0 * i, 0.0, 10.0, 10.0))
        return kf
    one, three, none = learnt(), learnt(), learnt()
    u1, u3 = one.predict(1.0)[0], three.predict(3.0)[0]
    assert abs((u3 - 140) - 3 * (u1 - 140)) < 0.5  # three times as far
    assert three.centre_std > one.centre_std
    before = (none.box, none.centre_std)
    none.predict(0.0)
    assert (none.box, none.centre_std) == before
