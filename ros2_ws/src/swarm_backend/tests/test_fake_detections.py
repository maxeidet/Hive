import random

from swarm_backend.detect.fake_detections import (
    PERFECT, Errors, TruthBox, crossing_targets, detect, run, targets_from_truth)


def test_perfect_detector_reports_the_truth():
    truth = [TruthBox(1, 100.0, 50.0, 10.0, 10.0), TruthBox(2, 300.0, 200.0, 6.0, 6.0)]
    image = detect(truth, 640, 480, PERFECT, random.Random(0))
    assert sorted(image.sources) == [1, 2]
    for d, source in zip(image.detections, image.sources):
        t = next(t for t in truth if t.id == source)
        assert (d.u, d.v, d.width, d.height) == (t.u, t.v, t.width, t.height)
        assert 0.6 <= d.score <= 0.95


def test_errors_show_up_at_about_their_rates():
    errors = Errors(miss=0.2, weak=0.3, false_per_image=1.0)
    frames = [[TruthBox(1, 320.0, 240.0, 10.0, 10.0)]] * 2000
    images = run(frames, 640, 480, errors, seed=1)
    true = [d for im in images for d, s in zip(im.detections, im.sources) if s == 1]
    false = [d for im in images for d, s in zip(im.detections, im.sources) if s is None]
    assert abs(1 - len(true) / 2000 - 0.2) < 0.03
    assert abs(sum(d.score <= 0.5 for d in true) / len(true) - 0.3) < 0.04
    assert abs(len(false) / 2000 - 1.0) < 0.1
    assert all(d.score <= 0.5 for d in false)


def test_detections_are_sorted_and_repeatable():
    frames = crossing_targets()
    a, b = run(frames, 1280, 720, seed=7), run(frames, 1280, 720, seed=7)
    assert a == b
    for image in a:
        scores = [d.score for d in image.detections]
        assert scores == sorted(scores, reverse=True)


def test_crossing_targets_meet_in_the_middle():
    frames = crossing_targets()
    by_id = [{t.id: t for t in boxes} for boxes in frames]
    gaps = [abs(f[1].u - f[2].u) for f in by_id if 1 in f and 2 in f]
    assert min(gaps) < 20  # close enough that a naive tracker can mix them up


def test_targets_from_saved_truth():
    truth = {'camera': {'width': 1280, 'height': 720, 'hfov_rad': 1.0},
             'drones': [{'id': 0, 'pixel': [640.0, 360.0], 'distance': 50.0},
                        {'id': 3, 'pixel': None, 'distance': 20.0}]}
    [box] = targets_from_truth(truth)
    assert (box.id, box.u, box.v) == (0, 640.0, 360.0)
    assert 10 < box.width < 15  # 0.5 m at 50 m with a ~1170 px focal length


def test_uneven_rate_skips_images_and_stamps_the_rest():
    from swarm_backend.detect.fake_detections import panning_camera, uneven_rate

    frames, axes = panning_camera(crossing_targets())
    kept, kept_axes, stamps = uneven_rate(frames, axes, drop=0.5, rate_hz=10.0, seed=1)
    assert 0.3 * len(frames) < len(kept) < 0.7 * len(frames)
    assert stamps[0] == 0.0 and stamps == sorted(stamps) and len(stamps) == len(kept_axes)
    i = round(stamps[-1] * 10)
    assert kept[-1] == frames[i] and kept_axes[-1] == axes[i]
