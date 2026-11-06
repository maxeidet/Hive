import math

from swarm_backend.detect.bytetrack import Settings
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.fake_detections import Errors, TruthBox, run as fake_run
from swarm_backend.detect.track_eval import (
    ByteTrackRunner, ConfirmerRunner, main, run, scenario, score)


def two_targets(images: int = 10):
    """Target 1 at u = 100, target 2 at u = 300, both still."""
    return [[TruthBox(1, 100.0, 100.0, 8, 8), TruthBox(2, 300.0, 100.0, 8, 8)]
            for _ in range(images)]


def test_a_perfect_tracker_scores_perfectly():
    truth = two_targets()
    outputs = [[(7, 100.0, 100.0), (9, 300.0, 100.0)] for _ in truth]
    m = score(truth, outputs)
    assert (m.mota, m.idf1, m.coverage) == (1.0, 1.0, 1.0)
    assert (m.id_switches, m.fragments, m.false, m.false_tracks) == (0, 0, 0, 0)
    assert m.to_confirm == [0, 0]


def test_swapped_ids_count_as_two_switches_and_lower_idf1():
    truth = two_targets()
    outputs = [[(1, 100.0, 100.0), (2, 300.0, 100.0)] if i < 5
               else [(2, 100.0, 100.0), (1, 300.0, 100.0)] for i in range(10)]
    m = score(truth, outputs)
    assert m.id_switches == 2
    assert m.coverage == 1.0 and math.isclose(m.idf1, 0.5)


def test_a_gap_is_a_miss_and_a_fragment_and_confirmation_takes_time():
    truth = two_targets()[:6]
    outputs = [[], [], [(1, 100.0, 100.0)], [], [(1, 101.0, 100.0)], [(1, 100.0, 99.0)]]
    m = score(truth, outputs)
    assert m.fragments == 1
    assert m.misses == 12 - 3
    assert m.to_confirm == [2] and m.never_confirmed == 1  # target 2 never had a track


def test_a_track_on_nothing_is_false():
    truth = two_targets(3)
    outputs = [[(1, 100.0, 100.0), (2, 300.0, 100.0), (5, 600.0, 400.0)] for _ in truth]
    m = score(truth, outputs)
    assert m.false == 3 and m.false_tracks == 1
    assert math.isclose(m.mota, 1 - 3 / 6)


def test_bytetrack_beats_the_confirmer_on_the_swinging_gimbal():
    camera = CameraModel()
    frames, axes = scenario('panning', camera)
    byte, conf = None, None
    for seed in range(10):
        images = fake_run(frames, camera.width, camera.height, Errors(), seed=seed)
        b = run(lambda: ByteTrackRunner(Settings(), camera), images, axes, 10.0)
        c = run(lambda: ConfirmerRunner(3, 20.0, 10.0, camera), images, axes, 10.0)
        byte, conf = (b if byte is None else byte + b), (c if conf is None else conf + c)
    assert byte.id_switches == 0
    assert byte.mota > conf.mota and byte.idf1 > conf.idf1


def test_the_command_line_runs(capsys):
    assert main(['--seeds', '2', '--sweep', 'recover_low', 'false', 'true']) == 0
    out = capsys.readouterr().out
    assert 'Confirmer' in out and 'recover_low=true' in out
    assert main(['--seeds', '2', '--drop', '0.5']) == 0
    assert 'without time stamps' in capsys.readouterr().out
