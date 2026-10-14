import argparse

import cv2
import numpy as np
import pytest

from swarm_backend.detect import detectors


def options(*argv):
    parser = argparse.ArgumentParser()
    detectors.add_options(parser)
    return parser.parse_args(list(argv))


def sky_with_drone():
    image = np.zeros((240, 320, 3), np.uint8)
    image[:] = (150, 190, 235)
    cv2.circle(image, (160, 100), 3, (20, 20, 25), -1)
    return image


def test_baseline_returns_detections():
    found = detectors.make('baseline', options())(sky_with_drone(), None)
    assert len(found) == 1
    d = found[0]
    assert isinstance(d, detectors.Detection)
    assert abs(d.u - 160) <= 2 and abs(d.v - 100) <= 2
    assert d.label == 'drone' and d.score > 0 and d.size >= 5


def test_baseline_takes_its_options():
    assert detectors.make('baseline', options('--contrast', '250'))(sky_with_drone(), None) == []


def test_yolo_is_not_there_yet():
    with pytest.raises(NotImplementedError):
        detectors.make('yolo', options())


def test_every_detector_has_a_name():
    assert set(detectors.DETECTORS) == {'baseline', 'yolo'}


def test_draw_keeps_the_size_and_makes_grey_colour():
    found = detectors.make('baseline', options())(sky_with_drone(), None)
    grey = cv2.cvtColor(sky_with_drone(), cv2.COLOR_RGB2GRAY)
    assert detectors.draw(grey, found, labels=['1']).shape == (240, 320, 3)
