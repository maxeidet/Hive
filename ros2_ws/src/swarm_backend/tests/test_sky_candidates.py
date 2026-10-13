import cv2
import numpy as np

from swarm_backend.detect.sky_candidates import find_candidates


def sky(width=320, height=240):
    """Blue sky that gets brighter toward the horizon, with a little noise."""
    rng = np.random.default_rng(0)
    rows = np.linspace(0, 1, height)[:, None, None]
    image = np.array([110, 160, 220]) + rows * np.array([110, 70, 20])
    image = image + rng.normal(0, 2, (height, width, 3))
    return np.clip(image, 0, 255).astype(np.uint8)


def test_finds_small_dark_objects_of_each_size():
    for size in (3, 5, 20):
        image = sky()
        cv2.circle(image, (160, 100), size // 2, (20, 20, 25), -1)
        found = find_candidates(image)
        assert len(found) == 1, size
        cx, cy = found[0].centre
        assert abs(cx - 160) <= 2 and abs(cy - 100) <= 2


def test_empty_sky_gives_nothing():
    assert find_candidates(sky()) == []


def test_blue_gap_between_white_clouds_is_not_a_candidate():
    image = np.full((240, 320, 3), 235, np.uint8)  # white cloud
    cv2.circle(image, (160, 100), 4, (120, 165, 215), -1)  # a gap of blue sky
    assert find_candidates(image) == []


def test_large_dark_object_is_not_a_candidate():
    image = sky()
    cv2.rectangle(image, (100, 0), (180, 90), (30, 25, 35), -1)  # our own arm in view
    assert find_candidates(image) == []


def test_dark_object_against_the_ground_is_not_a_candidate():
    image = sky()
    image[120:] = (85, 100, 45)  # grass
    cv2.circle(image, (160, 180), 3, (20, 20, 25), -1)
    assert find_candidates(image) == []
