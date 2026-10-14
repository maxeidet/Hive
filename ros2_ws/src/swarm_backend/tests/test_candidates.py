import cv2
import numpy as np
import pytest

from swarm_backend.detect.candidates import above_line, draw, find_candidates


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


@pytest.mark.parametrize('diameter', [3, 5, 9, 13, 30])
def test_box_is_about_the_size_of_the_object(diameter):
    image = sky()
    cv2.circle(image, (160, 100), diameter // 2, (20, 20, 25), -1)
    found = find_candidates(image)
    assert len(found) == 1
    assert abs(max(found[0].width, found[0].height) - diameter) <= 3


def test_finds_several_and_orders_by_contrast():
    image = sky()
    cv2.circle(image, (60, 50), 2, (90, 95, 100), -1)  # fainter
    cv2.circle(image, (200, 80), 2, (20, 20, 25), -1)  # dark
    found = find_candidates(image)
    assert len(found) == 2
    assert abs(found[0].u - 200) <= 2 and abs(found[0].v - 80) <= 2
    assert found[0].contrast > found[1].contrast


def test_object_cut_off_by_the_image_edge_is_not_a_candidate():
    image = sky()
    cv2.circle(image, (160, 0), 4, (20, 20, 25), -1)  # like our own propeller tip
    assert find_candidates(image) == []


def test_mask_drops_what_is_outside_it():
    image = sky()
    cv2.circle(image, (100, 40), 2, (20, 20, 25), -1)
    cv2.circle(image, (200, 90), 2, (20, 20, 25), -1)
    found = find_candidates(image, mask=above_line(image.shape, 60))
    assert len(found) == 1 and abs(found[0].u - 100) <= 2


def test_above_line_follows_a_rolled_horizon():
    mask = above_line((100, 200), 20, 80)
    assert mask[19, 0] and not mask[21, 0]
    assert mask[79, 199] and not mask[81, 199]


def test_grey_input_and_draw():
    image = sky()
    cv2.circle(image, (160, 100), 2, (20, 20, 25), -1)
    grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    found = find_candidates(grey)
    assert len(found) == 1
    assert draw(grey, found, labels=['1']).shape == (240, 320, 3)
