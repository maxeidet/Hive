import math

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from swarm_backend.detect.candidates import (  # noqa: E402
    CandidateConfig, above_line, draw, find_candidates)
from swarm_backend.detect.tracking import Confirmer, angle_between  # noqa: E402

W, H = 640, 360


def sky(seed=0, noise=2.0):
    """Bright blue sky, darker towards the top, with a little sensor noise."""
    rng = np.random.default_rng(seed)
    v = np.linspace(0.0, 1.0, H)[:, None, None]
    base = np.array([110, 160, 230]) + v * np.array([60, 50, 20])
    img = np.broadcast_to(base, (H, W, 3)) + rng.normal(0, noise, (H, W, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


def disc(img, u, v, diameter, color=(40, 40, 45)):
    cv2.circle(img, (u, v), max(1, diameter // 2), color, -1, cv2.LINE_AA)
    return img


def near(boxes, u, v, tol=2.0):
    return [b for b in boxes if math.hypot(b.u - u, b.v - v) <= tol]


# --- candidates -------------------------------------------------------------

def test_empty_sky_gives_nothing():
    assert find_candidates(sky()) == []


@pytest.mark.parametrize('diameter', [3, 5, 9, 13])
def test_finds_small_dark_discs(diameter):
    img = disc(sky(), 300, 120, diameter)
    boxes = find_candidates(img)
    assert len(boxes) == 1
    assert near(boxes, 300, 120, tol=1.0)
    assert max(boxes[0].width, boxes[0].height) <= diameter + 2


def test_finds_several_and_orders_by_contrast():
    img = sky()
    disc(img, 100, 80, 5, color=(85, 120, 170))  # faint
    disc(img, 400, 200, 5)  # dark
    disc(img, 550, 50, 3)
    boxes = find_candidates(img)
    assert len(boxes) == 3
    assert near(boxes[:1], 400, 200)
    assert boxes[-1].contrast < boxes[0].contrast


def test_ignores_large_soft_cloud_and_big_objects():
    img = sky()
    cloud = np.zeros((H, W), np.float32)
    cv2.circle(cloud, (320, 180), 80, 1.0, -1)
    cloud = cv2.GaussianBlur(cloud, (0, 0), 30)[:, :, None]
    img = np.clip(img - cloud * 60, 0, 255).astype(np.uint8)  # a darker grey cloud
    disc(img, 100, 300, 40)  # bigger than the kernel: not a far-away target
    assert find_candidates(img) == []


def test_ignores_thin_lines():
    img = sky()
    cv2.line(img, (100, 100), (114, 100), (40, 40, 40), 1)  # 15 x 1 px, like a wire
    assert find_candidates(img) == []


def test_mask_drops_everything_below_the_horizon():
    img = disc(disc(sky(), 300, 100, 5), 300, 300, 5)
    boxes = find_candidates(img, mask=above_line(img.shape, 200))
    assert len(boxes) == 1 and near(boxes, 300, 100)


def test_above_line_follows_a_rolled_horizon():
    mask = above_line((100, 200), 20, 80)
    assert mask[19, 0] and not mask[21, 0]
    assert mask[79, 199] and not mask[81, 199]


def test_threshold_is_raised_by_noise():
    noisy = disc(sky(noise=12.0), 300, 120, 5, color=(150, 170, 220))  # faint target
    assert near(find_candidates(noisy, config=CandidateConfig(min_contrast=5)), 300, 120) == []


def test_grey_input_and_draw():
    img = disc(sky(), 300, 120, 5)
    grey = img.min(axis=2)
    boxes = find_candidates(grey)
    assert near(boxes, 300, 120)
    out = draw(grey, boxes, labels=['1'])
    assert out.shape == (H, W, 3)


# --- confirmation -----------------------------------------------------------

def test_confirmed_after_k_images_in_a_row():
    confirmer = Confirmer(k=3, gate=10.0)
    for i in range(2):
        confirmer.update([(100 + 2 * i, 50)])
        assert confirmer.confirmed == []
    confirmer.update([(104, 50)])
    assert [t.id for t in confirmer.confirmed] == [1]


def test_miss_resets_the_streak_and_long_misses_drop_the_track():
    confirmer = Confirmer(k=2, gate=10.0, max_misses=2)
    confirmer.update([(0, 0)])
    confirmer.update([(1, 0)])
    assert confirmer.confirmed
    confirmer.update([])
    assert not confirmer.confirmed and len(confirmer.tracks) == 1
    confirmer.update([(2, 0)])
    assert not confirmer.confirmed  # streak restarted at 1
    confirmer.update([(3, 0)])
    assert confirmer.confirmed and confirmer.confirmed[0].id == 1
    for _ in range(3):
        confirmer.update([])
    assert confirmer.tracks == []


def test_two_targets_keep_their_ids_and_far_jumps_start_new_tracks():
    confirmer = Confirmer(k=1, gate=10.0)
    confirmer.update([(0, 0), (100, 0)])
    tracks = confirmer.update([(103, 0), (2, 0)])
    assert {t.id: t.position for t in tracks} == {1: (2, 0), 2: (103, 0)}
    tracks = confirmer.update([(50, 0)])
    assert [t.id for t in tracks if t.streak] == [3]


def test_angle_gate_on_world_directions():
    confirmer = Confirmer(k=2, gate=math.radians(1.0), distance=angle_between)
    a = (1.0, 0.0, 0.0)
    b = (math.cos(math.radians(0.5)), math.sin(math.radians(0.5)), 0.0)
    c = (math.cos(math.radians(5)), math.sin(math.radians(5)), 0.0)
    confirmer.update([a])
    confirmer.update([b])
    assert confirmer.confirmed
    confirmer.update([c])
    assert not confirmer.confirmed
