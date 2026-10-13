import math

from swarm_backend.detect.tracking import Confirmer, angle_between

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
