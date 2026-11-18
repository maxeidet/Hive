import json
import math

import numpy as np
import pytest

from swarm_backend.octomap_snapshot import (
    KEY_OFFSET, TREE_DEPTH, colourize, fill_gaps, read_bt, top_view, write_snapshot)

HEADER = (b'# Octomap OcTree binary file\nid OcTree\nsize 3\nres {res}\ndata\n')


def node_bytes(codes: dict[int, int]) -> bytes:
    """One node's two bytes: codes maps child index -> 1 free, 2 occupied, 3 inner."""
    bits = sum(code << (2 * i) for i, code in codes.items())
    return bytes([bits & 0xFF, bits >> 8])


def bt(res: float, *nodes: bytes) -> bytes:
    return HEADER.replace(b'{res}', str(res).encode()) + b''.join(nodes)


def test_root_children():
    # Child 7 (upper half in x, y and z) occupied, child 0 (lower in all) free.
    leaves = read_bt(bt(0.001, node_bytes({7: 2, 0: 1})))
    half = (1 << (TREE_DEPTH - 1)) * 0.001
    assert leaves.resolution == 0.001
    np.testing.assert_allclose(leaves.occupied_lo, [[0, 0, 0]])
    np.testing.assert_allclose(leaves.occupied_size, [half])
    np.testing.assert_allclose(leaves.free_lo, [[-half, -half, -half]])


def test_inner_nodes_depth_first():
    # Root: child 1 (upper x) is inner, child 2 (upper y) is inner. Each holds one leaf.
    leaves = read_bt(bt(0.1, node_bytes({1: 3, 2: 3}),
                        node_bytes({0: 2}),       # inside child 1: its lower corner, occupied
                        node_bytes({4: 1})))      # inside child 2: upper z, free
    quarter = (1 << (TREE_DEPTH - 2)) * 0.1
    np.testing.assert_allclose(leaves.occupied_lo, [[0, -2 * quarter, -2 * quarter]])
    np.testing.assert_allclose(leaves.occupied_size, [quarter])
    np.testing.assert_allclose(leaves.free_lo, [[-2 * quarter, 0, -quarter]])


def test_not_an_octree():
    with pytest.raises(ValueError):
        read_bt(b'# Octomap\nid ColorOcTree\nres 0.1\ndata\n\x00\x00')


def voxel_leaves(occupied, free, res=1.0):
    """Leaves of single voxels given as integer ENU cells."""
    from swarm_backend.octomap_snapshot import Leaves
    to = lambda cells: (np.array(cells, float).reshape(-1, 3) * res, np.full(len(cells), res))
    return Leaves(res, *to(occupied), *to(free))


def test_top_view_north_up_and_highest_voxel():
    # A 2 m column at east 2, north 0 and one ground voxel at east 0, north 1.
    leaves = voxel_leaves(occupied=[(2, 0, 0), (2, 0, 1), (0, 1, -1)], free=[(1, 0, 0)])
    view = top_view(leaves)
    assert view.extent == {'north_min': 0.0, 'north_max': 2.0, 'east_min': 0.0, 'east_max': 3.0}
    assert view.top[1, 2] == 2.0  # bottom row is north 0..1; top of the column
    assert view.top[0, 0] == 0.0  # top row is north 1..2
    assert view.seen[1, 1]


def test_big_leaf_and_bounds():
    from swarm_backend.octomap_snapshot import Leaves
    leaves = Leaves(1.0, np.array([[-4.0, -4.0, 0.0]]), np.array([8.0]),
                    np.zeros((0, 3)), np.zeros(0))
    view = top_view(leaves, bounds=(0, 2, 0, 2))
    assert view.top.shape == (2, 2) and (view.top == 4.0 + 4.0).all()  # top of a box from z 0, 8 m high


def test_fill_gaps_fills_stripes_not_open_space():
    top = np.full((5, 5), np.nan)
    top[:, [0, 2, 4]] = 0.0  # stripes with one-cell gaps
    filled = fill_gaps(top, passes=1)
    assert not np.isnan(filled[1:-1, 1]).any()
    lone = np.full((5, 5), np.nan)
    lone[2, 2] = 1.0
    assert np.isnan(fill_gaps(lone)[2, 3])


def test_colourize_alpha():
    leaves = voxel_leaves(occupied=[(0, 0, 0)], free=[(1, 0, 0)])
    image = colourize(top_view(leaves, cell=1.0), ground_z=0.0)
    assert image.shape == (1, 2, 4)
    assert image[0, 0, 3] == 255 and 0 < image[0, 1, 3] < 255


def test_write_snapshot(tmp_path):
    data = bt(0.1, node_bytes({7: 2, 0: 1}))
    info = write_snapshot(data, tmp_path, 0.0, cell=100.0, bounds=None, world='hive_base')
    saved = json.loads((tmp_path / 'map.json').read_text())
    assert saved == info and saved['world'] == 'hive_base'
    assert saved['north_max'] > saved['north_min'] and saved['east_max'] > saved['east_min']
    assert (tmp_path / 'map.png').read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'
    assert not list(tmp_path.glob('*.tmp'))
