from swarm_backend.demo.poselog import keep_names


def test_keep_names():
    keep = keep_names(['x500', 'bird_', 'rec_cam_'], ['standard_vtol_5'])
    for name in ('x500_4', 'x500_gimbal_2', 'bird_3', 'rec_cam_arc_side', 'standard_vtol_5'):
        assert keep(name), name
    for name in ('base_link', 'ground_plane', 'camera_link', 'standard_vtol_50'):
        assert not keep(name), name
