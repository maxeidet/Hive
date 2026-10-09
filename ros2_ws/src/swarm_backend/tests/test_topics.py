from swarm_backend.px4.topics import PX4Topics


def test_instance_zero_has_no_prefix():
    topics = PX4Topics('')
    assert topics.vehicle_command == '/fmu/in/vehicle_command'
    assert topics.vehicle_status == '/fmu/out/vehicle_status_v4'


def test_namespaced_instance():
    topics = PX4Topics('px4_2')
    assert topics.trajectory_setpoint == '/px4_2/fmu/in/trajectory_setpoint'
    assert topics.vehicle_odometry == '/px4_2/fmu/out/vehicle_odometry'
    assert PX4Topics('/px4_2/').offboard_control_mode == '/px4_2/fmu/in/offboard_control_mode'
