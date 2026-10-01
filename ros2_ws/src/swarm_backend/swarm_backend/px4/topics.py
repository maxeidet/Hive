"""The one place PX4 topic names are built.

PX4 instance 0 publishes on /fmu/..., instance N on /px4_N/fmu/... (PX4's rcS sets
the uXRCE-DDS namespace). Some output topics carry a message-version suffix; the
names below were confirmed with `ros2 topic list` against our PX4 image.
"""


class PX4Topics:

    def __init__(self, namespace: str = ''):
        namespace = namespace.strip('/')
        self.namespace = namespace
        self._prefix = f'/{namespace}' if namespace else ''

    # --- to PX4 ------------------------------------------------------------

    @property
    def offboard_control_mode(self) -> str:
        return self._in('offboard_control_mode')

    @property
    def trajectory_setpoint(self) -> str:
        return self._in('trajectory_setpoint')

    @property
    def vehicle_command(self) -> str:
        return self._in('vehicle_command')

    # --- from PX4 ----------------------------------------------------------

    @property
    def vehicle_odometry(self) -> str:
        return self._out('vehicle_odometry')

    @property
    def vehicle_status(self) -> str:
        return self._out('vehicle_status_v4')

    @property
    def vehicle_command_ack(self) -> str:
        return self._out('vehicle_command_ack_v1')

    @property
    def gimbal_attitude(self) -> str:
        return self._out('gimbal_device_attitude_status')

    def _in(self, name: str) -> str:
        return f'{self._prefix}/fmu/in/{name}'

    def _out(self, name: str) -> str:
        return f'{self._prefix}/fmu/out/{name}'
