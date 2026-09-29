class PX4Topics:
    def __init__(self, namespace):
        self.namespace = namespace

    @property
    def vehicle_command(self):
        return f"{self.namespace}/fmu/in/vehicle_command"

    @property
    def trajectory_setpoint(self):
        return f"{self.namespace}/fmu/in/trajectory_setpoint"

    @property
    def vehicle_local_position(self):
        return f"{self.namespace}/fmu/out/vehicle_local_position"