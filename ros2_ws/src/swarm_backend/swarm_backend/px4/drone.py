class Drone:

    def __init__(self, drone_id):
        self.id = drone_id

        self.position = None
        self.velocity = None
        self.attitude = None

    def arm(self):
        ...

    def takeoff(self, altitude):
        ...

    def land(self):
        ...

    def set_position(self, x, y, z):
        ...

    def set_velocity(self, vx, vy, vz):
        ...