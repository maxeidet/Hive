class Swarm:

    def __init__(self, drones):
        self.drones = drones

    def arm_all(self):
        for drone in self.drones:
            drone.arm()

    def land_all(self):
        for drone in self.drones:
            drone.land()