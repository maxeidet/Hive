class SwarmNode(Node):

    def __init__(self):
        super().__init__("swarm")

        configs = load_config()

        drones = [
            PX4Drone(self, config)
            for config in configs
        ]

        self.swarm = Swarm(drones)
        self.manager = SwarmManager(self.swarm)
        self.mission = Mission(self.manager)