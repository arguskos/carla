class WorldState:
    def __init__(self, world):
        self.vehicles = None
        self.world = world
        pass

    def tick():
        self.vehicles = self.world.get_actors().filter('vehicle.*')

