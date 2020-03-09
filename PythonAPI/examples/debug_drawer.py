from agents.tools.misc import distance_vehicle, draw_waypoints

class Drawer:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle

    def draw_nearby_boudings(actors, max_distance = 200):
        if len(self.vehicles) > 1:
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in self.vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > max_distance:
                    break
                pass 
        pass 

    def draw_lidar():
        pass 

    def draw_plan(local_planner):
        draw_waypoints(self.world.world, [local_planner.target_waypoint], vehicle.get_location().z + 1.0)
        pass

