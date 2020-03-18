import pygame
import carla
import weakref
import random
import math

class PlayerVehicle:
    
    def __init__(self, controller, world, tm, vehicle=None, start_pos=None):
        self._controller = controller
        if start_pos == None:
            m = world.get_map(
                
            )
            start_pos = m.get_spawn_points()[40]

        if vehicle == None:
            blueprint_library = world.get_blueprint_library()

            vehicle = world.spawn_actor(
                random.choice(blueprint_library.filter('vehicle.toyota.prius')),
                start_pos
            )
        self.player = vehicle
        self.transform = self.player.get_transform()
        self._control = carla.VehicleControl()
        self._steer_cache = 0
        self._rotation_speed = 0.5
        self._tm = tm
        self._draw_traj_state = False
        self._autopilot = False
        self._controller.add_action('g', self.draw_traj)
        self._controller.add_action('p', self.toggle_autopilot)


    def tick(self, dt):
        self._control.throttle = 1.0 if self._controller.forward else 0.0
        steer_increment = self._rotation_speed * dt
        if self._controller.left:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif self._controller.right:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        # print(self._steer_cache)
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if self._controller.back else 0.0
        # self._control.hand_brake = self._controller.hand_brake
        self.player.apply_control(self._control)

    def toggle_autopilot(self):
        self._autopilot = not self._autopilot 
        self.player.set_autopilot(self._autopilot)

    def draw_traj(self):
        self._draw_traj_state = not self._draw_traj_state
        self._tm.draw_trajectory(self.player, self._draw_traj_state)

    def clear():
        self.player.destroy()