import sys
import glob
import os

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

class VehicleController:
    def __init__(self, vehicle, controller):
        self._vehicle = vehicle
        self._controller = controller
        self._control = carla.VehicleControl()
        self._steer_cache = 0
        self._rotation_speed = 0.5

    def set_vehicle(self, vehicle):
        _vehicle = vehicle

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
        self._vehicle.apply_control(self._control)
    
            
