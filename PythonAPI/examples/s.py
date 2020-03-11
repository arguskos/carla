#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue

from controller import KeyboardControl
from vehicle_controller import VehicleController
from carla import ColorConverter as cc



class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction, display, renderer):
        self.renderer = renderer
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.width = 800
        self.height = 600
        self.recording = False
        self.display = display
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 1
        self.camera_sensor = ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}]
        
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        self.camera_bp = bp_library.find(self.camera_sensor[0])
        self.camera_bp.set_attribute('image_size_x', str(self.width))
        self.camera_bp.set_attribute('image_size_y', str(self.height))
        if self.camera_bp.has_attribute('gamma'):
            self.camera_bp.set_attribute('gamma', str(gamma_correction))
        for attr_name, attr_value in self.camera_sensor[3].items():
            self.camera_bp.set_attribute(attr_name, attr_value)

        self.index = None

    def set_sensor(self, index, notify=True, force_respawn=False):
        if self.sensor is not None:
            self.sensor.destroy()
            self.surface = None
            return 
        self.sensor = self._parent.get_world().spawn_actor(
                self.camera_bp,
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
        self.weak_ref = weakref.ref(self)
        self.sensor.listen(lambda image: Renderer._parse_static_image(self.weak_ref, image))

class Renderer():
    def __init__(self):
        self.surface = None
        self.recording = False

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    # @staticmethod
    def _parse_image(self, image, image_converter=cc.Raw):
        # self = weak_self()
        # if not self:
        #     return
        image.convert(image_converter)
        # self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        # draw_image(self.display, image)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)
display = None
def simple_parse(image, image_converter=cc.Raw):
    global display
    image.convert(image_converter)
    # self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    # draw_image(self.display, image)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    pass 

class SimpleCamera():
    def __init__(self, world, vehicle):
        self.world = world
        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        # blueprint.set_attribute('image_size_x', '1920')
        # blueprint.set_attribute('image_size_y', '1080')
        # blueprint.set_attribute('fov', '110')
        # transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        Attachment = carla.AttachmentType
        pos_desc = (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm)
        self.sensor = world.spawn_actor(blueprint, pos_desc[0], attach_to=vehicle, attachment_type=pos_desc[-1])

    def setup_listener(self, parse_function):
        self.sensor.listen(lambda data: parse_function(data))

    def destroy(self):
        if self.sensor:
            self.sensor.destroy()

class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        # self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self.sync = False
        self._settings = None
        self._server_clock = pygame.time.Clock()
        self.display = kwargs['display']
        global display
        display = self.display
        self.renderer = Renderer()
        self.vehicle = kwargs.get('vehicle')
        self.world.on_tick(self.on_world_tick)
        # self.weak_renderer = weakref.ref(self.renderer)
        # self.camera_manager = CameraManager(kwargs.get('vehicle'), None, 2.2, kwargs['display'], self.renderer)
        # self.camera_manager.transform_index = 0
        # self.camera_manager.set_sensor(0, notify=False)
        self.camera = SimpleCamera(self.world, self.vehicle)

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.timestamp = timestamp
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds
        self.dt = self.timestamp.delta_seconds

    def __enter__(self):
        self._settings = self.world.get_settings()
        if self.sync:
            self.frame = self.world.apply_settings(carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=self.delta_seconds))
            
            def make_queue(register_event):
                q = queue.Queue()
                register_event(q.put)
                self._queues.append(q)

            make_queue(self.world.on_tick)
            make_queue(self.camera.sensor.listen)
        else: 
            self.frame = self.world.apply_settings(carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=False,
                fixed_delta_seconds=0))
            self.camera.setup_listener(simple_parse)
        return self

    def tick(self, timeout):
        if self.sync:
            self.frame = self.world.tick()
            data = [self._retrieve_data(q, timeout) for q in self._queues]
            assert all(x.frame == self.frame for x in data)
            image = data[1]
            simple_parse(image)
            # self.renderer._parse_image(image)
            return data

    def render(self):
        self.renderer.render(self.display)

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)
        self.camera.destroy()

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame or not self.sync:
                return data


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def main():
    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    controller = KeyboardControl()

    try:
        m = world.get_map()
        start_pose = m.get_spawn_points()[40]
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.toyota.prius')),
            start_pose)
        # vehicle.set_autopilot(True)
        
        vehicle_controller = VehicleController(vehicle, controller)
        
 
      
        actor_list.append(vehicle)

        with CarlaSyncMode(world, fps=30, vehicle=vehicle, display=display) as sync_mode:
            while True:
                if should_quit():
                    return
                controller.parse_events()
                #clock.tick() # basicly same shit but less acurate 
                clock.tick()
                # Advance the simulation and wait for the data.
                sync_mode.tick(timeout=2)
                # snapshot = snapshot[0]
                # dt = snapshot.timestamp.delta_seconds
                vehicle_controller.tick(sync_mode.dt)
                print(sync_mode.dt)
                # sync_mode.render()
                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / sync_mode.dt)
                # Draw the display.
                # draw_image(display, image_rgb)
                # draw_image(display, image_semseg, blend=True)
                display.blit(
                    font.render('% 5d FPS (real - client)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('{}, {} FPS (simulated - server)'.format(fps, int(sync_mode.server_fps)), True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
