from __future__ import print_function

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import ColorConverter as cc

import argparse
import logging
import math
import random
import re
import weakref
import time

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent # pylint: disable=import-error
from agents.navigation.crest import randNum # pylint: disable=import-error

IM_WIDTH = 2560
IM_HEIGHT = 1440
actor_list = []
agents_list = []
all_id = []
vehicles = []

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

def get_speed_km(v):
    kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
    return kmh

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_world, args, client):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.camera_manager = None
        self.client = client
        self._gamma = args.gamma
        self.restart()

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # get tesla vehicle from blueprint
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter('model3')[0]

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            actor_list.append(self.player)
            vehicles.append(self.player)
            agent = BasicAgent(self.player)
            agents_list.append(agent)

            #destination = random.choice(spawn_points).location
            #agent.set_destination(destination)
            #self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
            #self.player.set_autopilot(True)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            actor_list.append(self.player)
            vehicles.append(self.player)
            agent = BasicAgent(vehicle=self.player, target_speed=20)
            agents_list.append(agent)
            #destination = random.choice(spawn_points).location
            #agent.set_destination(destination)
            #self.show_vehicle_telemetry = False
            
            # self.modify_vehicle_physics(self.player)
            #self.player.set_autopilot(True)

        # Set up the sensors.
        self.camera_manager = CameraManager(self.player, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def render(self, display):
        self.camera_manager.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.spawn_point = carla.Transform(carla.Location(x=.20, y=0, z=1.10))

        self.transform_index = 1

        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
        ]

        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            bp.set_attribute('fov', '110')
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(IM_WIDTH))
                bp.set_attribute('image_size_y', str(IM_HEIGHT))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, force_respawn=True)

    def set_sensor(self, index, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self.spawn_point,
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        
        self.index = index

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world):#, start_in_autopilot):
        #self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            #world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def parse_events(self, world):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    # if self._autopilot_enabled:
                    #     world.restart()
                    #     world.player.set_autopilot(False)
                    #if:
                    world.restart()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        world = World(sim_world, args, client)

        controller = KeyboardControl(world)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()

        oldTime = time.time()

        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(world):
                return

            actor_list[0].apply_control(agents_list[0].run_step())

            # 1
            if time.time() - oldTime >= (10) and time.time() - oldTime < (20):
                print("HERE1")
                print("speed before", get_speed_km(v=vehicles[0].get_velocity()))
                # vehicles[0].apply_control(agents_list[0].run_step(action=0))
                agents_list[0].set_target_speed(action=randNum.returnRand())

            if time.time() - oldTime >= (20) and time.time() - oldTime < (30):
                print("HERE2")
                print("speed before", get_speed_km(v=vehicles[0].get_velocity()))
                # vehicles[0].apply_control(agents_list[0].run_step(action=0))
                agents_list[0].set_target_speed(action=randNum.returnRand())

            if time.time() - oldTime >= (30) and time.time() - oldTime < (40):
                print("HERE3")
                print("speed before", get_speed_km(v=vehicles[0].get_velocity()))
                # vehicles[0].apply_control(agents_list[0].run_step(action=0))
                agents_list[0].set_target_speed(action=randNum.returnRand())
                
                
                
            # # 2
            # if time.time() - oldTime >= (20) and time.time() - oldTime < (30):
            #     vehicles[0].apply_control(agents_list[0].run_step(4))
                
            print("speed after", get_speed_km(v=vehicles[0].get_velocity()))
            world.render(display)
            pygame.display.flip()
   
    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if world is not None:
            world.destroy()

        pygame.quit()

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_false',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='2560x1440',
        help='window resolution (default: 2560x1440)')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    print(args)

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user.')

    finally:
        for actor in actor_list:
            actor.destroy()

if __name__ == '__main__':

    main()
