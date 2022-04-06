import carla
import math
from enum import Enum
from shapely.geometry import Polygon

class BasicAgent(object):

    def __init__(self, vehicle, target_speed=20, opt_dict={}):
        self._vehicle = vehicle
        self._vehicle.set_autopilot(True)
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        # Base parameters
        self._target_speed = target_speed
        self._max_brake = 0.5

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'max_brake' in opt_dict:
            self._max_steering = opt_dict['max_brake']

    def add_emergency_stop(self, control):
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        control = carla.VehicleControl()
        v = self._vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        if speed < kmh:
            h = 1
        elif speed > kmh:
            h = 0

        while speed != kmh:
            v = self._vehicle.get_velocity()
            kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))

            if kmh < speed:
                h += .1
                control.throttle = h
            else:
                h -= .1
                control.throttle = h
        
        return control

    def run_step(self, action):

        if action == 0:
            self.set_target_speed(self, 10)
        elif action == 1:
            self.set_target_speed(self, 20)
        elif action == 2:
            self.set_target_speed(self, 30)
        elif action == 3:
            self.set_target_speed(self, 40)
        elif action == 4:
            self.set_target_speed(self, 50)
        elif action == 5:
            self.set_target_speed(self, 60)

        return 

