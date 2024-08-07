"""
  Robotem Rovne 2024
"""
import datetime
import math

import numpy as np

from osgar.node import Node
from osgar.followme import EmergencyStopException
from osgar.lib import quaternion


class RobotemRovne(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.stop_dist = config.get('stop_dist', 1.0)
        self.verbose = False
        self.last_position = None  # not defined, probably should be 0, 0, 0
        self.last_obstacle = 0
        self.raise_exception_on_stop = False

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        if self.last_obstacle < self.stop_dist:  # meters
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, 0
        if self.verbose:
            print(speed, steering_angle)
        self.send_speed_cmd(speed, steering_angle)

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed*1000), round(math.degrees(steering_angle)*100)]
        )

    def on_obstacle(self, data):
        self.last_obstacle = data

    def on_detections(self, data):
        pass

    def on_depth(self, data):
        pass

    def on_orientation_list(self, data):
        if self.verbose:
            for quat in data:
                print(self.last_position, quaternion.heading(quat[2:]))

# vim: expandtab sw=4 ts=4
