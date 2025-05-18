"""
  Robotem Rovne 2024
"""
import datetime
import math

import numpy as np

from osgar.lib.route import Convertor
from osgar.node import Node
from osgar.followme import EmergencyStopException
from osgar.lib import quaternion


def mask_center(mask):
    if mask.max() == 0:
        return mask.shape[0]//2, mask.shape[1]//2
    assert mask.max() == 1, mask.max()
    indices = np.argwhere(mask == 1)  # shape (num_points, 2)

    # Compute the center of mass as the mean of these indices
    return tuple(int(x) for x in indices.mean(axis=0))


class RobotemRovne(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.stop_dist = config.get('stop_dist', 1.0)
        self.limit_dist = config.get('dist_limit', None)
        self.verbose = False
        self.last_position = None  # not defined, probably should be 0, 0, 0
        self.last_obstacle = 0
        self.last_nn_mask = None
        self.last_dir = 0  # straight
        self.start_lon_lat = None
        self.raise_exception_on_stop = config.get('terminate_on_stop', False)

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        if self.last_obstacle < self.stop_dist or self.last_nn_mask is None:  # meters
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, self.last_dir
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

    def on_nmea_data(self, data):
        if ('lat' in data and 'lon' in data) and (data['lat'] is not None and data['lon'] is not None):
            lat, lon = data['lat'], data['lon']
            if self.start_lon_lat is None:
                self.start_lon_lat = (lon, lat)
            conv = Convertor(refPoint=self.start_lon_lat)
            dist = math.hypot(*conv.geo2planar((lon, lat)))
            if self.limit_dist is not None and dist > self.limit_dist:
                print('reached', dist)
                raise EmergencyStopException()

    def on_nn_mask(self, data):
        self.last_nn_mask = data.copy()  # make sure you modify only own copy
        assert self.last_nn_mask.shape == (120, 160), self.last_nn_mask.shape
        self.last_nn_mask[:60, :] = 0  # remove sky detections

        center_y, center_x = mask_center(self.last_nn_mask)
        dead = 10
        turn_angle = math.radians(20)
        if center_x > 80 + dead:
            self.last_dir = -turn_angle
        elif center_x < 80 - dead:
            self.last_dir = turn_angle
        else:
            self.last_dir = 0  # straight

    def on_orientation_list(self, data):
        if self.verbose:
            for quat in data:
                print(self.last_position, quaternion.heading(quat[2:]))

# vim: expandtab sw=4 ts=4
