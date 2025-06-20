"""
  RoboOrienteering - navigate in terrain on GPS points with cones
"""

import argparse
import math
import threading
from datetime import timedelta

import numpy as np

from osgar.lib.config import config_load
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.record import record
from osgar.node import Node

from osgar.drivers.gps import INVALID_COORDINATES


def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale


def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


def latlon2xy(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))



class RoboOrienteering(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'scan')
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.max_dist = config.get('max_dist', 3.0)
        self.goals = [latlon2xy(lat, lon) for lat, lon in config['waypoints']]
        self.last_position = None  # (lon, lat) in milliseconds
        self.verbose = False
        self.scan = None

        self.last_detections = None
        self.last_cones_distances = None  # not available

        """
        self.last_imu_yaw = None  # magnetic north in degrees
        self.status = None
        self.wheel_heading = None
        self.cmd = [0, 0]
        self.monitors = []
        self.last_position_angle = None  # for angle computation from dGPS
        """

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        pass

    def get_direction(self, arr):
        # based on FRE2025 code
        center = len(arr) // 2
        direction = 0  # default, if you cannot decide, go straight
        if arr[center] > 1000:
            # no close obstalce -> go straight
            direction = 0
            left = 0
            for i in range(0, center):
                if arr[center - i] > 1000:
                    left = i
                else:
                    break
            right = 0
            for i in range(0, center):
                if arr[center + i] > 1000:
                    right = i
                else:
                    break
            if self.verbose:
                print(self.time, left, right)
            if left <= 2 or right <= 2:
                if left >= 5:
                    # right is too close
                    direction = self.turn_angle // 2
                if right >= 5:
                    ## left is too close
                    direction = -self.turn_angle // 2
        else:
            # cannot go straight
            for i in range(1, center):
                if arr[center - i] > 1000 and arr[center + i] <= 1000:
                    # free space on the left
                    direction = self.turn_angle
                    break
                elif arr[center - i] <= 1000 and arr[center + i] > 1000:
                    # free space on the right
                    direction = -self.turn_angle
                    break
        return direction

    def on_pose2d(self, data):
        if math.hypot(data[0]/1000.0, data[1]/1000.0) >= self.max_dist:
            speed, steering_angle = 0, 0
        elif self.scan is None:
            # no depth data yet
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, self.get_direction(self.scan)
        if self.verbose:
            print(speed, steering_angle)
        self.send_speed_cmd(speed, steering_angle)

    def on_nmea_data(self, data):
        assert 'lat' in data, data
        assert 'lon' in data, data
        lat, lon = data['lat'], data['lon']
        if lat is not None and lon is not None:
            print(data)

    def on_detections(self, data):
        self.last_detections = data[:]

    def on_depth(self, data):
        line = 400//2
        line_end = 400//2 + 30
        box_width = 160
        arr = []
        for index in range(0 , 641 - box_width, 20):
            mask = data[line:line_end, index:box_width + index] != 0
            if mask.max():
                dist = int(np.percentile( data[line:line_end, index:box_width + index][mask], 5))
            else:
                dist = 0
            arr.append(dist)
        self.publish('scan', arr)
        self.scan = arr

        if self.last_detections is None:
            return

        def frameNorm(w, h, bbox):
            normVals = np.full(len(bbox), w)
            normVals[::2] = h
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        self.last_cones_distances = []
        for detection in self.last_detections:
            # ['cone', 0.92236328125, [0.42129743099212646, -0.0010452494025230408, 0.4836755692958832, 0.1296510100364685]]
            w, h = 640, 400
            a, b, c, d = frameNorm(h, h, detection[2]).tolist()
            name, x, y, width, height = detection[0], a + (w - h) // 2, b, c - a, d - b

            assert name == 'cone', name
            cone_depth = data[y:y+height, x:x+width]
            mask = cone_depth > 0
            if mask.max() == True:
                dist = cone_depth[mask].min() / 1000
            else:
                dist = None
            self.last_cones_distances.append(dist)

        if self.verbose:
            print(f'{self.time} cone at {self.last_cones_distances}')


    def on_orientation_list(self, data):
        pass

    def Xnavigate_to_goal(self, goal, timeout):
        start_time = self.time
        self.last_position_angle = self.last_position
        gps_angle = None
        while geo_length(self.last_position, goal) > 1.0 and self.time - start_time < timeout:
            desired_heading = normalizeAnglePIPI(geo_angle(self.last_position, goal))
            step = geo_length(self.last_position, self.last_position_angle)
            if step > 1.0:
                gps_angle = normalizeAnglePIPI(geo_angle(self.last_position_angle, self.last_position))
                print('step', step, math.degrees(gps_angle))
                self.last_position_angle = self.last_position
                desired_wheel_heading = normalizeAnglePIPI(desired_heading - gps_angle + self.wheel_heading)

            if gps_angle is None or self.wheel_heading is None:
                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw - 35.5))
                desired_wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)

            self.set_speed(self.max_speed, desired_wheel_heading)

            prev_time = self.time
            self.update()

            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_position, goal), self.last_imu_yaw, self.wheel_heading)

        print("STOP (3s)")
        self.set_speed(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.set_speed(0, 0)
            self.update()

# vim: expandtab sw=4 ts=4
