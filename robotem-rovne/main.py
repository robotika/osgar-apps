"""
Robotem Rovne 2024
"""

import math

import cv2
import numpy as np
from osgar.followme import EmergencyStopException
from osgar.lib import quaternion
from osgar.lib.route import Convertor
from osgar.node import Node


def mask_center(mask):
    if mask.max() == 0:
        return mask.shape[0] // 2, mask.shape[1] // 2
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
        self.min_safe_dist = config.get('min_safe_dist', 2.0)  # meters, steering filter
        self.danger_dist = config.get('danger_dist', 1.2)  # meters, speed stop
        self.limit_dist = config.get('dist_limit', None)
        self.verbose = False
        self.last_position = None
        self.last_obstacle = 0
        self.last_nn_mask = None
        self.last_depth = None
        self.last_dir = 0  # straight
        self.start_lon_lat = None
        self.raise_exception_on_stop = config.get('terminate_on_stop', False)
        self.blocked_count = 0

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        if self.last_nn_mask is None:
            speed, steering_angle = 0, 0
        elif self.stop_dist > 0 and self.last_obstacle < self.stop_dist:
            speed, steering_angle = 0, 0
        elif self.blocked_count >= 5:
            # Full stop if central path is blocked for several frames
            speed, steering_angle = 0, 0
        elif self.blocked_count > 0:
            # Slow down if something is in the way
            speed, steering_angle = self.max_speed / 2.0, self.last_dir
        else:
            speed, steering_angle = self.max_speed, self.last_dir

        if self.verbose:
            print(f'{speed} {steering_angle}')
        self.send_speed_cmd(speed, steering_angle)

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish('desired_steering', [round(speed * 1000), round(math.degrees(steering_angle) * 100)])

    def on_obstacle(self, data):
        self.last_obstacle = data

    def on_detections(self, data):
        pass

    def on_depth(self, data):
        self.last_depth = data

    def on_nmea_data(self, data):
        if ('lat' in data and 'lon' in data) and (data['lat'] is not None and data['lon'] is not None):
            lat, lon = data['lat'], data['lon']
            if self.start_lon_lat is None:
                self.start_lon_lat = (lon, lat)
            conv = Convertor(refPoint=self.start_lon_lat)
            dist = math.hypot(*conv.geo2planar((lon, lat)))
            if self.limit_dist is not None and dist > self.limit_dist:
                print(self.time, 'reached', dist)
                raise EmergencyStopException()

    def on_nn_mask(self, data):
        self.last_nn_mask = data.copy()
        height, width = self.last_nn_mask.shape
        self.last_nn_mask[: height // 2, :] = 0  # remove sky detections

        if self.last_depth is not None:
            # Resize depth to mask size for fusion
            depth_resized = cv2.resize(self.last_depth, (width, height), interpolation=cv2.INTER_NEAREST)

            # 1. Check for immediate danger in central path (Narrow ROI)
            roi_y_start, roi_y_end = int(height * 0.4), int(height * 0.7)
            roi_x_start, roi_x_end = int(width * 0.4), int(width * 0.6)

            danger_zone = depth_resized[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
            valid_danger = danger_zone[danger_zone > 0]

            if len(valid_danger) > 0 and np.percentile(valid_danger, 10) < self.danger_dist * 1000:
                self.blocked_count = min(self.blocked_count + 1, 10)
            elif self.blocked_count > 0:
                self.blocked_count -= 1

            # 2. Filter road mask by depth for steering (Wider ROI)
            safe_dist_mm = self.min_safe_dist * 1000
            obstacle_mask = (depth_resized < safe_dist_mm) & (depth_resized > 0)
            self.last_nn_mask[obstacle_mask] = 0

        center_y, center_x = mask_center(self.last_nn_mask)
        dead = width // 16
        turn_angle = math.radians(20)
        if center_x > width // 2 + dead:
            self.last_dir = -turn_angle
        elif center_x < width // 2 - dead:
            self.last_dir = turn_angle
        else:
            self.last_dir = 0

        if self.verbose:
            print(f'{self.time} center_x: {center_x}, blocked: {self.blocked_count}')

    def on_orientation_list(self, data):
        if self.verbose:
            for quat in data:
                print(self.last_position, quaternion.heading(quat[2:]))


# vim: expandtab sw=4 ts=4
