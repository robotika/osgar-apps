"""
  Tulak po krasu 2026 - combination of RedRoad and Follow AprilTag
"""

import os
import math
from datetime import timedelta

import av
import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.exceptions import EmergencyStopException


def mask_center(mask):
    if mask.max() == 0:
        return mask.shape[0]//2, mask.shape[1]//2
    assert mask.max() == 1, mask.max()
    indices = np.argwhere(mask == 1)  # shape (num_points, 2)

    # Compute the center of mass as the mean of these indices
    return tuple(int(x) for x in indices.mean(axis=0))


class Tulak(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')

        # Configuration parameters
        self.max_speed = config.get('max_speed', 0.5)
        self.target_distance = config.get('target_distance', 0.5)
        self.Kp_distance = config.get('Kp_distance', 0.5)
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)

        self.last_nn_mask = None
        self.last_dir = 0  # straight

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.send_speed_cmd(0, 0)  # STOP!

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def on_bumpers_rear(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def on_apriltags(self, data):
        pass

    def on_targets(self, data):
        if len(data) == 0:
            # fallback to nn-mask
            speed, steering_angle = self.max_speed, self.last_dir
            self.send_speed_cmd(speed, steering_angle)
        else:
            dist, angle = data[0]  # TODO select nearest
            if dist < self.target_distance:
                self.send_speed_cmd(0, 0)
            else:
                self.send_speed_cmd(self.max_speed, angle)

    def on_nn_mask(self, data):
        self.last_nn_mask = data.copy()  # make sure you modify only own copy
        #        assert self.last_nn_mask.shape == (120, 160), self.last_nn_mask.shape
        height, width = self.last_nn_mask.shape
        self.last_nn_mask[:height // 2, :] = 0  # remove sky detections

        center_y, center_x = mask_center(self.last_nn_mask)
        dead = width // 16  # was 10
        turn_angle = math.radians(20)
        if center_x > width // 2 + dead:
            self.last_dir = -turn_angle
        elif center_x < width // 2 - dead:
            self.last_dir = turn_angle
        else:
            self.last_dir = 0  # straight

    def on_depth(self, data):
        pass

    def on_pose2d(self, data):
        pass

    def on_rotation(self, data):
        pass

    def on_orientation_list(self, data):
        pass

    def on_nmea_data(self, data):
        pass

# vim: expandtab sw=4 ts=4
