"""
  ELROB Mule 2024
"""
import datetime
import math
from unittest.mock import patch
from enum import Enum

import numpy as np

from osgar.node import Node
from osgar.followme import EmergencyStopException, FollowMe
from osgar.followpath import FollowPath, Route

from osgar.lib import quaternion


class MuleModes(Enum):
    FOLLOWME = 1
    BACKWARD = 2
    FORWARD = 3


def dummy_register(name):
    print(f'Fake registration of "{name}"')


class Mule(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed', 'pose2d')
        self.min_step = config.get('min_step', 0.5)
        save_register_fn = bus.register
        bus.register = dummy_register
        self.app = FollowMe(config, bus)
        self.app.publish = self.my_publish
        self.app.listen = self.my_listen
        self.app.update = self.my_update
        self.app2 = FollowPath(config, bus)

        bus.register = save_register_fn
        self.verbose = False
        self.mode = MuleModes.FOLLOWME
        self.path = []
        self.last_imu_heading = None
        self.prev_heading = None  # already processed heading
        self.prev_pose = None
        self.prev_old_pose = None

    def my_publish(self, name, data):
        self.publish(name, data)

    def my_listen(self):
        self.listen()

    def my_update(self):
        self.update()

    def correct_pose(self, pose2d):
        pose = pose2d[0]/1000.0, pose2d[1]/1000.0, math.radians(pose2d[2]/100.0)
        if self.prev_pose is None or self.prev_heading is None:
            new_pose = pose
        else:
            dist = math.hypot(pose[0] - self.prev_old_pose [0], pose[1] - self.prev_old_pose [1])
            angle = self.last_imu_heading - self.prev_heading
            new_pose = (self.prev_pose[0] + dist * math.cos(self.prev_pose[2] + angle),
                        self.prev_pose[1] + dist * math.sin(self.prev_pose[2] + angle),
                        self.prev_pose[2] + angle)
        self.prev_pose = new_pose
        self.prev_old_pose = pose
        self.prev_heading = self.last_imu_heading
        x, y, heading = new_pose
        return int(x * 1000), int(y * 1000), int(math.degrees(heading) * 100)

    def on_pose2d(self, data):
        self.app.time = self.time
#        data = self.correct_pose(data)
#        self.publish('pose2d', data)  # corrected pose by IMU
        self.app.on_pose2d(data)
        x, y = data[0]/1000.0, data[1]/1000.0
        if len(self.path) == 0 or math.hypot(self.path[-1][0] - x, self.path[-1][1] - y):
            self.path.append((x, y))

    def on_scan(self, data):
        self.app.time = self.time
        self.app.on_scan(data)

    def on_emergency_stop(self, data):
        self.app.time = self.time
        self.app.on_emergency_stop(data)

    def on_orientation_list(self, data):
        for quat in data:
            last_imu = quaternion.heading(quat[2:])
        self.last_imu_heading = last_imu

    def dummy_handler(self, data):
        pass

    def run(self):
        self.app.verbose = self.verbose
        self.app.run()

        # switch to follow path
        self.app2.last_position = self.app.last_position  # assign end of route
        self.app = self.app2
        self.app.publish = self.my_publish
        self.app.listen = self.my_listen
        self.app.update = self.my_update
        self.app.on_scan = self.dummy_handler  # FollowPath does not have on_scan
        self.app.verbose = self.verbose
        self.path.reverse()
        self.app.route = Route(pts=self.path)
        self.app.finished = False
        print(f'FollowPath {self.app.route.pts[0]} -> {self.app.route.pts[-1]}')
        self.app.run()


# vim: expandtab sw=4 ts=4
