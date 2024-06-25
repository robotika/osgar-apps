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
        bus.register('desired_speed')
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

    def my_publish(self, name, data):
        self.publish(name, data)

    def my_listen(self):
        self.listen()

    def my_update(self):
        self.update()

    def on_pose2d(self, data):
        self.app.time = self.time
        self.app.on_pose2d(data)
        if len(self.path) == 0 or math.hypot(self.path[-1][0] - data[0], self.path[-1][1] - data[-1]):
            self.path.append(data)

    def on_scan(self, data):
        self.app.time = self.time
        self.app.on_scan(data)

    def on_emergency_stop(self, data):
        self.app.time = self.time
        self.app.on_emergency_stop(data)

    def dummy_handler(self, data):
        pass

    def run(self):
        self.app.verbose = self.verbose
        self.app.run()

        # switch to follow path
        self.path.reverse()
        self.app2.last_position = self.app.last_position  # assign end of route
        self.app = self.app2
        self.app.route = Route(pts=self.path)
        self.app.publish = self.my_publish
        self.app.listen = self.my_listen
        self.app.update = self.my_update
        self.app.on_scan = self.dummy_handler  # FollowPath does not have on_scan
        self.app.run()



# vim: expandtab sw=4 ts=4
