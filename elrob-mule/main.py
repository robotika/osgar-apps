"""
  ELROB Mule 2024
"""
import datetime
import math
from unittest.mock import patch

import numpy as np

from osgar.node import Node
from osgar.followme import EmergencyStopException, FollowMe
from osgar.lib import quaternion


def dummy_register(name):
    print(f'Fake registration of "{name}"')


class Mule(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        save_register_fn = bus.register
        bus.register = dummy_register
        self.app = FollowMe(config, bus)
        self.app.publish = self.my_publish
        bus.register = save_register_fn
        self.verbose = False

    def my_publish(self, name, data):
        self.publish(name, data)

    def on_pose2d(self, data):
        self.app.time = self.time
        self.app.on_pose2d(data)

    def on_scan(self, data):
        self.app.time = self.time
        self.app.on_scan(data)

    def on_emergency_stop(self, data):
        self.app.time = self.time
        self.app.on_emergency_stop(data)

    def run(self):
        self.app.verbose = self.verbose
        self.app.run()

# vim: expandtab sw=4 ts=4
