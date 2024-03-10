"""
  Cones Challenge 2024
"""
import math

from osgar.node import Node
from osgar.followme import EmergencyStopException


class ConesChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.verbose = False
        self.last_position = None  # not defined, probably should be 0, 0, 0

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        speed, angular_speed = self.control(self.last_position)
        if self.verbose:
            print(speed, angular_speed)
        self.send_speed_cmd(speed, angular_speed)

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def send_speed_cmd(self, speed, angular_speed):
        return self.bus.publish(
            'desired_speed',
            [round(speed*1000), round(math.degrees(angular_speed)*100)]
        )

# vim: expandtab sw=4 ts=4
