"""
  Cones Challenge 2024
"""
import datetime
import math

from osgar.node import Node
from osgar.followme import EmergencyStopException


class ConesChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.stop_dist = config.get('stop_dist', 1.0)
        self.verbose = False
        self.last_position = None  # not defined, probably should be 0, 0, 0
        self.last_obstacle = 0
        self.last_detections = None
        self.raise_exception_on_stop = False
        self.field_of_view = math.radians(45)  # TODO, should clipped camera image pass it?
        self.turning_state = False
        self.turning_state_start_time = None

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        if self.last_obstacle < self.stop_dist:  # meters
            speed, steering_angle = 0, 0
        else:
            if self.turning_state:
                speed, steering_angle = self.max_speed, -45  # steer max to the left (check signs!)
                if self.turning_state_start_time > datetime.timedelta(seconds=20):
                    if self.last_detections is not None and len(self.last_detections) == 1:
                        self.turning_state = False
            else:
                speed, steering_angle = self.max_speed, 0
                if self.last_detections is not None and len(self.last_detections) == 1:
                    x1, y1, x2, y2 = self.last_detections[0][2]
                    steering_angle = (self.field_of_view/2) * (0.5 - (x1 + x2)/2)  # steering left is positive
                    if self.last_obstacle <= 1.5:
                        self.turning_state = True
                        self.turning_state_start_time = self.time
                else:
                    speed, steering_angle = 0, 0
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
        self.last_detections = data[:]

# vim: expandtab sw=4 ts=4
