"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node


class Click2Go(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'pose2dimg')
        self.start_pose = None
        self.traveled_dist = 0.0
        self.verbose = False
        self.max_speed = config['max_speed']
        self.dist = 0.2  # TODO based on click coordinate
        self.timeout = timedelta(seconds=config['timeout'])

        self.emergency_stop = None
        self.last_h26x_image = None
        self.last_cmd = None
        self.last_pose2d = [0, 0, 0]

    def send_speed_cmd(self, speed, steering_angle):
        self.publish('desired_steering', [round(speed*1000), round(math.degrees(steering_angle)*100)])

    def on_pose2d(self, data):
        self.last_pose2d = data[:]
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.start_pose is None:
            self.start_pose = pose
        self.traveled_dist = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])
        if self.traveled_dist >= self.dist:
            self.last_cmd = None
            self.traveled_dist = 0.0
            self.start_pose = pose
        if self.last_cmd is None:
            self.send_speed_cmd(0, 0)
        else:
            if self.last_cmd[-1][0] < 640:
                steering_angle = math.radians(20)
            elif self.last_cmd[-1][0] > 2*640:
                steering_angle = math.radians(-20)
            else:
                steering_angle = 0
            self.send_speed_cmd(self.max_speed, steering_angle)

    def on_emergency_stop(self, data):
        self.emergency_stop = data

    def on_tick(self, data):
        pass

    def on_color(self, data):
        if data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 460150')):
            # I - key frame
            self.last_h26x_image = data
            self.publish('pose2dimg', [self.last_pose2d, self.last_h26x_image])

    def on_cmd(self, data):
        print('New cmd:', data)
        self.last_cmd = data.copy()

# vim: expandtab sw=4 ts=4
