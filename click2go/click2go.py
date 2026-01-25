"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node


class Click2Go(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'image')
        self.start_pose = None
        self.traveled_dist = 0.0
        self.verbose = False
        self.max_speed = config['max_speed']
        self.dist = 0.2  # TODO based on click coordinate
        self.timeout = timedelta(seconds=config['timeout'])

        self.emergency_stop = None
        self.last_h26x_image = None
        self.last_cmd = None
        self.img_count = 0

    def send_speed_cmd(self, speed, steering_angle):
        self.publish('desired_steering', [round(speed*1000), round(math.degrees(steering_angle)*100)])

    def on_pose2d(self, data):
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
            self.send_speed_cmd(self.max_speed, 0)

    def on_emergency_stop(self, data):
        self.emergency_stop = data

    def wait(self, dt):  # TODO refactor to some common class
        if self.time is None:
            self.update()
        start_time = self.time
        while self.time - start_time < dt:
            self.update()

    def on_tick(self, data):
        pass

    def on_color(self, data):
        if data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 460150')):
            # I - key frame
            self.last_h26x_image = data
            self.img_count += 1
            if self.img_count >= 1:
                self.img_count = 0
                self.publish('image', self.last_h26x_image)

    def on_cmd(self, data):
        print('New cmd:', data)
        if self.last_h26x_image is None:
            with open('save.h264', 'rb') as f:
                image = f.read()
        else:
            image = self.last_h26x_image
        self.publish('image', image)
        self.last_cmd = data.copy

# vim: expandtab sw=4 ts=4
