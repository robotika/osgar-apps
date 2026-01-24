"""
  Test basic driving functionality
"""
import math
from datetime import timedelta

from osgar.node import Node


class Click2Go(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed', 'desired_steering', 'image')
        self.start_pose = None
        self.traveled_dist = 0.0
        self.verbose = False
        self.speed = config['max_speed']
        self.dist = config['dist']
        self.timeout = timedelta(seconds=config['timeout'])

        self.desired_steering_angle = None  # not defined, do not publish by default
        self.desired_angular_speed = None
        steering_deg = config.get('steering_deg')  # desired steering angle in degrees
        angular_speed_degs = config.get('angular_speed_degs')  # desired angular speed in degrees per second
        if steering_deg is None and angular_speed_degs is None:
            # if nothing is specified publish both interfaces
            self.desired_steering_angle = 0
            self.desired_angular_speed = 0
        if steering_deg is not None:
            self.desired_steering_angle = math.radians(steering_deg)
        if angular_speed_degs is not None:
            self.desired_angular_speed = math.radians(angular_speed_degs)

        self.repeat = config.get('repeat', 1)
        self.emergency_stop = None

    def send_speed_cmd(self, speed, angular_speed=None, steering_angle=None):
        if angular_speed is not None:
            self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])
        if steering_angle is not None:
            self.publish('desired_steering', [round(speed*1000), round(math.degrees(steering_angle)*100)])

    def on_pose2d(self, data):
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.start_pose is None:
            self.start_pose = pose
        self.traveled_dist = math.hypot(pose[0] - self.start_pose[0], pose[1] - self.start_pose[1])

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

    def on_cmd(self, data):
#        image = bytes([10, 20, 30])
        with open('save.h264', 'rb') as f:
            image = f.read()
        self.publish('image', image)  # TODO change to "image"

    def sub_run(self):
        self.update()  # define self.time
        print(self.time, "Go!")
        start_time = self.time
        if self.dist >= 0:
            self.send_speed_cmd(self.speed,
                                angular_speed=self.desired_angular_speed, steering_angle=self.desired_steering_angle)
        else:
            # TODO flip also steering angle?!
            self.send_speed_cmd(-self.speed,
                                angular_speed=self.desired_angular_speed, steering_angle=self.desired_steering_angle)
        while self.traveled_dist < abs(self.dist) and self.time - start_time < self.timeout:
            self.update()
            if self.emergency_stop:
                print(self.time, "(sub_run) Emergency STOP")
                break
        print(self.time, "STOP")
        self.send_speed_cmd(0.0, angular_speed=0.0, steering_angle=self.desired_steering_angle)
        self.wait(timedelta(seconds=1))
        print(self.time, "distance:", self.traveled_dist, "time:", (self.time - start_time).total_seconds())

    def run(self):
        for run_number in range(self.repeat):
            self.traveled_dist = 0.0
            self.start_pose = None
            self.sub_run()
            if self.emergency_stop:
                print(self.time, "(run) Emergency STOP")
                break
            self.dist = -self.dist  # next run will be in opposite direction

# vim: expandtab sw=4 ts=4
