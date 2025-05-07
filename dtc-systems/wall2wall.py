"""
  Wall to wall (until bumper hit) driving
"""
import math
from datetime import timedelta

from osgar.node import Node


class Wall2wall(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.speed = config['max_speed']
        self.step_angle = math.radians(config.get('step_deg', 0))  # get change of angle step
        self.verbose = False
        self.desired_steering_angle = 0
        self.desired_speed = self.speed

    def send_speed_cmd(self, speed, steering_angle):
        self.publish('desired_steering', [round(speed*1000), round(math.degrees(steering_angle)*100)])

    def on_bumpers_front(self, data):
        if self.desired_speed > 0 and data:
            self.desired_speed = -self.speed
            self.desired_steering_angle = -self.step_angle
            print('Go back!')
        self.send_speed_cmd(self.desired_speed, self.desired_steering_angle)

    def on_bumpers_rear(self, data):
        if self.desired_speed < 0 and data:
            self.desired_speed = self.speed
            self.desired_steering_angle = self.step_angle
            print('Go forward!')
        self.send_speed_cmd(self.desired_speed, self.desired_steering_angle)


if __name__ == "__main__":
    pass

# vim: expandtab sw=4 ts=4
