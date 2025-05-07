"""
  Cones Challenge 2024
"""
import datetime
import math

import numpy as np

from osgar.node import Node
from osgar.followme import EmergencyStopException


# maximal time to wait standing for any cone detection
MAX_PATIENCE = datetime.timedelta(seconds=2)


class ConesChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.stop_dist = config.get('stop_dist', 1.0)
        self.min_turn_time = datetime.timedelta(seconds=config.get('min_turn_time_sec', 3.0))
        self.verbose = False
        self.last_position = None  # not defined, probably should be 0, 0, 0
        self.last_obstacle = 0
        self.last_detections = None
        self.last_cones_distances = None  # not available
        self.raise_exception_on_stop = False
        self.field_of_view = math.radians(45)  # TODO, should clipped camera image pass it?
        self.turning_state = False
        self.turning_state_start_time = None
        self.no_detections_start_time = None
        self.verbose = False

    def on_pose2d(self, data):
        x, y, heading = data
        self.last_position = [x / 1000.0, y / 1000.0, math.radians(heading / 100.0)]
        if self.last_obstacle < self.stop_dist:  # meters
            speed, steering_angle = 0, 0
        else:
            if self.turning_state:
                speed, steering_angle = self.max_speed/2, math.radians(45)  # steer slowly max to the left
                if self.time - self.turning_state_start_time > self.min_turn_time:
                    if self.last_detections is not None and len(self.last_detections) >= 1:
                        print(self.time, 'stop turning')
                        self.turning_state = False
            else:
                speed, steering_angle = self.max_speed, 0
                if self.last_detections is not None and len(self.last_detections) >= 1:
                    self.no_detections_start_time = None  # clear, as there are some detections now
                    best = 0
                    max_x = None
                    for index, detection in enumerate(self.last_detections):
                        x1, y1, x2, y2 = detection[2]
                        if max_x is None or max_x < x1 + x2:
                            max_x = x1 + x2
                            best = index
                    x1, y1, x2, y2 = self.last_detections[best][2]

                    steering_angle = (self.field_of_view/2) * (0.5 - (x1 + x2)/2)  # steering left is positive
                    if (self.last_cones_distances is not None and len(self.last_cones_distances) > best and
                        self.last_cones_distances[best] is not None and self.last_cones_distances[best] <= 2.0):
                        print(self.time, 'start turning', self.last_cones_distances)
                        self.turning_state = True
                        self.turning_state_start_time = self.time
                else:
                    speed, steering_angle = 0, 0
                    if self.no_detections_start_time is None:
                        self.no_detections_start_time = self.time
                    if self.time - self.no_detections_start_time > MAX_PATIENCE:
                        if self.turning_state_start_time is not None:
                            print(self.time, "BLOCKED!", self.time - self.turning_state_start_time)
                            self.turning_state = True
                            # keep the same self.turning_state_start_time

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

    def on_depth(self, data):
        """
        add calculated distance to all detections - note that order may not correspond perfectly ...
        """
        if self.last_detections is None:
            return

        def frameNorm(w, h, bbox):
            normVals = np.full(len(bbox), w)
            normVals[::2] = h
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        self.last_cones_distances = []
        for detection in self.last_detections:
            # ['cone', 0.92236328125, [0.42129743099212646, -0.0010452494025230408, 0.4836755692958832, 0.1296510100364685]]
            w, h = 1280, 720  #640, 400
            a, b, c, d = frameNorm(h, h, detection[2]).tolist()
            name, x, y, width, height = detection[0], a + (w - h) // 2, b, c - a, d - b

            assert name == 'cone', name
            cone_depth = data[y:y+height, x:x+width]
            mask = cone_depth > 0
            if mask.max() == True:
                dist = cone_depth[mask].min() / 1000
            else:
                dist = None
            self.last_cones_distances.append(dist)

        if self.verbose:
            print(f'{self.time} cone at {self.last_cones_distances}')


# vim: expandtab sw=4 ts=4
