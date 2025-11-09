"""
  Follow person detected via OAK camera with YOLO detector
"""

import math
from datetime import timedelta

import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.followme import EmergencyStopException  # hard to believe! :(

LEFT_LED_INDEX = 1  # to be moved into matty.py
RIGHT_LED_INDEX = 0  # to be moved into matty.py
LED_COLORS = {  # red, gree, blue
    'm01-': [0, 0, 0xFF],
    'm02-': [0, 0xFF, 0],
    'm03-': [0xFF, 0, 0],
    'm04-': [0xFF, 0x6E, 0xC7], # pink
    'm05-': [0xFF, 0x7F, 0]  # orange
}


class FollowPerson(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering',
                     'scan',  # based on depth data from camera
                     'set_leds',  # set LEDs - [index, red, green, blue]
                     )
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.horizon = config.get('horizon', 200)
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX', 'm01-')
        self.field_of_view = math.radians(45)  # TODO, review - get info from camera

        self.last_position = None
        self.verbose = False

        self.last_detections = None
        self.yaw = None
        self.debug_arr = []

        self.tracking_start_time = None
        self.status_ready = False

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.publish('set_leds', [LEFT_LED_INDEX, 0, 0, 0])  # turn off left LED
            self.publish('set_leds', [RIGHT_LED_INDEX, 0, 0xFF, 0])  # turn on green LED
            self.send_speed_cmd(0, 0)  # STOP! (note, that it could be e-stop)

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            # collision
            pass
 #           self.backup_start_time = self.time

    def on_bumpers_rear(self, data):
        pass

    def on_pose2d(self, data):
        speed, steering_angle = 0, None
        if self.last_detections is not None and len(self.last_detections) >= 1:
            if self.tracking_start_time is None:
                self.tracking_start_time = self.time
                print(self.time, f'Started tracking ... ({len(self.last_detections)})')
            best = 0
            max_x = None
            for index, detection in enumerate(self.last_detections):
                x1, y1, x2, y2 = detection[2]
                if max_x is None or max_x < x1 + x2:
                    max_x = x1 + x2
                    best = index
            x1, y1, x2, y2 = self.last_detections[best][2]
            steering_angle = (self.field_of_view / 2) * (0.5 - (x1 + x2) / 2)  # steering left is positive
            speed = self.max_speed  # TODO on/off, maybe even based on angle difference?
#            if self.last_cones_distances is not None and len(self.last_cones_distances) > best and self.last_cones_distances[best] is not None:
#                if ((self.last_cones_distances[best] < self.report_dist or y1 < 0.1)
#                        and self.report_start_time is None):
#                    print(self.time, 'SCANNING PERSON started', y1, y2, self.last_cones_distances[best])
        else:
            if self.tracking_start_time is not None:
                print(self.time, f'Lost track {self.time - self.tracking_start_time}')
                self.tracking_start_time = None

        if steering_angle is None:
            # no way to go! -> STOP and look around
            speed, steering_angle = 0, 0
        if self.verbose:
            print(speed, steering_angle)
        self.send_speed_cmd(speed, steering_angle)

    def on_nmea_data(self, data):
        assert 'lat' in data, data
        assert 'lon' in data, data
        lat, lon = data['lat'], data['lon']
        utc_time = data['utc_time']
        if lon is not None and data['lon_dir'] == 'W':
            lon = -lon
        if lat is not None and data['lat_dir'] == 'S':
            lat = -lat
        if lat is not None and lon is not None:
            self.last_position = lat, lon

    def on_detections(self, data):
        self.last_detections = [det for det in data if det[0] == 'person']

    def on_depth(self, data):
        data = data.copy()
        line = self.horizon - 30
        line_end = self.horizon + 30
        box_width = 160
        arr = []
        for index in range(0 , 641 - box_width, 20):
            mask = data[line:line_end, index:box_width + index] == 0
            data[line:line_end, index:box_width + index][mask] = 10000 # 10m
            dist = int(np.percentile( data[line:line_end, index:box_width + index], 5))
            arr.append(dist)
        self.publish('scan', arr)
        self.scan = arr

        if not self.status_ready:
            self.publish('set_leds', [LEFT_LED_INDEX] + [v//2 for v in LED_COLORS.get(self.system_name, [0, 0, 0])])
            self.publish('set_leds', [RIGHT_LED_INDEX] + [v//2 for v in LED_COLORS.get(self.system_name, [0, 0, 0])])
            self.status_ready = True

    def on_orientation_list(self, data):
        pass

    def on_rotation(self, data):
        yaw, pitch, roll = data
        self.yaw = math.radians(yaw/100.0)

# vim: expandtab sw=4 ts=4
