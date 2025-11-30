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

MAX_STEERING_AGE = 10
P_SCALE_STEERING_DIFF = 2.0  # scaling factor for steering


class FollowPerson(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering',
                     'scan',  # based on depth data from camera
                     'set_leds',  # set LEDs - [index, red, green, blue]
                     )
        self.max_speed = config.get('max_speed', 0.2)
        self.min_dist_limit = config.get('min_dist_limit', None)  # default unlimited
        self.turn_angle = config.get('turn_angle', 20)
        self.horizon = config.get('horizon', 200)
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX', 'm01-')
        self.field_of_view = math.radians(69)  # OAK-D Pro color camera TODO review night -> config

        self.last_position = None
        self.verbose = False

        self.last_detections = None
        self.yaw = None
        self.debug_arr = []

        self.tracking_start_time = None
        self.status_ready = False
        self.last_steering = None
        self.last_steering_age = None

        # Pozyx/Qorvo
        self.follow_enabled = None  # not known
        self.follow_last_dist = None  # not known

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.publish('set_leds', [LEFT_LED_INDEX, 0, 0, 0])  # turn off left LED
            self.publish('set_leds', [RIGHT_LED_INDEX, 0, 128, 0])  # turn on green LED
            self.send_speed_cmd(0, 0)  # STOP! (note, that it could be e-stop)

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            # collision
            pass

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
            steering_angle = P_SCALE_STEERING_DIFF * (self.field_of_view / 2) * (0.5 - (x1 + x2) / 2)  # steering left is positive
            speed = self.max_speed  # TODO on/off
            self.last_steering = steering_angle
            self.last_steering_age = 0
        else:
            if self.last_steering is not None and self.last_steering_age < MAX_STEERING_AGE:
                speed, steering_angle = self.max_speed, self.last_steering  # or slower? or depending on age
                self.last_steering_age += 1
            if self.tracking_start_time is not None:
                print(self.time, f'Lost track {self.time - self.tracking_start_time}')
                self.tracking_start_time = None

        if steering_angle is None:
            # no way to go! -> STOP and look around
            speed, steering_angle = 0, 0
        if self.min_dist_limit is not None:
            if self.follow_last_dist is None or self.follow_last_dist < self.min_dist_limit:
                speed = 0
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

    def on_tag_range(self, data):
        # Qorvo data [[19767, 951], [20396, 291]] ... [ID, dist_mm] to anchors
        self.follow_enabled = (len(data) > 0)  # power off for disabled service
        if len(data) == 1:  # not supported multiple anchors yet
            self.follow_last_dist = data[0][1]/1000.0  # data in millimeters

    def on_pozyx_range(self, data):
        # [1, 3431, 3411, [2777589, 357, -78]]
        if data[0] == 1:
            tag = 0x6827
            if data[1] == tag or data[2] == tag:
                src = data[1] if data[2] == tag else data[2]
                dist = data[3][1] / 1000
                """
                if src == self.left_id:
                    self.left_range_arr.append(dist)
                    self.left_range_arr = self.left_range_arr[-FILTER_SIZE:]
                    self.left_range = median(self.left_range_arr)
                elif src == self.right_id:
                    self.right_range_arr.append(dist)
                    self.right_range_arr = self.right_range_arr[-FILTER_SIZE:]
                    self.right_range = median(self.right_range_arr)
                else:
                    assert src is None, src
                    self.back_range = dist

                if self.left_range is not None and self.right_range is not None:
                    diff = self.left_range - self.right_range
                    dist = (self.left_range + self.right_range)/2
                    if self.verbose:
                        print(diff, dist)
                        self.debug_arr.append((self.time.total_seconds(), diff))
                    angular_speed = math.radians(10)
                    speed = 0.0
                    if dist > 1.2:
                        speed = min(0.5, 0.1 + (dist - 1.2) * 0.4)
                    if self.last_min_dist is not None and self.last_min_dist < 700:
                        speed = 0.0

                    if self.follow_enabled:
                        if abs(diff) < 0.05:
                            self.send_speed_cmd(speed, 0.0)
                        elif diff > 0:
                            self.send_speed_cmd(speed, -angular_speed)
                        else:
                            self.send_speed_cmd(speed, angular_speed)
                    else:
                        self.send_speed_cmd(0, 0)
                        """

    def on_pozyx_gpio(self, data):
        # [1, 26663, 0]
        valid, device_id, digital_input = data
        if valid:
            self.follow_enabled = (digital_input == 0)

# vim: expandtab sw=4 ts=4
