"""
  RoboOrienteering - navigate in terrain on GPS points with cones
"""

import argparse
import math
import threading

from datetime import timedelta

from osgar.lib.config import config_load
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.record import record
from osgar.node import Node

from osgar.drivers.gps import INVALID_COORDINATES


def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale


def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[0]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


def latlon2xy(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))



class RoboOrienteering(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.2)
        self.goals = [latlon2xy(lat, lon) for lat, lon in config['waypoints']]
        self.last_position = None  # (lon, lat) in milliseconds

        """
        self.last_imu_yaw = None  # magnetic north in degrees
        self.status = None
        self.wheel_heading = None
        self.cmd = [0, 0]
        self.monitors = []
        self.last_position_angle = None  # for angle computation from dGPS
        """

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        pass

    def on_pose2d(self, data):
        if math.hypot(data[0]/1000.0, data[1]/1000.0) >= 3.0:
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, 0
        if self.verbose:
            print(speed, steering_angle)
        self.send_speed_cmd(speed, steering_angle)

    def on_nmea_data(self, data):
        assert 'lat' in data, data
        assert 'lon' in data, data
        lat, lon = data['lat'], data['lon']
        if lat is not None and lon is not None:
            assert 0, data

    def on_detections(self, data):
        pass

    def on_depth(self, data):
        pass

    def on_orientation_list(self, data):
        pass


    def Xupdate(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('RO', packet)
            timestamp, channel, data = packet
            self.time = timestamp
            if channel == 'position':
                self.last_position = data
            elif channel == 'orientation':
                (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z) = data
                self.last_imu_yaw = yaw
            elif channel == 'status':  # i.e. I can drive only spider??
                self.status, steering_status = data
                if steering_status is None:
                    self.wheel_heading = None
                else:
                    self.wheel_heading = math.radians(-360 * steering_status[0] / 512)
                self.bus.publish('move', self.cmd)
            for monitor_update in self.monitors:
                monitor_update(self)

    def Xset_speed(self, desired_speed, desired_wheel_heading):
        # TODO split this for Car and Spider mode
        speed = int(round(desired_speed))
        desired_steering = int(-512 * math.degrees(desired_wheel_heading) / 360.0)

        if speed != 0:
            if self.wheel_heading is None:
                speed = 1  # in in place
            else:
                 d = math.degrees(normalizeAnglePIPI(self.wheel_heading - desired_wheel_heading))
                 if abs(d) > 20.0:
                     speed = 1  # turn in place (II.)

        self.cmd = [speed, desired_steering]

    def Xstart(self):
        self.thread = threading.Thread(target=self.play)
        self.thread.start()


    def Xplay(self):
        print("Waiting for valid GPS position...")
        while self.last_position is None or self.last_position == INVALID_COORDINATES:
            self.update()
        print(self.last_position)

        print("Wait for valid IMU...")
        while self.last_imu_yaw is None:
            self.update()
        print(self.last_imu_yaw)

        print("Wait for steering info...")
        while self.wheel_heading is None:
            self.update()
        print(math.degrees(self.wheel_heading))

        print("Ready", self.goals)
        try:
            with EmergencyStopMonitor(self):
                for goal in self.goals:
                    print("Goal at %.2fm" % geo_length(self.last_position, goal))
                    angle = geo_angle(self.last_position, goal)
                    if angle is not None:
                        print("Heading %.1fdeg, imu" % math.degrees(angle), self.last_imu_yaw)
                    else:
                        print("Heading None, imu", self.last_imu_yaw)
                    self.navigate_to_goal(goal, timedelta(seconds=200))
        except EmergencyStopException:
            print("EMERGENCY STOP (wait 3s)")
            self.set_speed(0, 0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=3):
                self.set_speed(0, 0)
                self.update()

    def navigate_to_goal(self, goal, timeout):
        start_time = self.time
        self.last_position_angle = self.last_position
        gps_angle = None
        while geo_length(self.last_position, goal) > 1.0 and self.time - start_time < timeout:
            desired_heading = normalizeAnglePIPI(geo_angle(self.last_position, goal))
            step = geo_length(self.last_position, self.last_position_angle)
            if step > 1.0:
                gps_angle = normalizeAnglePIPI(geo_angle(self.last_position_angle, self.last_position))
                print('step', step, math.degrees(gps_angle))
                self.last_position_angle = self.last_position
                desired_wheel_heading = normalizeAnglePIPI(desired_heading - gps_angle + self.wheel_heading)

            if gps_angle is None or self.wheel_heading is None:
                spider_heading = normalizeAnglePIPI(math.radians(180 - self.last_imu_yaw - 35.5))
                desired_wheel_heading = normalizeAnglePIPI(desired_heading-spider_heading)

            self.set_speed(self.max_speed, desired_wheel_heading)

            prev_time = self.time
            self.update()

            if int(prev_time.total_seconds()) != int(self.time.total_seconds()):
                print(self.time, geo_length(self.last_position, goal), self.last_imu_yaw, self.wheel_heading)

        print("STOP (3s)")
        self.set_speed(0, 0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=3):
            self.set_speed(0, 0)
            self.update()

# vim: expandtab sw=4 ts=4
