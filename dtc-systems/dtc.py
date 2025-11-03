"""
  DARPA Triage Challenge - System (copy of RoboOrienteering)
"""

import math
from datetime import timedelta

import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.followme import EmergencyStopException  # hard to believe! :(
from geofence import Geofence
from report import DTCReport, normalize_matty_name, pack_data
from doctor import DTC_QUERY_SOUND

MAX_CMD_HISTORY = 100  # beware of dependency on pose2d update

SCANNING_TIME_SEC = 13  # 8s talking 5s listening

LEFT_LED_INDEX = 1  # to be moved into matty.py
RIGHT_LED_INDEX = 0  # to be moved into matty.py
LED_COLORS = {  # red, gree, blue
    'm01-': [0, 0, 0xFF],
    'm02-': [0, 0xFF, 0],
    'm03-': [0xFF, 0, 0],
    'm04-': [0xFF, 0x6E, 0xC7], # pink
    'm05-': [0xFF, 0x7F, 0]  # orange
}

def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    x_scale = math.cos(math.radians(pos1[1]/3600000))  # based on lat
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale


def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 1.0:
        return None
    x_scale = math.cos(math.radians(pos1[1]/3600000))  # based on lat
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)


def latlon2xy(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))



class DARPATriageChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering',
                     'scan',  # based on depth data from camera
                     'report_latlon',  # dictionary {'lat': degrees, 'lon': degrees}
                     'scanning_person',  # data collection from nearby position of causalty (Boolean)
                     'play_sound',  # filename without extension in sounds/ folder
                     'lora_latlon',  # LoRa encoded empty encoded DTC report
                     'set_leds',  # set LEDs - [index, red, green, blue]
                     )
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.horizon = config.get('horizon', 200)
        self.waypoints = config.get('waypoints', [])[1:]  # remove start
        self.debug_all_waypoints = config.get('waypoints', [])[:]
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX', 'm01-')

        self.geofence = None
        # try system specific geofence
        geofence_lat_lon = config.get(self.system_name + 'geofence')
        if geofence_lat_lon is None:
            # if not available use common geofence
            geofence_lat_lon = config.get('geofence')
        if geofence_lat_lon is not None:
            self.geofence = Geofence(geofence_lat_lon)
            if len(self.waypoints) == 0:
                pt = self.geofence.get_random_inner_waypoint()
                print(f'Adding RND waypoint {pt}')
                self.waypoints.append(pt)

        self.last_position = None
        self.verbose = False
        self.scan = None
        self.backup_start_time = None
        self.report_start_time = None
        self.tracking_start_time = None

        self.last_detections = None
        self.last_cones_distances = None  # not available
        self.field_of_view = math.radians(45)  # TODO, should clipped camera image pass it?
        self.report_dist = config.get('report_dist', 2.0)
        self.is_scanning_person = False

        self.closest_waypoint = None
        self.closest_waypoint_dist = None
        self.gps_heading = None
        self.yaw = None
        self.debug_arr = []

        self.look_around = False  # in case of blocked path look left and right and pick direction
        self.cmd_history = []
        self.status_ready = False

    def send_speed_cmd(self, speed, steering_angle):
        self.cmd_history.append((speed, steering_angle))
        if len(self.cmd_history) > MAX_CMD_HISTORY:
            self.cmd_history = self.cmd_history[-MAX_CMD_HISTORY:]
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.publish('set_leds', [LEFT_LED_INDEX, 0, 0, 0])  # turn off left LED
            self.send_speed_cmd(0, 0)  # STOP! (note, that it could be e-stop)

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            # collision
            self.backup_start_time = self.time

    def on_bumpers_rear(self, data):
        pass

    def get_direction(self, arr):
        # based on FRE2025 code
        center = len(arr) // 2
        direction = 0  # default, if you cannot decide, go straight
        if arr[center] > 1000:
            # no close obstacle -> go straight
            direction = 0
            # check left and right limits for free space
            left = 0
            for i in range(0, center):
                if arr[center - i] > 1000:
                    left = i
                else:
                    break
            right = 0
            for i in range(0, center):
                if arr[center + i] > 1000:
                    right = i
                else:
                    break
            if self.verbose:
                print(self.time, left, right)
            if left <= 2 or right <= 2:
                if left >= 5:
                    # right is too close
                    direction = self.turn_angle // 2
                if right >= 5:
                    ## left is too close
                    direction = -self.turn_angle // 2
        else:
            # cannot go straight
            for i in range(1, center):
                if arr[center - i] > 1000 and arr[center + i] <= 1000:
                    # free space on the left
                    direction = self.turn_angle
                    break
                elif arr[center - i] <= 1000 and arr[center + i] > 1000:
                    # free space on the right
                    direction = -self.turn_angle
                    break
            else:
                direction = None
                if self.verbose:
                    print(self.time, "NO FREE SPACE", direction)
        if direction is not None and direction != 0:
            direction = math.radians(direction)  # fix agresive steering but leave 0 as default action
        return direction

    def on_pose2d(self, data):
        if self.backup_start_time is not None:
            # front collision, backup for 5s
            if self.time - self.backup_start_time < timedelta(seconds=5):
                self.send_speed_cmd(-0.2, math.radians(10))  # try a small arc
                return  # terminate with reverse motion
            else:
                self.backup_start_time = None  # end of collision

        if self.report_start_time is not None:
            # report via stop 3s
            if self.time - self.report_start_time < timedelta(seconds=SCANNING_TIME_SEC):
                self.send_speed_cmd(0, 0)
                return  # terminate without other driving
            elif self.time - self.report_start_time < timedelta(seconds=SCANNING_TIME_SEC + 2):
                self.send_speed_cmd(-0.25, 0)
                return  # reverse 0.5m
            elif self.time - self.report_start_time < timedelta(seconds=SCANNING_TIME_SEC + 4):
                # experimental - use also backup data collection
                if self.is_scanning_person:
                    self.is_scanning_person = False
                    self.publish('scanning_person', self.is_scanning_person)
                self.send_speed_cmd(0.2, math.radians(-45))  # turn right
                return  # reverse 0.5m
            elif self.time - self.report_start_time < timedelta(seconds=SCANNING_TIME_SEC + 9):
                # ignore detections for a moment (10s)
                pass  # waypoints no longer correspond to cones/expected locations of objects
            else:
                self.report_start_time = None  # end of report

        if self.scan is None:
            # no depth data yet
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, self.get_direction(self.scan)
            if self.last_detections is not None and len(self.last_detections) >= 1 and steering_angle == 0:
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
                if self.last_cones_distances is not None and len(self.last_cones_distances) > best and self.last_cones_distances[best] is not None:
                    if ((self.last_cones_distances[best] < self.report_dist or y1 < 0.1)
                            and self.report_start_time is None):
                        print(self.time, 'SCANNING PERSON started', y1, y2, self.last_cones_distances[best])
                        self.report_start_time = self.time
                        report = {
                            'lat' : self.last_position[0] if self.last_position is not None else None,
                            'lon': self.last_position[1] if self.last_position is not None else None,
                        }
                        self.is_scanning_person = True
                        self.publish('scanning_person', self.is_scanning_person)
                        self.publish('report_latlon', report)
                        self.publish('play_sound', DTC_QUERY_SOUND)
            else:
                if self.tracking_start_time is not None:
                    print(self.time, f'Lost track {self.time - self.tracking_start_time}')
                    self.tracking_start_time = None

            # GPS hacking
            if self.last_position is not None and self.gps_heading is not None and self.closest_waypoint_dist is not None:
                if self.closest_waypoint_dist > 5:
                    to_waypoint = geo_angle(latlon2xy(*self.last_position), latlon2xy(*self.waypoints[self.closest_waypoint]))
                    diff_angle = normalizeAnglePIPI(to_waypoint - self.gps_heading)
                    if steering_angle == 0:
                        steering_angle = math.copysign(math.radians(10), diff_angle)
                else:
                    if self.geofence is not None:
                        # remove the closest waypoint and generate new one
                        self.waypoints = [self.geofence.get_random_inner_waypoint()]
                        print('New waypoints', self.waypoints)

        if steering_angle is None:
            # no way to go! -> STOP and look around
            speed, steering_angle = 0, 0
            self.look_around = True
        if self.verbose:
            print(speed, steering_angle)
        if not self.look_around:
            # TODO refactoring - otherwise lookaround conflicts with other commands
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
        if utc_time is not None:
            matty_name = normalize_matty_name(self.system_name)
            if int(round(float(utc_time))) % 10 == int(matty_name[-1]):
                # per system every 10s
                empty_report = DTCReport(matty_name, lat, lon)
                self.publish('lora_latlon', pack_data(empty_report) + b'\n')  # extra '\n' required by crypt
        if lat is not None and lon is not None:
            border_dist = None
            if self.geofence is not None:
                border_dist = self.geofence.border_dist((lat, lon))
            if int(self.time.total_seconds()) % 10 == 0:
                print(self.time, 'GPS', lat, lon, border_dist, self.waypoints)
            p = lat, lon
            if self.verbose:
                self.debug_arr.append((self.time, p, self.gps_heading, self.yaw))
            best_i, best_dist  = None, None
            for i, waypoint in enumerate(self.waypoints):
                dist = geo_length(latlon2xy(*p), latlon2xy(*waypoint))
                if best_dist is None or best_dist > dist:
                    best_i = i
                    best_dist = dist
            if self.closest_waypoint != best_i:
                print(f'{self.time} ----- Switching to {best_i} at {best_dist} -----')
                for i, waypoint in enumerate(self.waypoints):
                    dist = geo_length(latlon2xy(*p), latlon2xy(*waypoint))
                    print(i, waypoint, dist)
                print(f'{self.time} ----------------------')
            self.last_position = p
            self.closest_waypoint = best_i
            self.closest_waypoint_dist = best_dist

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
            self.publish('play_sound', self.system_name + 'ready')
            self.status_ready = True

        if self.last_detections is None:
            return

        def frameNorm(w, h, bbox):
            normVals = np.full(len(bbox), w)
            normVals[::2] = h
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        self.last_cones_distances = []
        for detection in self.last_detections:
            # ['cone', 0.92236328125, [0.42129743099212646, -0.0010452494025230408, 0.4836755692958832, 0.1296510100364685]]
            w, h = 640, 400
            a, b, c, d = frameNorm(h, h, detection[2]).tolist()
            name, x, y, width, height = detection[0], a + (w - h) // 2, b, c - a, d - b

            if name == 'person':
                cone_depth = data[y:y+height, x:x+width]
                mask = cone_depth > 0
                if mask.max():
                    dist = np.percentile(cone_depth[mask], 50) / 1000
                else:
                    dist = None
                self.last_cones_distances.append(dist)

        if self.verbose:
            print(f'{self.time} cone at {self.last_cones_distances}')


    def on_orientation_list(self, data):
        pass

    def on_rotation(self, data):
        yaw, pitch, roll = data
        self.yaw = math.radians(yaw/100.0)
        self.gps_heading = self.yaw  # replace former heading estimation from multiple GPS points

    def action_look_around(self):
        """
        turn in place 45 deg left then 45 deg right and accumulate scan
        :return: big scan
        """
        print('--------- ACTION LOOK AROUND ---------')
        MDEG_STEP = 200
        big_scan = []
        for start, end, step in [(0, 4500, MDEG_STEP), (4500, -4500, -MDEG_STEP)]:
            for angle in range(start, end, step):
                # node dependency on pose2d update rate
                while self.update() != 'pose2d':
                    pass
                steering_angle_rad = math.radians(angle/100)
                self.send_speed_cmd(0, steering_angle_rad)
            big_scan.extend(self.scan)
        print('left, right:', big_scan)
        print('--------- END OF LOOK AROUND ---------')
        return big_scan

    def action_go(self, speed, steering_angle, duration):
        print(f'--------- ACTION GO {speed}, {steering_angle}, {duration} ---------')
        start_time = self.time
        while start_time + duration > self.time:
            if self.update() == 'pose2d':
                self.send_speed_cmd(speed, steering_angle)
        print('--------- END OF GO ---------')

    def action_replay(self, cmd_list, reverse=True):
        """
        replay history in backward order
        """
        print(f'--------- ACTION REPLAY {len(cmd_list)} ---------')
        assert reverse  # direct replay not supported yet
        for speed, steering_angle in reversed(cmd_list):
            while True:
                if self.update() == 'pose2d':
                    self.send_speed_cmd(-speed, steering_angle)
                    break
        print('--------- END OF REPLAY ---------')

    def run(self):
        """
        override default Node.run()
        :return:
        """
        try:
            while True:
                if self.look_around:
                    big_scan = self.action_look_around()
                    steering_angle = self.get_direction(big_scan)
                    if steering_angle is not None:
                        self.action_go(speed=self.max_speed/2, steering_angle=steering_angle,
                                       duration=timedelta(seconds=1.0))
                    else:
                        backup_history = self.cmd_history[:]
                        self.action_replay(backup_history, reverse=True)
                    self.look_around = False
                else:
                    self.update()
        except BusShutdownException:
            pass

    def draw(self):
        from matplotlib.patches import Circle
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        fig, ax = plt.subplots()
        for y_center, x_center in self.waypoints:
            ax.scatter(x_center, y_center)

        x_center = [p[1] for (t, p, _, _) in self.debug_arr]
        y_center = [p[0] for (t, p, _, _) in self.debug_arr]
        gps_heading = [a[2] for a in self.debug_arr]
        yaw = [a[3] for a in self.debug_arr]
        ax.scatter(x_center, y_center)

        scale = 0.00001
        lines = []
        for x, y, h in zip(x_center, y_center, gps_heading):
            if h is None:
                continue
            x2 = math.cos(h)*scale + x
            y2 = math.sin(h)*scale + y
            lines.append(((x, y), (x2, y2)))
        lc = LineCollection(lines, colors='black', linewidths=2)
        ax.add_collection(lc)

        lines = []
        for x, y, h in zip(x_center, y_center, yaw):
            if h is None:
                continue
            x2 = math.cos(h)*scale + x
            y2 = math.sin(h)*scale + y
            lines.append(((x, y), (x2, y2)))
        lc = LineCollection(lines, colors='red', linewidths=2)
        ax.add_collection(lc)

        radius = 0.0001
        for c in self.waypoints + self.debug_all_waypoints:
            circle = Circle([c[1], c[0]], radius, fill=False, edgecolor='r', linestyle='--')
            ax.add_patch(circle)
            ax.set_aspect('equal')
            ax.scatter(x_center, y_center)

        if self.geofence is not None:
            pts = self.geofence.polygon_coords_lon_lat
            x_coords = [lon for lon, lat in pts]  # x-coordinates of vertices, closing the loop
            y_coords = [lat for lon, lat in pts]  # y-coordinates of vertices, closing the loop
            ax.fill(x_coords, y_coords, color='blue', alpha=0.2, edgecolor='black')

        plt.legend()
        plt.title('Waypoints')
        plt.grid(True, linestyle='--', color='gray', alpha=0.6)
        plt.show()

# vim: expandtab sw=4 ts=4
