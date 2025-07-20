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



class DARPATriageChallenge(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'scan')
        self.max_speed = config.get('max_speed', 0.2)
        self.turn_angle = config.get('turn_angle', 20)
        self.waypoints = config.get('waypoints', [])[1:]  # remove start
        self.debug_all_waypoints = config.get('waypoints', [])[:]
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)

        self.last_position = None
        self.verbose = False
        self.scan = None
        self.backup_start_time = None
        self.report_start_time = None

        self.last_detections = None
        self.last_cones_distances = None  # not available
        self.field_of_view = math.radians(45)  # TODO, should clipped camera image pass it?
        self.report_dist = config.get('report_dist', 2.0)

        self.closest_waypoint = None
        self.closest_waypoint_dist = None
        self.gps_heading = None
        self.debug_arr = []

        self.look_around = False  # in case of blocked path look left and right and pick direction

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
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
                print(self.time, "NO FREE SPACE", direction)
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
            if self.time - self.report_start_time < timedelta(seconds=3):
                self.send_speed_cmd(0, 0)
                return  # terminate without other driving
            elif self.time - self.report_start_time < timedelta(seconds=10):
                # ignore detections for a moment (10s)
                if self.closest_waypoint_dist is not None and self.closest_waypoint_dist < 10:
                    print(f'{self.time} REMOVING {self.closest_waypoint} dist={self.closest_waypoint_dist}')
                    self.waypoints = self.waypoints[:self.closest_waypoint] + self.waypoints[self.closest_waypoint + 1:]
                    self.closest_waypoint_dist = None
                    self.closest_waypoint = None
            else:
                self.report_start_time = None  # end of report

        if self.scan is None:
            # no depth data yet
            speed, steering_angle = 0, 0
        else:
            speed, steering_angle = self.max_speed, self.get_direction(self.scan)
            if self.last_detections is not None and len(self.last_detections) >= 1 and steering_angle == 0:
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
                    if self.last_cones_distances[best] < self.report_dist and self.report_start_time is None:
                        self.report_start_time = self.time

            # GPS hacking
            if self.last_position is not None and self.gps_heading is not None and self.closest_waypoint_dist is not None:
                if self.closest_waypoint_dist > 20:
                    to_waypoint = geo_angle(latlon2xy(*self.last_position), latlon2xy(*self.waypoints[self.closest_waypoint]))
                    diff_angle = normalizeAnglePIPI(to_waypoint - self.gps_heading)
                    if steering_angle == 0:
                        steering_angle = math.copysign(math.radians(10), diff_angle)

        if steering_angle is None:
            # no way to go! -> STOP and look around
            speed, steering_angle = 0, 0
            self.look_around = True
        if self.verbose:
            print(speed, steering_angle)
        self.send_speed_cmd(speed, steering_angle)

    def on_nmea_data(self, data):
        assert 'lat' in data, data
        assert 'lon' in data, data
        lat, lon = data['lat'], data['lon']
        if lat is not None and lon is not None:
            if int(self.time.total_seconds()) % 10 == 0:
                print(self.time, 'GPS', data['lat'], data['lon'])
            p = data['lat'], data['lon']
            if self.verbose:
                self.debug_arr.append((self.time, p))
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
            if self.last_position is not None:
                tmp = geo_angle(latlon2xy(*self.last_position), latlon2xy(*p))
                if tmp is not None:
                    self.last_position = p
                    self.gps_heading = tmp
            else:
                self.last_position = p
            self.closest_waypoint = best_i
            self.closest_waypoint_dist = best_dist

    def on_detections(self, data):
        self.last_detections = data[:]

    def on_depth(self, data):
        line = 400//2
        line_end = 400//2 + 30
        box_width = 160
        arr = []
        for index in range(0 , 641 - box_width, 20):
            mask = data[line:line_end, index:box_width + index] != 0
            if mask.max():
                dist = int(np.percentile( data[line:line_end, index:box_width + index][mask], 5))
            else:
                dist = 0
            arr.append(dist)
        self.publish('scan', arr)
        self.scan = arr

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

    def action_look_around(self):
        """
        turn in place 45 deg left then 45 deg right and accumulate scan
        :return: big scan
        """
        print('--------- ACTION LOOK AROUND ---------')
        MDEG_STEP = 100
        big_scan = []
        for start, end, step in [(0, 4500, MDEG_STEP), (4500, -4500, -MDEG_STEP), (-4500, 0, MDEG_STEP)]:
            for angle in range(start, end, step):
                # node dependency on pose2d update rate
                while self.update() != 'pose2d':
                    pass
                steering_angle_rad = math.radians(angle/100)
                self.send_speed_cmd(0, steering_angle_rad)
            big_scan.extend(self.scan)
        print('left, right, mid scan:', big_scan)
        print('--------- END OF LOOK AROUND ---------')
        return big_scan

    def run(self):
        """
        override default Node.run()
        :return:
        """
        try:
            while True:
                if self.look_around:
                    self.action_look_around()
                    self.look_around = False
                else:
                    self.update()
        except BusShutdownException:
            pass

    def draw(self):
        from matplotlib.patches import Circle
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        for x_center, y_center in self.waypoints:
            ax.scatter(x_center, y_center)

        x_center = [p[0] for (t, p) in self.debug_arr]
        y_center = [p[1] for (t, p) in self.debug_arr]
        ax.scatter(x_center, y_center)

        radius = 0.0001
        for c in self.waypoints + self.debug_all_waypoints:
            circle = Circle(c, radius, fill=False, edgecolor='r', linestyle='--')
            ax.add_patch(circle)
            ax.set_aspect('equal')
            ax.scatter(x_center, y_center)

        plt.legend()
        plt.title('Waypoints')
        plt.grid(True, linestyle='--', color='gray', alpha=0.6)
        plt.show()

# vim: expandtab sw=4 ts=4
