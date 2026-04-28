"""
  Dobyvání hradu (Conquer the Castle)
  Autonomous mobile robot competition
"""
import math

import numpy as np
from osgar.bus import BusShutdownException
from osgar.node import Node

from .geofence import Geofence  # We will copy/adapt geofence.py


class ConquerCastle(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.pose2d = [0, 0, 0]  # x, y, heading
        self.last_detections = None
        self.last_cones_distances = []
        self.verbose = config.get('verbose', False)
        self.geofence = None
        if 'geofence' in config:
            self.geofence = Geofence(config['geofence'])

        self.visited_cones = []  # list of (lat, lon)
        self.min_cone_dist_diff = config.get('min_cone_dist_diff', 2.0)  # meters
        self.wait_time = config.get('wait_time', 5.0)  # seconds
        self.max_speed = config.get('max_speed', 0.5)
        self.state = 'SEARCHING'
        self.state_start_time = None
        self.last_gps = None
        self.emergency_stop = False

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_pose2d(self, data):
        self.pose2d = data[:]

    def on_emergency_stop(self, data):
        self.emergency_stop = data

    def on_nmea_data(self, data):        # Basic GPS tracking (simplified, might need more robust parsing)
        if data.startswith('$GNGGA') or data.startswith('$GPGGA'):
            parts = data.split(',')
            if len(parts) > 4 and parts[2] and parts[4]:
                lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
                if parts[3] == 'S':
                    lat = -lat
                lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
                if parts[5] == 'W':
                    lon = -lon
                self.last_gps = (lat, lon)

    def on_detections(self, data):
        self.last_detections = data

    def on_depth(self, data):
        if self.last_detections is None:
            return

        w, h = 640, 400
        def frameNorm(w, h, bbox):
            normVals = np.full(len(bbox), w)
            normVals[::2] = h
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        self.last_cones_distances = []
        for detection in self.last_detections:
            a, b, c, d = frameNorm(h, h, detection[2]).tolist()
            _name, x, y, width, height = detection[0], a + (w - h) // 2, b, c - a, d - b

            cone_depth = data[y:y+height, x:x+width]
            mask = cone_depth > 0
            if mask.max():
                dist = np.percentile(cone_depth[mask], 50) / 1000
            else:
                dist = None
            self.last_cones_distances.append(dist)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler:
            handler(self.bus.listen()[1])

        # Main state machine logic
        self.run_logic()

    def run_logic(self):
        if self.state == 'SEARCHING':
            # 1. Check for cones
            if self.last_cones_distances:
                # TODO: Check if it is a new cone (GPS-based)
                # TODO: Filter by distance
                # self.state = 'APPROACHING'
                pass

            # 2. Stay within geofence
            if self.geofence and self.last_gps:
                dist = self.geofence.border_dist(self.last_gps)
                if dist < 1.0:
                    # Turn back or change direction
                    pass

        elif self.state == 'APPROACHING':
            pass

        elif self.state == 'WAITING':
            if self.time - self.state_start_time > self.wait_time:
                self.state = 'SEARCHING'

    def run(self):
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass

if __name__ == "__main__":
    from osgar.launcher import launch
    launch(ConquerCastle)
