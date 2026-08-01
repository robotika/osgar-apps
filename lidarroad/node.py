import numpy as np

from osgar.node import Node
from lidarroad.lidarroad import analyze_scan


class LidarRoad(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')
        self.max_speed = config.get('max_speed', 0.1)

    def on_emergency_stop(self, data):
        pass

    def on_pose2d(self, data):
        pass

    def on_scan(self, data):
        assert len(data) == 1800, len(data)
        _, from_i, to_i = analyze_scan(data[450:-450], fast=True)
        direction_i = 450 + (from_i + to_i) // 2
        width = 100
        safe_dist = 1.0
        selection = np.array(data[direction_i-width:direction_i+width], dtype=int)
        mask = selection != 0
        if np.any(mask):
            near_obstacle = np.percentile(selection[mask], 5) / 1000.0
        else:
            near_obstacle = 0.0
        direction_deg = 360 * (900 - direction_i) / 1800
        if self.verbose:
            print(self.time, from_i, to_i, direction_deg, near_obstacle)
        speed = self.max_speed if near_obstacle < safe_dist else 0
        self.publish('desired_steering', [speed, int(direction_deg * 100)])
