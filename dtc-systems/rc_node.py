"""
    Node for safety remote control
"""
import numpy as np
from osgar.node import Node


def nearest_obstacle(scan):
    scan = np.asarray(scan[3*45:3*225])
    scan[scan < 100] = 20000
    return np.min(scan)


class RcNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed")
        self.bus = bus
        self.verbose = False
        self.front_obstacle = 0
        self.back_obstacle = 0
        self.limit = 350  # mm


    def update(self):
        dt, channel, data = self.listen()
        self.time = dt
        if channel == "scan":
            self.front_obstacle = nearest_obstacle(data)
        if channel == "scan_back":
            self.back_obstacle = nearest_obstacle(data)
        if channel == "desired_speed":
            speed, angular = data
            if speed > 0 and self.front_obstacle > self.limit:
                self.publish("desired_speed", data)
            elif speed < 0 and self.back_obstacle > self.limit:
                self.publish("desired_speed", data)
            else:
                self.publish("desired_speed", [0, 0])
