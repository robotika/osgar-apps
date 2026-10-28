"""DH2 ver0: depth-guided roaming with a preferred turn direction.

No reversing, GPS navigation, or guarantee of complete area coverage.
"""

import math

import numpy as np
from osgar.node import Node


class Castle(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'status', 'defender', 'people')
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX') or ''
        self.side = config.get(self.system_name + 'search_side', config.get('search_side', 'left'))
        if self.side not in ('left', 'right'):
            raise ValueError('search_side must be left or right')
        self.sign = 1 if self.side == 'left' else -1
        self.speed = config.get('max_speed', 0.2)
        self.clearance = config.get('clearance', 0.8)
        self.timeout = config.get('sensor_timeout', 0.7)
        self.duration = config.get('duration', 300)
        self.confirm_time = config.get('confirm_time', 2)
        self.report_distance = config.get('report_distance', 1.0)
        self.approach = config.get('approach', True)
        self.geometry_verified = config.get('geometry_verified', False)
        self.fov = math.radians(config.get('horizontal_fov_deg', 69))
        self.depth = None
        self.depth_time = None
        self.pose = None
        self.pose_time = None
        self.cones = []
        self.cone_time = None
        self.stopped = False
        self.front = None
        self.rear = None
        self.started = None
        self.confirm_started = None
        self.confirm_position = None
        self.confirm_pose = None
        self.leave_started = None
        self.visited = []
        self.last_status = None

    def fresh(self, timestamp):
        return timestamp is not None and (self.time - timestamp).total_seconds() <= self.timeout

    def command(self, speed, angle, reason):
        # Every path to motion passes the same persistent interlocks.
        if self.stopped or self.front is not False or self.rear is not False:
            speed, angle, reason = 0, 0, 'stop/bumper interlock'
        elif not self.fresh(self.depth_time) or not self.fresh(self.pose_time):
            speed, angle, reason = 0, 0, 'waiting for fresh depth/pose'
        self.publish('desired_steering', [round(speed * 1000), round(math.degrees(angle) * 100)])
        if reason != self.last_status:
            self.publish('status', reason)
            self.last_status = reason

    def on_emergency_stop(self, data):
        # Latch until process restart, including after physical release.
        if data:
            self.stopped = True
            self.command(0, 0, 'emergency stop')

    def on_bumpers_front(self, data):
        self.front = bool(data)
        if data:
            self.stopped = True
            self.command(0, 0, 'front bumper')

    def on_bumpers_rear(self, data):
        self.rear = bool(data)
        if data:
            self.stopped = True
            self.command(0, 0, 'rear bumper')

    def on_pose2d(self, data):
        x, y, heading = data
        self.pose = (x / 1000, y / 1000, math.radians(heading / 100))
        self.pose_time = self.time

    def on_depth(self, data):
        self.depth = np.asarray(data)
        self.depth_time = self.time

    def on_detections(self, data):
        # Both networks share this stream. Empty YOLO messages must not erase cones.
        cones = [d for d in data if d[0] == 'cone' and d[1] >= 0.6]
        if cones:
            self.cones, self.cone_time = cones, self.time
        people = [d for d in data if d[0] == 'person' and d[1] >= 0.5]
        if people:
            self.publish('people', people)

    def clearance_scan(self):
        if self.depth is None or self.depth.ndim != 2 or min(self.depth.shape) < 10:
            return None
        h, w = self.depth.shape
        # Image order: left, center, right. Wide overlapping corridors.
        scan = []
        for a, b in ((0.1, 0.55), (0.3, 0.7), (0.45, 0.9)):
            roi = self.depth[int(h * 0.4):int(h * 0.65), int(w * a):int(w * b)]
            valid = roi[roi > 0]
            # Missing depth is unknown, never open space.
            scan.append(float(np.percentile(valid, 5)) / 1000
                        if valid.size >= roi.size * 0.7 else 0)
        return scan

    def target(self):
        if not self.geometry_verified or not self.fresh(self.cone_time):
            return None
        h, w = self.depth.shape
        candidates = []
        for _, confidence, bbox in self.cones:
            if len(bbox) != 4 or not np.isfinite(bbox).all():
                continue
            x1, y1, x2, y2 = np.clip(bbox, 0, 1)
            # Square detector crop centered in RGB-aligned depth. Verify on robot.
            size = min(h, w)
            left = int((w - size) / 2 + x1 * size)
            right = int((w - size) / 2 + x2 * size)
            top = int((h - size) / 2 + y1 * size)
            bottom = int((h - size) / 2 + y2 * size)
            roi = self.depth[top:bottom, left:right]
            valid = roi[roi > 0]
            if not roi.size or valid.size < roi.size * 0.5:
                continue
            distance = float(np.median(valid)) / 1000
            angle = math.atan((0.5 - (left + right) / (2 * w)) * 2 * math.tan(self.fov / 2))
            x, y, yaw = self.pose
            position = (x + distance * math.cos(yaw + angle), y + distance * math.sin(yaw + angle))
            if any(math.dist(position, old) < 2 for old in self.visited):
                continue
            candidates.append((distance, angle, position, confidence))
        return min(candidates, default=None)

    def on_tick(self, data):
        if self.started is None:
            self.started = self.time
        if (self.time - self.started).total_seconds() >= self.duration:
            self.stopped = True
        scan = self.clearance_scan()
        if (self.stopped or self.front is not False or self.rear is not False
                or not self.fresh(self.depth_time) or not self.fresh(self.pose_time) or scan is None):
            self.confirm_started = None
            self.command(0, 0, 'stopped or missing sensors')
            return
        target = self.target()
        turn = math.radians(25)
        if self.leave_started is not None:
            if (self.time - self.leave_started).total_seconds() < 3:
                corridor = 0 if self.sign > 0 else 2
                if scan[1] > self.clearance and scan[corridor] > self.clearance:
                    self.command(self.speed / 2, self.sign * turn, 'leaving confirmed cone')
                else:
                    self.stopped = True
                    self.command(0, 0, 'blocked leaving cone: operator recovery required')
                return
            self.leave_started = None
        if target is not None and target[0] <= self.report_distance:
            distance, angle, position, confidence = target
            if (self.confirm_started is None or math.dist(position, self.confirm_position) > 0.75
                    or math.dist(self.pose[:2], self.confirm_pose) > 0.1):
                self.confirm_started, self.confirm_position = self.time, position
                self.confirm_pose = self.pose[:2]
            if (self.time - self.confirm_started).total_seconds() >= self.confirm_time:
                self.visited.append(position)
                self.publish('defender', {'local_id': len(self.visited), 'position': list(position),
                                         'range': distance, 'bearing': angle, 'confidence': confidence})
                self.confirm_started = None
                self.leave_started = self.time
            self.command(0, 0, 'confirming cone')
            return
        self.confirm_started = None
        preferred = 0 if self.sign > 0 else 2
        other = 2 - preferred
        if scan[1] > self.clearance:
            angle = 0
            if target is not None and self.approach:
                angle = max(-turn, min(turn, target[1]))
                # Steering toward a target still needs clearance in that corridor.
                if scan[0 if angle > 0 else 2] <= self.clearance:
                    angle = 0
            self.command(self.speed, angle, 'approaching' if angle else 'searching')
        elif scan[1] > self.clearance * 0.65 and scan[preferred] > self.clearance:
            self.command(self.speed / 2, self.sign * turn, 'preferred turn')
        elif scan[1] > self.clearance * 0.65 and scan[other] > self.clearance:
            self.command(self.speed / 2, -self.sign * turn, 'alternative turn')
        else:
            self.stopped = True
            self.command(0, 0, 'blocked: operator recovery required')
