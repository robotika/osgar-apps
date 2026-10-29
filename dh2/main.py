"""DH2 ver0: depth-guided roaming with a preferred turn direction.

No reversing, GPS navigation, or guarantee of complete area coverage.
"""

import math

import numpy as np
from osgar.exceptions import EmergencyStopException
from osgar.node import Node


class ConeDetector(Node):
    """Stationary cone-detection test; no motion output or range estimation."""

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cones')
        self.raise_exception_on_stop = config.get('terminate_on_stop', False)

    def on_emergency_stop(self, data):
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def run(self):
        try:
            super().run()
        except EmergencyStopException:
            self.request_stop()

    def on_detections(self, data):
        cones = [d for d in data if d[0] == 'cone']
        self.publish('cones', cones)
        if cones:
            print(self.time, 'cones:', cones)


class ConeApproach(ConeDetector):
    """Slow image-based approach for collecting samples, without depth ranging."""

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'status')
        self.max_speed = config.get('max_speed', 0.1)
        self.stop_height = config.get('stop_height', 0.65)
        self.max_travel = config.get('max_travel', 2.0)
        self.target_timeout = config.get('target_timeout', 0.5)
        self.min_confidence = config.get('min_confidence', 0.6)
        self.target = None
        self.target_time = None
        self.hits = 0
        self.emergency_stop = True
        self.done = False
        self.pose = None
        self.travelled = 0
        self.moving = False
        self.last_status = None

    def command(self, speed, angle, reason):
        if self.done or self.emergency_stop:
            speed, angle = 0, 0
        self.moving = speed > 0
        self.publish('desired_steering', [round(speed * 1000), round(math.degrees(angle) * 100)])
        if reason != self.last_status:
            self.publish('status', reason)
            print(self.time, reason)
            self.last_status = reason

    def on_emergency_stop(self, data):
        self.emergency_stop = bool(data)
        if data:
            self.command(0, 0, 'emergency stop')
        super().on_emergency_stop(data)

    def on_bumpers_front(self, data):
        if data:
            self.done = True
            self.command(0, 0, 'bumper: restart required')

    on_bumpers_rear = on_bumpers_front

    def on_pose2d(self, data):
        point = (data[0] / 1000, data[1] / 1000)
        if self.pose is not None and self.moving:
            self.travelled += math.dist(point, self.pose)
        self.pose = point
        if self.travelled >= self.max_travel:
            self.done = True
            self.command(0, 0, 'travel limit: restart required')

    def on_detections(self, data):
        super().on_detections(data)
        candidates = []
        for label, confidence, bbox in data:
            if label != 'cone' or confidence < self.min_confidence or len(bbox) != 4:
                continue
            if not np.isfinite(bbox).all():
                continue
            x1, y1, x2, y2 = np.clip(bbox, 0, 1)
            if x1 < x2 and y1 < y2:
                candidates.append((y2 - y1, (x1 + x2) / 2))
        previous = self.target
        elapsed = None if self.target_time is None else (self.time - self.target_time).total_seconds()
        self.target = max(candidates, default=None)
        self.target_time = self.time
        if self.target is None:
            self.hits = 0
            self.command(0, 0, 'no cone')
        else:
            consistent = (previous is not None and elapsed <= self.target_timeout
                          and abs(previous[1] - self.target[1]) < 0.15)
            self.hits = self.hits + 1 if consistent else 1
            if self.target[0] >= self.stop_height:
                self.done = True
                self.command(0, 0, 'cone large enough: restart required')

    def on_tick(self, data):
        if self.done or self.emergency_stop:
            self.command(0, 0, 'stopped')
        elif (self.target is None or self.hits < 3 or self.pose is None
              or (self.time - self.target_time).total_seconds() > self.target_timeout):
            self.command(0, 0, 'waiting for cone')
        else:
            height, center = self.target
            angle = max(-math.radians(25), min(math.radians(25), (0.5 - center) * 2))
            # Slow down as the cone grows in the image; steering left is positive.
            speed = self.max_speed * max(0.3, 1 - height / self.stop_height)
            self.command(speed, angle, 'approaching cone')


class Castle(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering', 'status', 'defender', 'people')
        self.raise_exception_on_stop = config.get('terminate_on_stop', False)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX') or ''
        self.side = config.get(self.system_name + 'search_side', config.get('search_side', 'left'))
        if self.side not in ('left', 'right'):
            raise ValueError('search_side must be left or right')
        self.sign = 1 if self.side == 'left' else -1
        self.speed = config.get('max_speed', 0.2)
        self.clearance = config.get('clearance', 0.8)
        self.target_timeout = config.get('target_timeout', 0.7)
        self.confirm_time = config.get('confirm_time', 2)
        self.report_distance = config.get('report_distance', 1.0)
        self.approach = config.get('approach', True)
        self.geometry_verified = config.get('geometry_verified', False)
        self.fov = math.radians(config.get('horizontal_fov_deg', 69))
        self.depth = None
        self.pose = None
        self.cones = []
        self.cone_time = None
        self.stopped = False
        self.front = None
        self.rear = None
        self.confirm_started = None
        self.confirm_position = None
        self.confirm_pose = None
        self.leave_started = None
        self.visited = []
        self.last_status = None

    def command(self, speed, angle, reason):
        # Every path to motion passes the same persistent interlocks.
        if self.stopped or self.front is not False or self.rear is not False:
            speed, angle, reason = 0, 0, 'stop/bumper interlock'
        self.publish('desired_steering', [round(speed * 1000), round(math.degrees(angle) * 100)])
        if reason != self.last_status:
            self.publish('status', reason)
            self.last_status = reason

    def on_emergency_stop(self, data):
        # Latch until process restart, including after physical release.
        if data:
            self.stopped = True
            self.command(0, 0, 'emergency stop')
            if self.raise_exception_on_stop:
                raise EmergencyStopException()

    def run(self):
        try:
            super().run()
        except EmergencyStopException:
            self.request_stop()

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

    def on_depth(self, data):
        self.depth = np.asarray(data)

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
        if (not self.geometry_verified or self.cone_time is None
                or (self.time - self.cone_time).total_seconds() > self.target_timeout):
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
        scan = self.clearance_scan()
        if (self.stopped or self.front is not False or self.rear is not False
                or self.pose is None or scan is None):
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
