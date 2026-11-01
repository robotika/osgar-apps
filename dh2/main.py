"""DH2 cone exploration controller."""

import math

import numpy as np
from osgar.exceptions import EmergencyStopException
from osgar.node import Node


class ConeExplore(Node):
    """Scan using zero-speed steering, advance a short step, and scan again."""

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cones')
        self.raise_exception_on_stop = config.get('terminate_on_stop', False)
        bus.register('desired_steering', 'status')
        self.stop_height = config.get('stop_height', 0.65)
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
        self.max_speed = min(config.get('max_speed', 0.3), 0.5)
        self.min_speed = min(config.get('min_speed', 0.15), self.max_speed)
        self.explore_speed = min(config.get('explore_speed', 0.2), self.max_speed)
        self.scan_dwell = config.get('scan_dwell', 1.5)
        self.scan_settle = config.get('scan_settle', 0.5)
        if not 0 <= self.scan_settle < self.scan_dwell:
            raise ValueError('scan_settle must be smaller than scan_dwell')
        self.step_distance = config.get('step_distance', 0.5)
        self.scan_angles = [math.radians(a) for a in (0, 30, -30, 0)]
        self.state = 'scan'
        self.scan_index = 0
        self.scan_started = None
        self.step_started_at = 0
        self.steps = 0
        self.look_angle = 0
        self.last_steered = None
        self.max_travel = config.get('max_travel', math.inf)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX') or ''
        self.required_cones = config.get('required_cones', 4)
        self.visit_wait = config.get('visit_wait', 3.0)
        self.repeat_radius = config.get('repeat_radius', 1.0)
        self.visited = []
        self.heading = 0
        self.front_pressed = False
        self.rear_pressed = False
        self.wait_started = None
        self.wait_pose = None
        self.wait_target = None
        self.ignore_until_travel = 0
        self.recovery_angles = [math.radians(a) for a in (30, -30, 0)]
        self.recovery_options = []
        self.recovery_cause = None
        self.recovery_count = 0
        self.escape_angle = 0
        self.escape_started = None
        self.depth_time = None
        self.allow_reverse = config.get('allow_reverse', False)
        self.reverse_distance = min(config.get('reverse_distance', 0.3), 0.3)
        self.reverse_speed = min(config.get('reverse_speed', 0.1), 0.1)
        if self.reverse_distance <= 0 or self.reverse_speed <= 0:
            raise ValueError('reverse distance and speed must be positive')
        self.reverse_used = False
        self.reverse_started = None
        self.obstacle_stop_dist = config.get('obstacle_stop_dist', 0.0)
        self.obstacle_resume_dist = config.get('obstacle_resume_dist', self.obstacle_stop_dist + 0.3)
        self.obstacle_min_pixels = config.get('obstacle_min_pixels', 20)
        if self.obstacle_min_pixels < 1:
            raise ValueError('obstacle_min_pixels must be positive')
        self.obstacle_blocked = True
        self.obstacle_distance = None
        bus.register('clearance', 'defender')


    def on_emergency_stop(self, data):
        self.emergency_stop = bool(data)
        if data:
            self.command(0, 0, 'emergency stop')
        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()


    def run(self):
        try:
            super().run()
        except EmergencyStopException:
            self.request_stop()


    def publish_cones(self, data):
        cones = [d for d in data if d[0] == 'cone']
        self.publish('cones', cones)
        if cones:
            print(self.time, 'cones:', cones)


    def command(self, speed, angle, reason):
        if speed < 0 and self.rear_pressed:
            speed = 0
            reason = 'rear bumper: reverse motion stopped'
        if speed > 0 and self.front_pressed:
            speed = 0
            reason = 'front bumper: forward motion stopped'
        if speed > 0 and self.obstacle_stop_dist > 0 and self.obstacle_blocked:
            speed = 0
            reason = 'obstacle or initial depth pending: forward motion stopped'
        if not self.done and not self.emergency_stop:
            self.look_angle = angle
        if self.done or self.emergency_stop:
            speed, angle = 0, 0
        self.moving = speed != 0
        self.publish('desired_steering', [round(speed * 1000), round(math.degrees(angle) * 100)])
        if reason != self.last_status:
            self.publish('status', reason)
            print(self.time, reason)
            self.last_status = reason


    def on_pose2d(self, data):
        self.heading = math.radians(data[2] / 100)
        point = (data[0] / 1000, data[1] / 1000)
        if self.pose is not None and self.moving:
            self.travelled += math.dist(point, self.pose)
        self.pose = point
        if self.travelled >= self.max_travel:
            self.done = True
            self.command(0, 0, 'travel limit: restart required')

        if self.state == 'reverse' and self.travelled - self.step_started_at >= self.reverse_distance:
            self.start_recovery(self.recovery_cause)

    def on_bumpers_front(self, data):
        self.front_pressed = bool(data)
        if data:
            self.start_recovery('front bumper')

    def on_bumpers_rear(self, data):
        self.rear_pressed = bool(data)
        if data:
            self.start_recovery('rear bumper')

    def target_position(self, target):
        # Rough local association only: image-size stop is assumed to be about 1 m.
        height, center = target
        distance = self.stop_height / height
        bearing = self.heading + self.look_angle + (0.5 - center) * math.radians(45)
        return (self.pose[0] + distance * math.cos(bearing),
                self.pose[1] + distance * math.sin(bearing))

    def select_target(self, candidates):
        if self.pose is None or self.travelled < self.ignore_until_travel:
            return None
        return max((target for target in candidates
                    if not any(math.dist(self.target_position(target), old) < self.repeat_radius
                               for old in self.visited)), default=None)

    def handle_close_cone(self):
        if self.hits >= 3 and self.state != 'wait' and abs(self.target[1] - 0.5) < 0.2:
            self.state = 'wait'
            self.wait_started = self.time
            self.wait_pose = self.pose
            self.wait_target = self.target_position(self.target)
            self.command(0, self.look_angle, 'waiting 3s at cone')

    def start_recovery(self, cause):
        self.state = 'recover_scan'
        self.recovery_cause = cause
        self.recovery_count += 1
        self.recovery_options = []
        self.scan_index = 0
        self.scan_started = self.time
        self.target = None
        self.hits = 0
        self.command(0, self.recovery_angles[0], 'recovery scan: ' + cause)

    def recovery_tick(self):
        if self.state == 'recover_scan':
            angle = self.recovery_angles[self.scan_index]
            self.command(0, angle, 'recovery scan: ' + self.recovery_cause)
            if (self.time - self.scan_started).total_seconds() < self.scan_dwell:
                return
            if (self.depth_time is not None
                    and (self.depth_time - self.scan_started).total_seconds() >= self.scan_settle
                    and not self.obstacle_blocked and not self.front_pressed
                    and (self.recovery_cause != 'cone visited' or abs(angle) > 0.1)):
                self.recovery_options.append((angle, self.obstacle_distance))
            self.scan_index += 1
            self.scan_started = self.time
            if self.scan_index < len(self.recovery_angles):
                self.command(0, self.recovery_angles[self.scan_index], 'recovery scan: ' + self.recovery_cause)
                return
            if not self.recovery_options:
                if self.allow_reverse and not self.reverse_used and not self.rear_pressed:
                    self.reverse_used = True
                    self.reverse_started = self.time
                    self.step_started_at = self.travelled
                    self.state = 'reverse'
                    self.command(-self.reverse_speed, 0, 'reversing for recovery')
                    return
                self.start_recovery(self.recovery_cause)
                return
            # Prefer more measured clearance; alternate the tie preference on retries.
            preferred_sign = 1 if self.recovery_count % 2 else -1
            self.escape_angle, _ = max(self.recovery_options,
                                      key=lambda item: (math.inf if item[1] is None else item[1],
                                                        item[0] * preferred_sign))
            self.state = 'recover_orient'
            self.command(0, self.escape_angle, 'orienting toward recovery path')
        elif self.state == 'recover_orient':
            self.command(0, self.escape_angle, 'orienting toward recovery path')
            if (self.time - self.scan_started).total_seconds() < self.scan_settle:
                return
            if (self.depth_time is None
                    or (self.depth_time - self.scan_started).total_seconds() < self.scan_settle):
                return
            if self.obstacle_blocked or self.front_pressed:
                self.start_recovery(self.recovery_cause)
                return
            self.state = 'escape'
            self.step_started_at = self.travelled
            self.escape_started = self.time
        elif self.state == 'reverse':
            if (self.rear_pressed or self.travelled - self.step_started_at >= self.reverse_distance
                    or (self.time - self.reverse_started).total_seconds()
                    >= self.reverse_distance / self.reverse_speed + 1):
                self.start_recovery(self.recovery_cause)
            else:
                self.command(-self.reverse_speed, 0, 'reversing for recovery')
        else:  # escape: keep looking along the departure arc, ignoring cone attraction
            if self.obstacle_blocked or self.front_pressed:
                self.start_recovery(self.recovery_cause)
            elif self.travelled - self.step_started_at >= self.step_distance:
                self.reverse_used = False
                self.start_scan()
            elif (self.time - self.escape_started).total_seconds() >= 6:
                self.start_recovery(self.recovery_cause)
            else:
                self.command(self.explore_speed, self.escape_angle, 'departing along recovery path')

    def on_depth(self, data):
        # Left-camera-aligned depth, in mm; independent of the RGB detector crop.
        depth = np.asarray(data)
        self.depth_time = self.time
        assert depth.ndim == 2 and min(depth.shape) >= 10, depth.shape
        h, w = depth.shape
        # Stay above image center to reduce nearby ground/grass at the low mounting height.
        roi = depth[int(h * 0.3):int(h * 0.5), int(w * 0.3):int(w * 0.7)]
        valid = roi[(roi > 0) & np.isfinite(roi)]
        fraction = valid.size / roi.size
        if valid.size < self.obstacle_min_pixels:
            self.obstacle_distance = None
            self.obstacle_blocked = False
        else:
            # A fixed number of close pixels is evidence; zeros and isolated speckles are not.
            index = self.obstacle_min_pixels - 1
            self.obstacle_distance = float(np.partition(valid, index)[index]) / 1000
            if self.obstacle_blocked:
                self.obstacle_blocked = self.obstacle_distance < self.obstacle_resume_dist
            else:
                self.obstacle_blocked = self.obstacle_distance <= self.obstacle_stop_dist
        self.publish('clearance', {'distance': self.obstacle_distance, 'valid_fraction': fraction,
                                   'blocked': self.obstacle_blocked})
        if (self.obstacle_stop_dist > 0 and self.obstacle_blocked and self.moving
                and self.state != 'reverse'):
            self.command(0, self.look_angle, 'obstacle: forward motion stopped')

    def handle_target_loss(self):
        if self.state == 'approach':
            self.command(0, self.look_angle, 'looking for cone')

    def on_detections(self, data):
        if (self.state in ('recover_orient', 'escape', 'reverse')
                or (self.state == 'recover_scan' and self.recovery_cause == 'cone visited')):
            self.publish_cones(data)
            self.target_time = self.time
            return
        # Ignore target acquisition while articulation is settling at a scan pose.
        if (self.state in ('scan', 'recover_scan') and (self.scan_started is None
                or (self.time - self.scan_started).total_seconds() < self.scan_settle)):
            self.publish_cones(data)
            self.target = None
            self.hits = 0
            self.target_time = self.time
            return
        self.publish_cones(data)
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
        self.target = self.select_target(candidates)
        self.target_time = self.time
        if self.target is None:
            self.hits = 0
            self.handle_target_loss()
        else:
            consistent = (previous is not None and elapsed <= self.target_timeout
                          and abs(previous[1] - self.target[1]) < 0.15)
            self.hits = self.hits + 1 if consistent else 1
            if self.target[0] >= self.stop_height:
                self.handle_close_cone()


    def start_scan(self):
        self.state = 'scan'
        self.scan_index = 0
        self.scan_started = self.time
        self.target = None
        self.hits = 0
        self.command(0, self.scan_angles[0], 'scanning')

    def on_tick(self, data):
        if self.done or self.emergency_stop:
            self.command(0, 0, 'stopped')
            return
        if self.pose is None or self.target_time is None:
            self.command(0, 0, 'waiting for initial observations')
            return
        if self.state in ('recover_scan', 'recover_orient', 'escape', 'reverse'):
            self.recovery_tick()
            return
        if self.state == 'wait':
            visible = (self.target is not None and self.target[0] >= 0.9 * self.stop_height
                       and (self.time - self.target_time).total_seconds() <= self.target_timeout
                       and math.dist(self.target_position(self.target), self.wait_target) < 0.5)
            if not visible:
                self.start_scan()
                return
            if math.dist(self.pose, self.wait_pose) > 0.1:
                self.wait_started = self.time
                self.wait_pose = self.pose
            self.command(0, self.look_angle, 'waiting 3s at cone')
            if (self.time - self.wait_started).total_seconds() >= self.visit_wait:
                self.visited.append(self.wait_target)
                report = {'id': len(self.visited), 'robot': self.system_name,
                          'position': list(self.wait_target), 'stop_pose': list(self.pose),
                          'detection_time': self.target_time.total_seconds(),
                          'wait_sec': (self.time - self.wait_started).total_seconds()}
                self.publish('defender', report)
                print(self.time, 'defender reported:', report)
                self.ignore_until_travel = self.travelled + 0.75
                if len(self.visited) >= self.required_cones:
                    self.done = True
                    self.command(0, 0, 'all defenders reported')
                else:
                    self.start_recovery('cone visited')
            return
        if (self.state in ('approach', 'advance') and self.obstacle_stop_dist > 0
                and self.depth_time is not None and (self.obstacle_blocked or self.front_pressed)):
            self.start_recovery('obstacle')
            return
        acquired = (self.target is not None and self.hits >= 3
                    and (self.time - self.target_time).total_seconds() <= self.target_timeout)
        if acquired:
            self.state = 'approach'
            height, center = self.target
            if self.last_steered != self.target_time:
                # Keep the articulation that found the cone, then correct incrementally.
                # Resetting steering to zero here would move a side target out of view.
                self.look_angle = max(-math.radians(35), min(math.radians(35),
                                                           self.look_angle + (0.5 - center) * 0.5))
                self.last_steered = self.target_time
            if height >= self.stop_height:
                self.command(0, self.look_angle, 'centering near cone')
                return
            speed = max(self.min_speed, self.max_speed * (1 - height / self.stop_height))
            self.command(speed, self.look_angle, 'approaching cone')
        elif self.state == 'approach':
            self.start_scan()
        elif self.state == 'scan':
            if self.scan_started is None:
                self.start_scan()
            elif (self.time - self.scan_started).total_seconds() >= self.scan_dwell:
                self.scan_index += 1
                self.scan_started = self.time
                self.target = None
                self.hits = 0
                if self.scan_index == len(self.scan_angles):
                    self.state = 'advance'
                    self.step_started_at = self.travelled
                    self.steps += 1
                else:
                    self.command(0, self.scan_angles[self.scan_index], 'scanning')
            else:
                self.command(0, self.scan_angles[self.scan_index], 'scanning')
        elif self.state == 'advance':
            if self.travelled - self.step_started_at >= self.step_distance:
                self.start_scan()
            else:
                # First step straight, then alternating arcs; no assigned search side.
                angle = 0 if self.steps == 1 else math.radians(20 if self.steps % 2 == 0 else -20)
                self.command(self.explore_speed, angle, 'advancing to next scan')
