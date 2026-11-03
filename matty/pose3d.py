"""Shadow-mode joint-centred Matty pose: planar translation and separate attitudes."""

import math
from bisect import bisect_left
from collections import deque

import numpy as np
from osgar.lib import quaternion as quat
from osgar.node import Node

from matty.odometry import validate_bounds, wheel_interval


def unit_quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Expected a finite nonzero XYZW quaternion')
    return (q / np.linalg.norm(q)).tolist()


def angle_difference(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def yaw_rotation(angle):
    return quat.from_axis_angle([0, 0, 1], angle)


def planar_step(increments, previous_front, front, previous_rear, rear,
                front_axle, rear_axle, track):
    """Solve planar joint displacement from four rolling and two lateral constraints.

    Wheel order is FL, FR, RL, RR. Attitudes enter through projected yaw only.
    Front/rear axles lie +front_axle/-rear_axle from the joint, in metres.
    Finite lever-arm changes include motion caused by stationary articulation.
    """
    matrix, target = [], []
    for ds, old, new, axle in [(increments[:2], previous_front, front, front_axle),
                                (increments[2:], previous_rear, rear, -rear_axle)]:
        change = angle_difference(new, old)
        mid = old + change / 2
        forward = np.array([math.cos(mid), math.sin(mid)])
        lateral = np.array([-forward[1], forward[0]])
        old_rotation = np.array([[math.cos(old), -math.sin(old)], [math.sin(old), math.cos(old)]])
        new_rotation = np.array([[math.cos(new), -math.sin(new)], [math.sin(new), math.cos(new)]])
        for distance, side in zip(ds, [track / 2, -track / 2]):
            lever_change = (new_rotation - old_rotation) @ [axle, side]
            matrix.append(forward)
            target.append(distance - forward @ lever_change)
        # Two wheels support each longitudinal axle constraint, so use matching weight.
        lever_change = (new_rotation - old_rotation) @ [axle, 0]
        matrix.append(math.sqrt(2) * lateral)
        target.append(-math.sqrt(2) * lateral @ lever_change)
    matrix, target = np.asarray(matrix), np.asarray(target)
    displacement, _, _, _ = np.linalg.lstsq(matrix, target, rcond=None)
    residual = matrix @ displacement - target
    return displacement, float(np.sqrt(np.mean(residual ** 2)))


class PoseEstimator:
    """Pure estimator, independent of message scheduling and camera clock conversion."""

    def __init__(self, config):
        self.wheel_order = config['wheel_order']
        if sorted(self.wheel_order) != [0, 1, 2, 3]:
            raise ValueError('wheel_order must be a permutation for FL, FR, RL, RR')
        self.front_axle = float(config['front_axle'])
        self.rear_axle = float(config['rear_axle'])
        self.track = float(config['track'])
        if not all(math.isfinite(v) and v > 0 for v in [self.front_axle, self.rear_axle, self.track]):
            raise ValueError('Axle distances and track must be positive and finite')
        self.housing_offset = np.asarray(config['housing_offset'], dtype=float)
        if self.housing_offset.shape != (3,) or not np.isfinite(self.housing_offset).all():
            raise ValueError('housing_offset must have three finite metre values')
        self.max_gap = config.get('max_gap', .5)
        self.max_wheel_speed = config.get('max_wheel_speed', 1.5)
        validate_bounds(self.max_gap, self.max_wheel_speed)
        self.position = np.zeros(3)
        self.previous = None
        self.front_alignment = self.rear_alignment = None
        self.continuous = True
        self.origin_time = None

    def update(self, timestamp, bundle, front, rear):
        flags, speed, joint_cdeg, counters = bundle
        front, rear = unit_quaternion(front), unit_quaternion(rear)
        quality = dict(data_time_s=timestamp, mode='planar_assumed_z', z_observed=False,
                       origin='central_joint_at_initialization', status_bits=flags,
                       reported_speed_mm_s=speed, joint_angle_cdeg=joint_cdeg,
                       integrated=False, reasons=[], model_rms_m=None)
        if self.front_alignment is None:
            # Initial yaw convention only; subsequent orientations use fixed world alignments.
            joint = math.radians(joint_cdeg / 100)
            self.front_alignment = yaw_rotation(joint / 2 - quat.heading(front))
            self.rear_alignment = yaw_rotation(-joint / 2 - quat.heading(rear))
            self.origin_time = timestamp
        front = unit_quaternion(quat.multiply(self.front_alignment, front))
        rear = unit_quaternion(quat.multiply(self.rear_alignment, rear))
        if self.previous is not None:
            old_time, old_bundle, old_front, old_rear = self.previous
            delta, reasons = wheel_interval(old_bundle[3], counters, timestamp - old_time,
                                            old_bundle[0], flags, max_gap=self.max_gap,
                                            max_wheel_speed=self.max_wheel_speed)
            quality['reasons'] = reasons
            if reasons:
                self.continuous = False  # Unknown travel is not silently declared zero.
            else:
                distances = np.asarray(delta)[self.wheel_order] / 1000
                displacement, rms = planar_step(distances, quat.heading(old_front), quat.heading(front),
                                                 quat.heading(old_rear), quat.heading(rear),
                                                 self.front_axle, self.rear_axle, self.track)
                self.position[:2] += displacement
                quality.update(integrated=True, model_rms_m=rms,
                               wheel_mean_m=float(np.mean(distances)), joint_step_xy=displacement.tolist())
        else:
            quality['reasons'] = ['initialization']
        self.previous = (timestamp, [flags, speed, joint_cdeg, list(counters)], front, rear)
        quality.update(translation_continuous=self.continuous, origin_time_s=self.origin_time,
                       projected_joint_yaw_residual_deg=math.degrees(angle_difference(
                           quat.heading(front) - quat.heading(rear), math.radians(joint_cdeg / 100))))
        position = self.position.tolist()
        housing = self.position + np.asarray(quat.rotate_vector(self.housing_offset.tolist(), front))
        return dict(pose3d=[position, front], rear_pose3d=[position.copy(), rear],
                    housing_pose3d=[housing.tolist(), front.copy()], pose_quality=quality)


class Attitudes:
    def __init__(self, max_gap=.3):
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError('max_attitude_gap must be positive and finite')
        self.samples = deque(maxlen=400)
        self.max_gap = max_gap

    def add(self, timestamp, q):
        q = unit_quaternion(q)
        if self.samples and timestamp <= self.samples[-1][0]:
            raise ValueError('Nonincreasing camera IMU acquisition time; restart/alignment review required')
        self.samples.append((timestamp, q))

    def at(self, timestamp):
        samples = list(self.samples)
        times = [t for t, _ in samples]
        i = bisect_left(times, timestamp)
        if i < len(samples) and times[i] == timestamp:
            return samples[i][1]
        if i == 0 or i == len(samples):
            return None
        t0, q0 = samples[i - 1]
        t1, q1 = samples[i]
        if t1 - t0 > self.max_gap:
            return None
        return unit_quaternion(quat.slerp(q0, q1, (timestamp - t0) / (t1 - t0)))


class MattyPose3D(Node):
    """Passive node; outputs never include motion commands.

    orientation_list: OAK fused batches; rotation: ESP OSGAR yaw/pitch/roll cdeg;
    odometry: bundled status/speed/joint/four wheel distances from the platform.
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose3d', 'rear_pose3d', 'housing_pose3d', 'pose_quality')
        self.model = PoseEstimator(config)
        self.front_from_fused = unit_quaternion(config['front_from_fused'])
        self.rear_from_esp = unit_quaternion(config['rear_from_esp'])
        self.attitudes = Attitudes(config.get('max_attitude_gap', .3))
        self.imu_offset = config.get('imu_time_offset')
        if self.imu_offset is not None and not math.isfinite(self.imu_offset):
            raise ValueError('imu_time_offset must be finite')
        self.clock_mode = 'configured_offset' if self.imu_offset is not None else 'first_batch_receipt_approximation'
        self.rear = None
        self.pending = deque()
        self.skipped = 0
        self.max_rear_age = config.get('max_rear_age', .15)
        if not math.isfinite(self.max_rear_age) or self.max_rear_age <= 0:
            raise ValueError('max_rear_age must be positive and finite')
        self.geometry_verified = bool(config.get('geometry_verified', False))
        self.geometry_id = config.get('geometry_id', 'unverified')

    def on_rotation(self, data):
        yaw, pitch, roll = [math.radians(v / 100) for v in data]
        # R_odom_rear = R_odom_esp * R_esp_rear.
        q = quat.multiply(quat.euler_to_quaternion(yaw, pitch, roll), quat.conjugate(self.rear_from_esp))
        self.rear = (self.time.total_seconds(), unit_quaternion(q))

    def on_orientation_list(self, data):
        if not data:
            return
        if self.imu_offset is None:
            self.imu_offset = self.time.total_seconds() - data[-1][0]
        for device_time, _, x, y, z, w in data:
            # R_odom_front = R_odom_fused * R_fused_front.
            q = quat.multiply(unit_quaternion([x, y, z, w]), quat.conjugate(self.front_from_fused))
            self.attitudes.add(device_time + self.imu_offset, q)
        self.drain()

    def on_odometry(self, data):
        if len(data) != 4 or len(data[3]) != 4:
            raise ValueError('odometry must be [status, speed_mm_s, joint_cdeg, [four uint16 distances]]')
        if any(not isinstance(v, int) or not 0 <= v <= 65535 for v in data[3]):
            raise ValueError('Encoder readings must be unsigned 16-bit integers')
        t = self.time.total_seconds()
        rear = self.rear if self.rear and 0 <= t - self.rear[0] <= self.max_rear_age else None
        self.pending.append((t, data, rear))
        if len(self.pending) > 200:
            self.pending.popleft()
            self.skipped += 1
            self.model.continuous = False
        self.drain()

    def drain(self):
        while self.pending and self.attitudes.samples:
            timestamp, bundle, rear = self.pending[0]
            if timestamp > self.attitudes.samples[-1][0]:
                break  # Await bracketing camera sample; no extrapolation.
            self.pending.popleft()
            front = self.attitudes.at(timestamp)
            if front is None or rear is None:
                self.skipped += 1
                if self.model.previous is not None:
                    self.model.previous = None
                    self.model.continuous = False
                continue
            outputs = self.model.update(timestamp, bundle, front, rear[1])
            outputs['pose_quality'].update(
                publish_time_s=self.time.total_seconds(), imu_clock_mode=self.clock_mode,
                imu_time_offset_s=self.imu_offset, rear_age_s=timestamp - rear[0],
                geometry_id=self.geometry_id, geometry_verified=self.geometry_verified,
                skipped_inputs=self.skipped,
                usable_for_driving=False)  # Shadow implementation, not validated motion protection.
            for name, value in outputs.items():
                self.publish(name, value)
