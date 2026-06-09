"""
  Pure Depth-Based Robot-Following Node
"""

import math
from datetime import timedelta

import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.followme import EmergencyStopException

LEFT_LED_INDEX = 1
RIGHT_LED_INDEX = 0
LED_COLORS = {  # red, green, blue
    'm01-': [0, 0, 0xFF],
    'm02-': [0, 0xFF, 0],
    'm03-': [0xFF, 0, 0],
    'm04-': [0xFF, 0x6E, 0xC7], # pink
    'm05-': [0xFF, 0x7F, 0]  # orange
}


class FollowRobot(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering',
                     'scan',
                     'set_leds')

        # Configuration parameters
        self.max_speed = config.get('max_speed', 0.5)
        self.target_distance = config.get('target_distance', 1.0)
        self.Kp_distance = config.get('Kp_distance', 0.5)
        self.Kp_steering = config.get('Kp_steering', 2.0)
        self.horizon = config.get('horizon', 300)
        self.max_track_dist = config.get('max_track_dist', 3.0)
        self.depth_height = config.get('depth_height', 60)
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)
        self.system_name = config.get('env', {}).get('OSGAR_LOGS_PREFIX', 'm01-')
        self.field_of_view = math.radians(69)  # OAK-D Pro camera horizontal FOV (69 degrees)

        # State variables
        self.last_target_x = None
        self.last_distance = None
        self.last_target_time = None
        self.pitch = None
        self.yaw = None
        self.status_ready = False

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.publish('set_leds', [LEFT_LED_INDEX, 0, 0, 0])  # turn off left LED
            self.publish('set_leds', [RIGHT_LED_INDEX, 0, 128, 0])  # turn on green LED
            self.send_speed_cmd(0, 0)  # STOP!

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def on_bumpers_rear(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def get_steering_angle(self, target_x, center_x):
        # Calculate angular deviation: steering left is positive, right is negative
        theta_err = ((center_x - target_x) / center_x) * (self.field_of_view / 2.0)
        return self.Kp_steering * theta_err

    def on_depth(self, data):
        # Local copy to prevent side effects
        data = data.copy()
        h, w = data.shape
        center_x = w / 2.0

        # Dynamic horizon adjustment based on pitch (e.g. from IMU)
        current_horizon = self.horizon
        if self.pitch is not None:
            # 1 degree of pitch corresponds to approx (h / 44.0) pixels on a ~44 deg vertical FOV mono camera
            pixels_per_degree = h / 44.0
            pitch_deg = self.pitch / 100.0
            current_horizon -= int(pitch_deg * pixels_per_degree)

        # Vertical band limits
        half_height = self.depth_height // 2
        line = max(0, int(current_horizon - half_height))
        line_end = min(h, int(current_horizon + half_height))

        # Extract horizon slice
        depth_slice = data[line:line_end, :]

        # Replace invalid pixels (0) with a large value representing maximum range (10m)
        depth_slice_clean = depth_slice.copy()
        depth_slice_clean[depth_slice_clean == 0] = 10000

        # Vectorized column-wise 10th percentile extraction
        scan_1d = np.percentile(depth_slice_clean, 10, axis=0) / 1000.0  # mm to meters

        # Determine dynamic search window width scaled by distance
        dist = self.last_distance if self.last_distance is not None else self.target_distance
        width_factor = w / 640.0
        window_width = max(40 * width_factor, min(160 * width_factor, round(120.0 * width_factor * (1.0 / dist))))

        # Define search window centered around the last known target x coordinate
        target_x = self.last_target_x if self.last_target_x is not None else center_x
        x_start = int(max(0, target_x - window_width / 2.0))
        x_end = int(min(w, target_x + window_width / 2.0))

        # Segment the target obstacle cluster within the search window
        window_scan = scan_1d[x_start:x_end]
        if len(window_scan) > 0:
            min_d = np.min(window_scan)
        else:
            min_d = 10.0

        if self.verbose:
            print(f"DEPTH: time={self.time} | min_d={min_d:.3f} | target_x={target_x:.1f} | window_width={window_width:.1f} | range=[{x_start},{x_end}]")

        if min_d <= self.max_track_dist:
            # Identify all columns in the search window close to min_d (within 30cm) and within track range
            is_target = (window_scan <= min_d + 0.3) & (window_scan < self.max_track_dist)
            target_indices = np.where(is_target)[0] + x_start

            if len(target_indices) > 0:
                # Update target track state
                self.last_target_x = float(np.mean(target_indices))
                self.last_distance = float(np.percentile(scan_1d[target_indices], 10))
                self.last_target_time = self.time

        # Subsample scan to 32 points for telemetry/logging
        subsample_factor = max(1, w // 32)
        scan_telemetry = [int(d * 1000) for d in scan_1d[::subsample_factor][:32]]
        self.publish('scan', scan_telemetry)

        # Handle LED initialization
        if not self.status_ready:
            led_color = LED_COLORS.get(self.system_name, [0, 0, 0])
            self.publish('set_leds', [LEFT_LED_INDEX] + [v // 2 for v in led_color])
            self.publish('set_leds', [RIGHT_LED_INDEX] + [v // 2 for v in led_color])
            self.status_ready = True

    def on_pose2d(self, data):
        # Parse current pose (x, y are in millimeters, heading is in centidegrees)
        x, y, heading = data
        w = 640  # default fallback width if no depth frame has been received yet
        center_x = w / 2.0

        # Calculate time elapsed since last successful target detection
        if self.last_target_time is not None:
            age_sec = (self.time - self.last_target_time).total_seconds()
        else:
            age_sec = float('inf')

        speed = 0.0
        steering_angle = 0.0

        if age_sec <= 0.1:
            # Target is actively tracked (10Hz depth provides updates every 0.1s)
            e_dist = self.last_distance - self.target_distance
            if e_dist > 0:
                speed = self.Kp_distance * e_dist
                speed = min(speed, self.max_speed)
            else:
                speed = 0.0

            steering_angle = self.get_steering_angle(self.last_target_x, center_x)

        elif age_sec <= 1.0:
            # Lock Retention Mode: Target is lost momentarily (up to 1.0s / 10 depth frames)
            # Decelerate speed to 0.0 for safety, but retain/decay steering towards last known direction
            speed = 0.0
            decay_factor = 1.0 - (age_sec / 1.0)
            steering_angle = decay_factor * self.get_steering_angle(self.last_target_x, center_x)

        else:
            # Target is completely lost
            speed = 0.0
            steering_angle = 0.0
            self.last_target_x = None
            self.last_distance = None

        if self.verbose:
            print(f"Time: {self.time} | Speed: {speed:.3f} | Steering: {math.degrees(steering_angle):.1f} | Age: {age_sec:.2f}")

        self.send_speed_cmd(speed, steering_angle)

    def on_rotation(self, data):
        yaw, pitch, roll = data
        self.yaw = math.radians(yaw / 100.0)
        self.pitch = pitch
        if self.verbose:
            print(f"ROTATION: time={self.time} | yaw={yaw / 100.0:.2f} | pitch={pitch / 100.0:.2f} | roll={roll / 100.0:.2f}")

    def on_orientation_list(self, data):
        pass

    def on_nmea_data(self, data):
        pass

# vim: expandtab sw=4 ts=4
