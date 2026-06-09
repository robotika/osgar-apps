"""
  Pure Depth-Based Robot-Following Node
"""

import os
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

        # Coexistence & Delegation (Section 6)
        self.algorithm = config.get('algorithm', 'slice')

        # Plan B specific configurations (Section 2 & 7)
        self.min_phys_width = config.get('min_phys_width', 0.3)
        self.max_phys_width = config.get('max_phys_width', 0.8)
        self.min_phys_height = config.get('min_phys_height', 0.2)
        self.max_phys_height = config.get('max_phys_height', 0.7)
        self.min_aspect = config.get('min_aspect', 0.5)
        self.max_aspect = config.get('max_aspect', 2.0)
        self.min_area = config.get('min_area', 200)

        self.min_depth = config.get('min_depth', 500)  # in mm (0.5m)
        self.max_depth = config.get('max_depth', 3200) # in mm (3.2m)

        # State variables
        self.last_target_x = None
        self.last_distance = None
        self.last_target_time = None
        self.pitch = None
        self.yaw = None
        self.status_ready = False

        # Plan B tracking state variables
        self.last_target_y = None
        self.tracker_candidate_bbox = None
        self.tracker_candidate_count = 0
        self.emergency_stop = False
        self.debug_images_cleaned = False

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        self.emergency_stop = bool(data)
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

        if self.algorithm == 'slice':
            target_x, distance = self.track_via_slice(data)
        elif self.algorithm == 'clustering':
            target_x, distance = self.track_via_clustering(data)
        else:
            raise ValueError(f"Unknown tracking algorithm: {self.algorithm}")

        if target_x is not None and distance is not None:
            self.last_target_x = target_x
            self.last_distance = distance
            self.last_target_time = self.time

        # Calculate a 1D scan from the central horizon slice for standard scan telemetry publishing
        current_horizon = self.horizon
        if self.pitch is not None:
            pixels_per_degree = h / 44.0
            pitch_deg = self.pitch / 100.0
            current_horizon -= int(pitch_deg * pixels_per_degree)

        half_height = self.depth_height // 2
        line = max(0, int(current_horizon - half_height))
        line_end = min(h, int(current_horizon + half_height))

        depth_slice = data[line:line_end, :]
        depth_slice_clean = depth_slice.copy()
        depth_slice_clean[depth_slice_clean == 0] = 10000

        scan_1d = np.percentile(depth_slice_clean, 10, axis=0) / 1000.0  # mm to meters

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

    def track_via_slice(self, data):
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

        target_x_ret = None
        distance_ret = None

        if min_d <= self.max_track_dist:
            # Identify all columns in the search window close to min_d (within 30cm) and within track range
            is_target = (window_scan <= min_d + 0.3) & (window_scan < self.max_track_dist)
            target_indices = np.where(is_target)[0] + x_start

            if len(target_indices) > 0:
                target_x_ret = float(np.mean(target_indices))
                distance_ret = float(np.percentile(scan_1d[target_indices], 10))

        return target_x_ret, distance_ret

    def track_via_clustering(self, data):
        import cv2
        h_full, w_full = data.shape
        center_x = w_full / 2.0

        # Step 1: Define a Wide 2D Region of Interest (ROI) (Section 2)
        # Scaled dynamically based on resolution
        x_min = int(80 * (w_full / 640.0))
        x_max = int(560 * (w_full / 640.0))
        y_min = int(120 * (h_full / 400.0))
        y_max = int(360 * (h_full / 400.0))

        roi = data[y_min:y_max, x_min:x_max]

        # Step 2: Distance Range Filtering
        binary_mask = ((roi >= self.min_depth) & (roi <= self.max_depth)).astype(np.uint8)

        # Step 3: Connected-Component Labeling (Blob Detection)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)

        # Step 4: 3D Physical Dimension Verification
        candidates = []
        rejected = []

        tan_half_hfov = math.tan(math.radians(34.5))
        tan_half_vfov = math.tan(math.radians(22.0))

        for i in range(1, num_labels):
            x_roi = stats[i, cv2.CC_STAT_LEFT]
            y_roi = stats[i, cv2.CC_STAT_TOP]
            w_box = stats[i, cv2.CC_STAT_WIDTH]
            h_box = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            cx_roi = centroids[i, 0]
            cy_roi = centroids[i, 1]

            # Convert to full frame coordinates
            x = x_roi + x_min
            y = y_roi + y_min
            cx = cx_roi + x_min
            cy = cy_roi + y_min

            # Calculate median depth of the cluster
            cluster_depths = roi[labels == i]
            if len(cluster_depths) > 0:
                median_depth_mm = np.median(cluster_depths)
            else:
                median_depth_mm = 10000.0
            median_depth_m = median_depth_mm / 1000.0

            # Projected 3D dimensions (Section 2 - Step 4)
            phys_w = 2.0 * median_depth_m * tan_half_hfov * (w_box / w_full)
            phys_h = 2.0 * median_depth_m * tan_half_vfov * (h_box / h_full)
            aspect_ratio = phys_w / phys_h if phys_h > 0 else 0.0

            # Classification checks
            is_valid_width = (self.min_phys_width <= phys_w <= self.max_phys_width)
            is_valid_height = (self.min_phys_height <= phys_h <= self.max_phys_height)
            is_valid_aspect = (self.min_aspect <= aspect_ratio <= self.max_aspect)
            is_valid_area = (area >= self.min_area)

            cluster_info = {
                'bbox': (x, y, w_box, h_box),
                'centroid': (cx, cy),
                'median_depth': median_depth_m,
                'phys_w': phys_w,
                'phys_h': phys_h,
                'aspect_ratio': aspect_ratio,
                'area': area
            }

            if is_valid_width and is_valid_height and is_valid_aspect and is_valid_area:
                candidates.append(cluster_info)
            else:
                # Store rejection reason for overlay
                if not is_valid_area:
                    cluster_info['reason'] = f"Area: {area}px < {self.min_area}"
                elif not is_valid_width:
                    cluster_info['reason'] = f"W: {phys_w:.2f}m"
                elif not is_valid_height:
                    cluster_info['reason'] = f"H: {phys_h:.2f}m"
                elif not is_valid_aspect:
                    cluster_info['reason'] = f"Aspect: {aspect_ratio:.2f}"
                rejected.append(cluster_info)

        # Step 5: Target Continuity & Tracking Match + Temporal Filtering (7.B)
        chosen_cluster = None
        event_type = "lost"

        if len(candidates) > 0:
            if self.tracker_candidate_bbox is not None:
                # Find overlapping candidates
                overlapping_candidates = []
                for cand in candidates:
                    cx_box, cy_box, cw, ch = cand['bbox']
                    tx, ty, tw, th = self.tracker_candidate_bbox
                    overlap = (cx_box < tx + tw) and (cx_box + cw > tx) and (cy_box < ty + th) and (cy_box + ch > ty)
                    if overlap:
                        overlapping_candidates.append(cand)

                if len(overlapping_candidates) > 0:
                    # Sort candidates
                    if self.tracker_candidate_count >= 3:
                        # Tracking is active, pick candidate closest to last_target_x, last_target_y
                        if self.last_target_x is not None and self.last_target_y is not None:
                            overlapping_candidates.sort(key=lambda cand: math.hypot(cand['centroid'][0] - self.last_target_x, cand['centroid'][1] - self.last_target_y))
                        else:
                            overlapping_candidates.sort(key=lambda cand: (abs(cand['centroid'][0] - center_x) / center_x) + (cand['median_depth'] / self.max_track_dist))
                    else:
                        # Still validating, sort by combined distance & center score
                        overlapping_candidates.sort(key=lambda cand: (abs(cand['centroid'][0] - center_x) / center_x) + (cand['median_depth'] / self.max_track_dist))

                    chosen_cluster = overlapping_candidates[0]
                    self.tracker_candidate_bbox = chosen_cluster['bbox']
                    self.tracker_candidate_count = min(3, self.tracker_candidate_count + 1)
                else:
                    # No overlapping candidates. Reset candidate and start new tracker candidate
                    self.tracker_candidate_bbox = None
                    self.tracker_candidate_count = 0

                    # Pick best of all candidates
                    candidates.sort(key=lambda cand: (abs(cand['centroid'][0] - center_x) / center_x) + (cand['median_depth'] / self.max_track_dist))
                    chosen_cluster = candidates[0]
                    self.tracker_candidate_bbox = chosen_cluster['bbox']
                    self.tracker_candidate_count = 1
            else:
                # No active candidate tracker, pick best of all candidates
                candidates.sort(key=lambda cand: (abs(cand['centroid'][0] - center_x) / center_x) + (cand['median_depth'] / self.max_track_dist))
                chosen_cluster = candidates[0]
                self.tracker_candidate_bbox = chosen_cluster['bbox']
                self.tracker_candidate_count = 1
        else:
            # No valid candidates at all
            self.tracker_candidate_bbox = None
            self.tracker_candidate_count = 0

        target_x, distance = None, None
        if chosen_cluster is not None and self.tracker_candidate_count >= 3:
            target_x = chosen_cluster['centroid'][0]
            distance = chosen_cluster['median_depth']
            event_type = "tracked"
            self.last_target_y = chosen_cluster['centroid'][1]
        else:
            event_type = "lost"

        if self.emergency_stop:
            event_type = "estop"

        # Diagnostic Image-Saving Trigger (Phase 1 & 2)
        if self.verbose:
            if not self.debug_images_cleaned:
                import glob as py_glob
                os.makedirs("debug_images", exist_ok=True)
                for f in py_glob.glob("debug_images/*.jpg"):
                    try:
                        os.remove(f)
                    except OSError:
                        pass
                self.debug_images_cleaned = True

            # Convert depth map to BGR grayscale representation for annotations
            display_depth = data.copy()
            display_depth[display_depth > 5000] = 5000
            display_depth_8 = (display_depth / 5000.0 * 255).astype(np.uint8)
            color_img = cv2.cvtColor(display_depth_8, cv2.COLOR_GRAY2BGR)

            # 1. Draw ROI Boundaries (dotted rectangle)
            draw_dashed_rectangle(color_img, (x_min, y_min), (x_max, y_max), (255, 255, 0), thickness=1, dash_length=8)

            # 2. Draw Rejected Overlays (Soft red/yellow boxes with reason)
            for rej in rejected:
                rx, ry, rw, rh = rej['bbox']
                cv2.rectangle(color_img, (rx, ry), (rx + rw, ry + rh), (0, 0, 200), 1)
                reason = rej.get('reason', '')
                cv2.putText(color_img, reason, (rx, max(15, ry - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)

            # 3. Draw Target Overlay (Bright green/yellow box with dimensions)
            if chosen_cluster is not None:
                cx_box, cy_box, cw, ch = chosen_cluster['bbox']
                is_active = (self.tracker_candidate_count >= 3)
                box_color = (0, 255, 0) if is_active else (0, 255, 255)
                cv2.rectangle(color_img, (cx_box, cy_box), (cx_box + cw, cy_box + ch), box_color, 2)

                label = "Robot" if is_active else "Pending"
                text = f"{label} (W: {chosen_cluster['phys_w']:.2f}m, H: {chosen_cluster['phys_h']:.2f}m, D: {chosen_cluster['median_depth']:.2f}m)"
                cv2.putText(color_img, text, (cx_box, max(15, cy_box - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1)

            short_log_name = get_short_log_name()
            centiseconds = int(round(self.time.total_seconds() * 100.0))
            filename = f"debug_images/{short_log_name}_{centiseconds:06d}_{event_type}.jpg"
            cv2.imwrite(filename, color_img)

        return target_x, distance

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

            # Plan B LED Solid Blue (Section 7.C)
            if self.algorithm == 'clustering':
                self.publish('set_leds', [LEFT_LED_INDEX, 0, 0, 255])
                self.publish('set_leds', [RIGHT_LED_INDEX, 0, 0, 255])

        elif age_sec <= 1.0:
            # Lock Retention Mode: Target is lost momentarily (up to 1.0s / 10 depth frames)
            # Decelerate speed to 0.0 for safety, but retain/decay steering towards last known direction
            speed = 0.0
            decay_factor = 1.0 - (age_sec / 1.0)
            steering_angle = decay_factor * self.get_steering_angle(self.last_target_x, center_x)

            # Plan B LED Blinking Orange/Yellow (Section 7.C)
            if self.algorithm == 'clustering':
                is_on = (int(self.time.total_seconds() * 4) % 2 == 0)
                color = [255, 127, 0] if is_on else [0, 0, 0]
                self.publish('set_leds', [LEFT_LED_INDEX] + color)
                self.publish('set_leds', [RIGHT_LED_INDEX] + color)

        else:
            # Target is completely lost
            speed = 0.0
            steering_angle = 0.0
            self.last_target_x = None
            self.last_distance = None

            # Plan B LED Solid Red or Blinking Orange/Yellow if initial search (Section 7.C)
            if self.algorithm == 'clustering':
                if self.last_target_time is None:
                    # Searching initially
                    is_on = (int(self.time.total_seconds() * 4) % 2 == 0)
                    color = [255, 127, 0] if is_on else [0, 0, 0]
                else:
                    # Target lost for longer than 1.0 second (safety timeout)
                    color = [255, 0, 0]
                self.publish('set_leds', [LEFT_LED_INDEX] + color)
                self.publish('set_leds', [RIGHT_LED_INDEX] + color)

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


def draw_dashed_rectangle(img, pt1, pt2, color, thickness=1, dash_length=10):
    import cv2
    x1, y1 = pt1
    x2, y2 = pt2
    # Draw top and bottom edges
    for x in range(x1, x2, dash_length * 2):
        cv2.line(img, (x, y1), (min(x + dash_length, x2), y1), color, thickness)
        cv2.line(img, (x, y2), (min(x + dash_length, x2), y2), color, thickness)
    # Draw left and right edges
    for y in range(y1, y2, dash_length * 2):
        cv2.line(img, (x1, y), (x1, min(y + dash_length, y2)), color, thickness)
        cv2.line(img, (x2, y), (x2, min(y + dash_length, y2)), color, thickness)


def get_short_log_name():
    import sys
    import os
    import re
    for arg in sys.argv:
        if arg.endswith('.log'):
            filename = os.path.basename(arg)
            # Match pattern like m05-matty-follow-robot-260608_183052.log
            match = re.match(r'^([^-]+)-.*_([0-9]+)\.log$', filename)
            if match:
                return f"{match.group(1)}_{match.group(2)}"
            # Fallback to general cleaning
            name = filename.replace('.log', '')
            name = re.sub(r'[^a-zA-Z0-9_]', '_', name)
            return name
    return "log"

# vim: expandtab sw=4 ts=4
