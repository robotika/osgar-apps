"""
  Rerun Route from OSGAR log with Visual Alignment
"""
import glob
import math
import os
import re
import sys

# Ensure we can find local modules
if os.path.dirname(__file__) not in sys.path:
    sys.path.append(os.path.dirname(__file__))

import av
import cv2
import numpy as np
from extract_route_images import extract_reference_data
from osgar.bus import BusShutdownException
from osgar.followme import EmergencyStopException
from osgar.followpath import FollowPath, Route
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id
from osgar.node import Node


class VideoDecoder:
    def __init__(self, codec_name='hevc'):
        # Use 'hevc' for H.265 or 'h264' for H.264
        self.codec = av.CodecContext.create(codec_name, 'r')

    def decode(self, data: bytes):
        """
        Takes raw H.264/H.265 bytes and returns the decoded OpenCV image (BGR).
        Returns None if the packet didn't contain enough data to form a full frame yet.
        """
        # Parse the raw bytes into FFmpeg Packets
        try:
            packets = self.codec.parse(data)
        except av.AVError as e:
            print(f"Warning: Failed to parse video packet: {e}")
            return None

        frames = []
        for packet in packets:
            try:
                # Decode the packet into VideoFrames
                frames.extend(self.codec.decode(packet))
            except av.AVError as e:
                print(f"Warning: Failed to decode video packet: {e}")
                continue

        if frames:
            # We return the last frame decoded in this batch
            # Convert it directly to a numpy array in OpenCV's BGR format
            return frames[-1].to_ndarray(format='bgr24')

        return None


class RerunRoute(Node):
    STATE_WAIT_FOR_IMAGE = 0
    STATE_JOINING = 1
    STATE_DRIVING = 2

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')

        self.logfile = self.resolve_path(config.get('logfile'))
        self.pose2d_stream = config.get('pose2d_stream', 'platform.pose2d')
        self.ref_dir = self.resolve_path(config.get('ref_dir'))
        self.debug_dir = self.resolve_path(config.get('debug_dir'))

        self.min_brightness = config.get('min_brightness', 30.0)
        self.min_inliers = config.get('min_inliers', 20)
        self.visualize_alignment = config.get('visualize_alignment', False)
        self.join_threshold = config.get('join_threshold', 0.5)

        # OAK-D THE_1080_P approximate intrinsics
        # TODO: These should be provided by the camera driver or calibrated for the specific resolution.
        # Currently, they assume 1920x1080 (THE_1080_P).
        intrinsics = config.get('intrinsics', [1400.0, 1400.0, 960.0, 540.0]) # fx, fy, cx, cy
        self.camera_matrix = np.array([[intrinsics[0], 0, intrinsics[2]],
                                       [0, intrinsics[1], intrinsics[3]],
                                       [0, 0, 1.0]], dtype=float)
        self.dist_coeffs = np.zeros((4,1)) # Assuming no distortion for now

        # Load path from log file
        self.path = self.extract_path(self.logfile, self.pose2d_stream)
        if not self.path:
            print(f"ERROR: No path extracted from {self.logfile}")
        else:
            print(f"Extracted {len(self.path)} points from {self.logfile}")

        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.ref_data = []

        if self.ref_dir:
            self.load_reference_images(self.ref_dir)
        elif self.logfile:
            print(f"Auto-extracting reference data from {self.logfile}...")
            self.ref_data = extract_reference_data(self.logfile, orb=self.orb, debug_dir=self.debug_dir)

        self.app = FollowPath(config, bus)
        self.app.route = Route(pts=self.path)

        # Override app methods to use this node's bus
        self.app.publish = self.my_publish
        self.app.listen = self.my_listen
        self.app.update = self.my_update

        self.state = self.STATE_WAIT_FOR_IMAGE if (self.ref_dir or self.logfile) else self.STATE_DRIVING
        self.pose_offset = [0.0, 0.0, 0.0] # x, y, heading_rad
        self.last_depth = None
        self.decoder = VideoDecoder(codec_name='hevc')
        print(f"Initial state: {self.state}")

    def resolve_path(self, path):
        if path is None or os.path.isabs(path):
            return path
        # Always use the directory of main.py as the single base for relative paths
        app_dir = os.path.dirname(__file__)
        return os.path.join(app_dir, path)

    def load_reference_images(self, ref_dir):
        print(f"Loading reference images from {ref_dir}...")
        ref_files = glob.glob(os.path.join(ref_dir, "*.png"))
        for ref_path in ref_files:
            img = cv2.imread(ref_path)
            if img is None:
                continue
            kp, des = self.orb.detectAndCompute(img, None)
            if des is not None:
                # Extract pose from filename: frame_000000_x0.00_y0.00.png
                match = re.search(r'_x(-?\d+\.\d+)_y(-?\d+\.\d+)', ref_path)
                if match:
                    pose = (float(match.group(1)), float(match.group(2)))
                    self.ref_data.append({'des': des, 'kp': kp, 'pose': pose, 'path': ref_path})
        print(f"Loaded {len(self.ref_data)} references.")

    def my_publish(self, name, data):
        self.publish(name, data)

    def my_listen(self):
        return self.listen()

    def my_update(self):
        return self.update()

    def extract_path(self, logfile, pose2d_stream):
        if logfile is None:
            return []
        path = []
        try:
            stream_id = lookup_stream_id(logfile, pose2d_stream)
        except Exception as e:
            print(f"Error looking up stream {pose2d_stream} in {logfile}: {e}")
            return []

        with LogReader(logfile, only_stream_id=stream_id) as log:
            for timestamp, stream_id, data in log:
                pose = deserialize(data)
                x, y = pose[0]/1000.0, pose[1]/1000.0
                if len(path) == 0 or math.hypot(path[-1][0] - x, path[-1][1] - y) > 0.1:
                    path.append((x, y))
        return path

    def on_depth(self, data):
        self.last_depth = data

    def on_pose2d(self, data):
        # Raw data from robot platform (starts at 0,0,0)
        x, y, heading = data

        # 1. Apply rotation first
        heading_rad = math.radians(heading / 100.0)
        corrected_heading_rad = heading_rad + self.pose_offset[2]

        # 2. Apply translation
        c, s = math.cos(self.pose_offset[2]), math.sin(self.pose_offset[2])
        rel_x, rel_y = x / 1000.0, y / 1000.0

        abs_x = self.pose_offset[0] + rel_x * c - rel_y * s
        abs_y = self.pose_offset[1] + rel_x * s + rel_y * c

        corrected_data = [
            int(abs_x * 1000),
            int(abs_y * 1000),
            int(math.degrees(corrected_heading_rad) * 100)
        ]

        if self.state == self.STATE_DRIVING:
            self.app.on_pose2d(corrected_data)
        elif self.state == self.STATE_JOINING:
            self.drive_to_path(abs_x, abs_y, corrected_heading_rad)

    def drive_to_path(self, x, y, heading):
        # Smooth curve joining: steer towards the nearest point on the route
        # Find closest point
        first, second = self.app.route.routeSplit((x, y))
        if len(second) == 0:
            print("Joining failed: no route points.")
            self.state = self.STATE_DRIVING # Fallback
            return

        target_pt = second[0]
        dist = math.hypot(target_pt[1] - y, target_pt[0] - x)

        if dist < self.join_threshold:
            print(f"Joined path (dist {dist:.2f}m). Switching to STATE_DRIVING.")
            self.state = self.STATE_DRIVING
            return

        # Target angle to the point
        target_angle = math.atan2(target_pt[1] - y, target_pt[0] - x)
        diff = target_angle - heading
        # Normalize to -pi, pi
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        # Proportional control for steering
        max_angular = math.radians(45)
        angular_speed = 1.0 * diff
        angular_speed = max(min(angular_speed, max_angular), -max_angular)

        # Send speed command
        self.publish('desired_speed', [round(self.app.max_speed * 1000), round(math.degrees(angular_speed) * 100)])

    def on_color(self, data):
        if self.state != self.STATE_WAIT_FOR_IMAGE:
            return

        img = self.decoder.decode(data)
        if img is None:
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        brightness = cv2.mean(gray)[0]
        if brightness < self.min_brightness:
            return

        print(f"Image quality OK (brightness {brightness:.1f}). Aligning...")
        kp, des = self.orb.detectAndCompute(img, None)
        if des is None or len(des) < 10:
            return

        best_inliers = 0
        best_pose = None
        best_ref_idx = -1
        best_rvec = None
        best_tvec = None

        best_mask = None
        best_matches = None
        best_ref_frame = None
        best_ref_kp = None

        for i, ref in enumerate(self.ref_data):
            matches = self.bf.match(des, ref['des'])
            good = [m for m in matches if m.distance < 50]

            if len(good) >= 10:
                # Try PnP if we have 3D points
                ref_kp3d = ref.get('kp_3d')
                if ref_kp3d is not None:
                    obj_pts = []
                    img_pts = []
                    for m in good:
                        p3d = ref_kp3d[m.trainIdx]
                        if p3d is not None:
                            obj_pts.append(p3d)
                            img_pts.append(kp[m.queryIdx].pt)

                    if len(obj_pts) >= 10:
                        obj_pts = np.array(obj_pts, dtype=float)
                        img_pts = np.array(img_pts, dtype=float)
                        ret, rvec, tvec, inliers_indices = cv2.solvePnPRansac(
                            obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                            reprojectionError=5.0, iterationsCount=100)

                        if ret:
                            inliers = len(inliers_indices)
                            if inliers > best_inliers:
                                best_inliers = inliers
                                best_pose = ref['pose']
                                best_rvec = rvec
                                best_tvec = tvec
                                best_mask = np.zeros(len(good), dtype=bool)
                                best_mask[inliers_indices] = True
                                best_matches = good
                                best_ref_frame = ref.get('frame')
                                best_ref_kp = ref.get('kp')
                                best_ref_idx = i
                else:
                    # Fallback to Homography if no 3D data
                    src_pts = np.float32([kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([ref['kp'][m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if mask is not None:
                        inliers = int(np.sum(mask))
                        if inliers > best_inliers:
                            best_inliers = inliers
                            best_pose = ref['pose']
                            best_mask = mask
                            best_matches = good
                            best_ref_frame = ref.get('frame')
                            best_ref_kp = ref.get('kp')
                            best_ref_idx = i
                            best_rvec = None # No 3D info

        if best_inliers >= self.min_inliers:
            ref_x, ref_y, ref_heading = best_pose

            if best_rvec is not None:
                # Calculate absolute pose from PnP result
                R, _ = cv2.Rodrigues(best_rvec)
                pos_in_ref = -R.T @ best_tvec
                dx, dy = pos_in_ref[0,0], pos_in_ref[2,0]
                yaw_diff = -best_rvec[1,0]

                c, s = math.cos(ref_heading), math.sin(ref_heading)
                abs_x = ref_x + dx * c - dy * s
                abs_y = ref_y + dx * s + dy * c
                abs_heading = ref_heading + yaw_diff

                print(f"Match found (PnP)! Inliers: {best_inliers}, "
                      f"Ref Pose: {best_pose[:2]}, "
                      f"Offset: ({dx:.2f}, {dy:.2f})m, "
                      f"yaw: {math.degrees(yaw_diff):.1f} deg")
            else:
                # Fallback to simple snap
                abs_x, abs_y, abs_heading = ref_x, ref_y, ref_heading
                print(f"Match found (Snap)! Inliers: {best_inliers}, Ref Pose: {best_pose[:2]}")

            self.pose_offset = [abs_x, abs_y, abs_heading]

            # Transition to JOINING or DRIVING
            first, second = self.app.route.routeSplit((abs_x, abs_y))
            dist = 0.0
            if len(second) > 0:
                dist = math.hypot(second[0][1] - abs_y, second[0][0] - abs_x)

            if dist > self.join_threshold:
                self.state = self.STATE_JOINING
                print(f"State: STATE_JOINING (dist to path: {dist:.2f}m)")
            else:
                self.state = self.STATE_DRIVING
                print(f"State: STATE_DRIVING (dist to path: {dist:.2f}m)")
        else:
            print(f"Alignment failed (best inliers: {best_inliers} at ref_idx {best_ref_idx})")
            if self.visualize_alignment and best_inliers > 5 and best_ref_frame is not None:
                mask_list = best_mask.astype(int).flatten().tolist()
                draw_params = dict(matchColor = (0,0,255),
                               singlePointColor = None,
                               matchesMask = mask_list,
                               flags = 2)
                vis_img = cv2.drawMatches(img, kp, best_ref_frame, best_ref_kp, best_matches, None, **draw_params)
                out_path = os.path.join(os.path.dirname(self.logfile), "alignment_failed.png")
                cv2.imwrite(out_path, vis_img)
                print(f"Saved failed alignment visualization to {out_path}")

    def on_emergency_stop(self, data):
        if data:
            print("!!!Emergency STOP!!!")
            raise EmergencyStopException()
        self.app.on_emergency_stop(data)

    def on_obstacle(self, data):
        if hasattr(self.app, 'on_obstacle'):
            self.app.on_obstacle(data)

    def run(self):
        try:
            while not self.app.finished:
                self.update()
        except (BusShutdownException, EmergencyStopException):
            pass
        print("Route finished, requesting stop.")
        self.request_stop()

# vim: expandtab sw=4 ts=4
