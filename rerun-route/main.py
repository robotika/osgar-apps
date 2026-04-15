"""
  Rerun Route from OSGAR log with Visual Alignment
"""
import math
import os
import cv2
import numpy as np
import glob
import re

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.followpath import FollowPath, Route
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def read_h264_image(data, i_frame_only=True):
    # Decoding logic from robotem-rovne/view_mask.py
    # Note: This creates a temporary file 'tmp.h26x' in the current directory.
    is_h264 = data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 0930'))
    is_h265 = data.startswith(bytes.fromhex('00000001 460150')) or data.startswith(bytes.fromhex('00000001 460130'))
    if not (is_h264 or is_h265):
        return None

    if data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 460150')):
        # I - key frame
        with open('tmp.h26x', 'wb') as f:
            f.write(data)
    elif data.startswith(bytes.fromhex('00000001 0930')) or data.startswith(bytes.fromhex('00000001 460130')):
        # P-frame
        if i_frame_only:
            return None
        with open('tmp.h26x', 'ab') as f:
            f.write(data)
    else:
        return None

    cap = cv2.VideoCapture('tmp.h26x')
    image = None
    ret = True
    while ret:
        ret, frame = cap.read()
        if ret:
            image = frame
    cap.release()
    return image


import sys
# Ensure we can find local modules
sys.path.append(os.path.dirname(__file__))
from extract_route_images import extract_reference_data

class RerunRoute(Node):
    STATE_WAIT_FOR_IMAGE = 0
    STATE_DRIVING = 1

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.logfile = config.get('logfile')
        self.pose2d_stream = config.get('pose2d_stream', 'platform.pose2d')
        self.ref_dir = config.get('ref_dir') # Optional: load from dir instead of log
        self.debug_dir = config.get('debug_dir') # Optional: export images from log to dir
        self.min_brightness = config.get('min_brightness', 30.0)
        self.min_inliers = config.get('min_inliers', 20)
        
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
        print(f"Initial state: {self.state}")

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

    def on_pose2d(self, data):
        # Raw data from robot platform (starts at 0,0,0)
        x, y, heading = data
        
        # 1. Apply rotation first
        # For simplicity, we only correct initial heading
        heading_rad = math.radians(heading / 100.0)
        corrected_heading_rad = heading_rad + self.pose_offset[2]
        
        # 2. Apply translation
        # Robot's (x, y) are in its local coordinate system.
        # We need to rotate them by initial heading offset and then add initial (x, y)
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

    def on_color(self, data):
        if self.state != self.STATE_WAIT_FOR_IMAGE:
            return

        img = read_h264_image(data)
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
        best_rot_rad = 0.0

        for ref in self.ref_data:
            matches = self.bf.match(des, ref['des'])
            # We use a simple distance threshold here for speed in live Node
            good = [m for m in matches if m.distance < 50]
            
            if len(good) >= 10:
                src_pts = np.float32([kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                dst_pts = np.float32([ref['kp'][m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                if mask is not None:
                    inliers = int(np.sum(mask))
                    if inliers > best_inliers:
                        best_inliers = inliers
                        best_pose = ref['pose']
                        best_rot_rad = -math.atan2(M[1,0], M[0,0])

        if best_inliers >= self.min_inliers:
            # We assume for now that the reference log also started with heading 0
            # If not, we'd need to extract reference heading from the log as well.
            print(f"Match found! Inliers: {best_inliers}, Ref Pose: {best_pose}, Rot: {math.degrees(best_rot_rad):.1f} deg")
            self.pose_offset = [best_pose[0], best_pose[1], best_rot_rad]
            self.state = self.STATE_DRIVING
            print(f"Switched to STATE_DRIVING with offset {self.pose_offset}")
        else:
            print(f"Alignment failed (best inliers: {best_inliers})")

    def on_emergency_stop(self, data):
        self.app.on_emergency_stop(data)

    def on_obstacle(self, data):
        if hasattr(self.app, 'on_obstacle'):
            self.app.on_obstacle(data)

    def run(self):
        try:
            while not self.app.finished:
                self.update()
        except BusShutdownException:
            pass
        print("Route finished, requesting stop.")
        self.request_stop()

# vim: expandtab sw=4 ts=4
