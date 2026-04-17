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
from osgar.followme import EmergencyStopException
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

        self.logfile = self.resolve_path(config.get('logfile'))
        self.pose2d_stream = config.get('pose2d_stream', 'platform.pose2d')
        self.ref_dir = self.resolve_path(config.get('ref_dir'))
        self.debug_dir = self.resolve_path(config.get('debug_dir'))

        self.min_brightness = config.get('min_brightness', 30.0)
        self.min_inliers = config.get('min_inliers', 20)
        self.visualize_alignment = config.get('visualize_alignment', False)
        
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
        best_ref_idx = -1

        best_mask = None
        best_matches = None
        best_ref_frame = None
        best_ref_kp = None

        best_M = None
        for i, ref in enumerate(self.ref_data):
            matches = self.bf.match(des, ref['des'])
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
                        best_M = M
                        best_mask = mask
                        best_matches = good
                        best_ref_frame = ref.get('frame')
                        best_ref_kp = ref.get('kp')
                        best_ref_idx = i

        if best_inliers >= self.min_inliers:
            ref_heading = best_pose[2]
            abs_query_heading = ref_heading - best_rot_rad
            
            # Simple translation estimation: M[0,2] and M[1,2] are tx, ty in pixels.
            # Without depth, we can't accurately map pixels to meters, but we can
            # at least log them for now or use a heuristic.
            tx, ty = best_M[0,2], best_M[1,2]
            
            print(f"Match found! Inliers: {best_inliers}, Ref Pose: {best_pose[:2]}, Ref Heading: {math.degrees(ref_heading):.1f} deg, Image Rot: {math.degrees(best_rot_rad):.1f} deg")
            print(f"Image translation tx: {tx:.1f}, ty: {ty:.1f} (pixels)")
            
            if self.visualize_alignment and best_ref_frame is not None and best_ref_kp is not None:
                draw_params = dict(matchColor = (0,255,0),
                               singlePointColor = None,
                               matchesMask = best_mask.flatten().tolist(),
                               flags = 2)
                vis_img = cv2.drawMatches(img, kp, best_ref_frame, best_ref_kp, best_matches, None, **draw_params)
                out_path = os.path.join(os.path.dirname(self.logfile), "alignment_match.png")
                cv2.imwrite(out_path, vis_img)
                print(f"Saved alignment visualization to {out_path}")

            self.pose_offset = [best_pose[0], best_pose[1], abs_query_heading]
            self.state = self.STATE_DRIVING
            print(f"Switched to STATE_DRIVING at {best_pose[:2]} with heading {math.degrees(abs_query_heading):.1f} deg")
            
            # Important: We need to feed the CURRENT (corrected) pose to FollowPath immediately
            # so it doesn't use the old [0,0,0] which might be far away.
            # The on_pose2d will be called with next incoming data, but let's force an update.
            # Actually, we don't have the raw 'data' here, but we know it's approx [0,0,0].
            # Let's wait for the next on_pose2d call which will happen very soon.
        else:
            print(f"Alignment failed (best inliers: {best_inliers} at ref_idx {best_ref_idx})")
            if self.visualize_alignment and best_inliers > 5 and best_ref_frame is not None:
                draw_params = dict(matchColor = (0,0,255),
                               singlePointColor = None,
                               matchesMask = best_mask.flatten().tolist(),
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
