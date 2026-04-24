
import argparse
import math
import os
import sys

import av
import cv2
import numpy as np

# Ensure we can find local modules
if os.path.dirname(__file__) not in sys.path:
    sys.path.append(os.path.dirname(__file__))

from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id

class VideoDecoder:
    def __init__(self, codec_name='hevc'):
        self.codec = av.CodecContext.create(codec_name, 'r')

    def decode(self, data: bytes):
        try:
            packets = self.codec.parse(data)
        except av.AVError:
            return None

        for packet in packets:
            try:
                frames = self.codec.decode(packet)
                if frames:
                    return frames[-1].to_ndarray(format='bgr24')
            except av.AVError:
                continue
        return None

def get_closest_data(ts, history):
    if not history:
        return None
    best_data = history[0][1]
    min_diff = abs((ts - history[0][0]).total_seconds())
    for d_ts, d_data in history:
        diff = abs((ts - d_ts).total_seconds())
        if diff < min_diff:
            min_diff = diff
            best_data = d_data
        elif diff > min_diff:
            break
    return best_data

def validate_calibration(log_path, num_plots=5):
    # Hardcoded intrinsics from main.py
    fx, fy = 1400.0, 1400.0
    cx, cy = 960.0, 540.0
    camera_matrix = np.array([[fx, 0, cx],
                               [0, fy, cy],
                               [0, 0, 1.0]], dtype=float)
    dist_coeffs = np.zeros((4,1))

    print(f"Analyzing {log_path}...")
    try:
        color_stream = lookup_stream_id(log_path, "oak.color")
        pose_stream = lookup_stream_id(log_path, "platform.pose2d")
        depth_stream = lookup_stream_id(log_path, "oak.depth")
    except Exception as e:
        print(f"Error: Required streams not found in log: {e}")
        return

    print("Reading pose and depth history...")
    pose_history = []
    with LogReader(log_path, only_stream_id=pose_stream) as log:
        for timestamp, stream_id, data in log:
            pose_history.append((timestamp, deserialize(data)))

    depth_history = []
    with LogReader(log_path, only_stream_id=depth_stream) as log:
        for timestamp, stream_id, data in log:
            depth_history.append((timestamp, deserialize(data)))

    orb = cv2.ORB_create(nfeatures=2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    decoder = VideoDecoder('hevc')

    last_frame_data = None
    errors = []
    plot_count = 0
    
    print("Processing frames...")
    with LogReader(log_path, only_stream_id=color_stream) as log:
        for timestamp, stream_id, data in log:
            raw_data = deserialize(data)
            frame = decoder.decode(raw_data)
            if frame is None:
                continue

            pose = get_closest_data(timestamp, pose_history)
            depth = get_closest_data(timestamp, depth_history)
            
            if pose is None or depth is None:
                # print(f"  [{timestamp}] Missing pose or depth")
                continue
                
            kp, des = orb.detectAndCompute(frame, None)
            if des is None or len(des) < 10:
                # print(f"  [{timestamp}] Not enough features")
                continue
                
            current_frame_data = {
                'frame': frame,
                'kp': kp,
                'des': des,
                'pose': pose,
                'depth': depth,
                'timestamp': timestamp
            }
            
            if last_frame_data is not None:
                # Check if there was significant movement to avoid degenerate cases
                x1, y1, _ = last_frame_data['pose']
                x2, y2, _ = current_frame_data['pose']
                dist_moved = math.hypot(x2 - x1, y2 - y1) / 1000.0
                
                if dist_moved > 0.05: # Reduced threshold to 5cm
                    matches = bf.match(last_frame_data['des'], current_frame_data['des'])
                    # print(f"  [{timestamp}] dist_moved: {dist_moved:.3f}m, matches: {len(matches)}")
                    
                    if len(matches) > 20:
                        obj_pts = []
                        img_pts = []
                        
                        d_h, d_w = last_frame_data['depth'].shape
                        f_h, f_w = last_frame_data['frame'].shape[:2]
                        
                        for m in matches:
                            kp_prev = last_frame_data['kp'][m.queryIdx]
                            u, v = int(kp_prev.pt[0]), int(kp_prev.pt[1])
                            ud, vd = int(u * d_w / f_w), int(v * d_h / f_h)
                            d = last_frame_data['depth'][vd, ud] if (0 <= vd < d_h and 0 <= ud < d_w) else 0
                            
                            if d > 0:
                                z = d / 1000.0
                                xc = (u - cx) * z / fx
                                yc = (v - cy) * z / fy
                                obj_pts.append([xc, yc, z])
                                img_pts.append(current_frame_data['kp'][m.trainIdx].pt)
                        
                        # print(f"  [{timestamp}] Valid 3D points: {len(obj_pts)}")
                                
                        if len(obj_pts) >= 15:
                            obj_pts = np.array(obj_pts, dtype=float)
                            img_pts = np.array(img_pts, dtype=float)
                            
                            # Using solvePnPRansac to validate intrinsics/depth-color alignment
                            ret, rvec, tvec, inliers = cv2.solvePnPRansac(
                                obj_pts, img_pts, camera_matrix, dist_coeffs,
                                reprojectionError=5.0, iterationsCount=100)
                            
                            if ret and inliers is not None:
                                # Calculate reprojection error for inliers
                                projected_pts, _ = cv2.projectPoints(obj_pts[inliers], rvec, tvec, camera_matrix, dist_coeffs)
                                projected_pts = projected_pts.reshape(-1, 2)
                                actual_pts = img_pts[inliers].reshape(-1, 2)
                                
                                err = np.linalg.norm(projected_pts - actual_pts, axis=1)
                                mean_err = np.mean(err)
                                errors.append(mean_err)
                                
                                if plot_count < num_plots:
                                    vis_img = current_frame_data['frame'].copy()
                                    for i in range(len(actual_pts)):
                                        # Circle for observation (Actual)
                                        cv2.circle(vis_img, (int(actual_pts[i][0]), int(actual_pts[i][1])), 6, (0, 255, 0), 2)
                                        # Cross for projection (Predicted)
                                        px, py = int(projected_pts[i][0]), int(projected_pts[i][1])
                                        cv2.line(vis_img, (px-6, py-6), (px+6, py+6), (0, 0, 255), 2)
                                        cv2.line(vis_img, (px+6, py-6), (px-6, py+6), (0, 0, 255), 2)
                                    
                                    out_name = f"debug_calib_{plot_count:02d}.png"
                                    cv2.imwrite(out_name, vis_img)
                                    print(f"Saved {out_name} (Mean Error: {mean_err:.2f} px, Inliers: {len(inliers)})")
                                    plot_count += 1
                    
                    last_frame_data = current_frame_data
                
            else:
                last_frame_data = current_frame_data
                
            if len(errors) >= 50:
                break
                
    if errors:
        print(f"\nResults for {os.path.basename(log_path)}:")
        print(f"  Samples: {len(errors)}")
        print(f"  Average Reprojection Error: {np.mean(errors):.2f} pixels")
        print(f"  Std Dev: {np.std(errors):.2f} pixels")
    else:
        print("No matches found to calculate error.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--plots", type=int, default=5, help="Number of visual plots to generate")
    args = parser.parse_args()
    validate_calibration(args.logfile, args.plots)
