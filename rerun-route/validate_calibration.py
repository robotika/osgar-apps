
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

def get_rotation_matrix(yaw, pitch=0, roll=0):
    # Standard ZYX rotation (Yaw, Pitch, Roll)
    # Yaw: around Z, Pitch: around Y, Roll: around X
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    
    return Rz @ Ry @ Rx

def validate_calibration(log_path, num_plots=5, limit=50, min_dist=0.05, use_pose=False, mount_offset=(0,0,0), mount_pitch=0, joint_offset=0, debug_frame=-1):
    # Hardcoded intrinsics from main.py
    fx, fy = 1400.0, 1400.0
    cx, cy = 960.0, 540.0
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1.0]], dtype=float)
    dist_coeffs = np.zeros((4, 1))

    # Camera coord system: Z forward, X right, Y down
    # Robot coord system: X forward, Y left, Z up
    # Rotation from Robot Front-Section to Camera (assuming pointing straight forward)
    # Cx = -Ry, Cy = -Rz, Cz = Rx
    R_front_to_cam_base = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0]
    ])

    # Add mounting pitch (rotation around robot's Y-axis)
    R_front_to_cam_base = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    R_pitch = np.array([
        [math.cos(mount_pitch), 0, math.sin(mount_pitch)],
        [0, 1, 0],
        [-math.sin(mount_pitch), 0, math.cos(mount_pitch)]
    ])
    R_front_to_cam = R_front_to_cam_base @ R_pitch

    print(f"Analyzing {log_path}...")
    if use_pose:
        print(f"  Mode: ROBOT POSE (Mount: {mount_offset}m, Pitch: {math.degrees(mount_pitch):.1f} deg, Joint Offset: {math.degrees(joint_offset):.1f} deg)")
    else:
        print("  Mode: solvePnPRansac (Estimated Pose)")

    try:
        color_stream = lookup_stream_id(log_path, "oak.color")
        pose_stream = lookup_stream_id(log_path, "platform.pose2d")
        depth_stream = lookup_stream_id(log_path, "oak.depth")
        joint_stream = lookup_stream_id(log_path, "platform.joint_angle")
    except Exception as e:
        print(f"Error: Required streams not found: {e}")
        return

    print("Reading history streams...")
    pose_history = []
    with LogReader(log_path, only_stream_id=pose_stream) as log:
        for timestamp, stream_id, data in log:
            pose_history.append((timestamp, deserialize(data)))

    depth_history = []
    with LogReader(log_path, only_stream_id=depth_stream) as log:
        for timestamp, stream_id, data in log:
            depth_history.append((timestamp, deserialize(data)))
            
    joint_history = []
    with LogReader(log_path, only_stream_id=joint_stream) as log:
        for timestamp, stream_id, data in log:
            joint_history.append((timestamp, deserialize(data)))

    orb = cv2.ORB_create(nfeatures=2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    decoder = VideoDecoder('hevc')

    last_frame_data = None
    errors = []
    plot_count = 0
    stats = {'total_frames': 0, 'no_pose_depth': 0, 'no_descriptors': 0, 'stationary': 0, 
             'low_matches': 0, 'insufficient_3d': 0, 'pnp_failed': 0, 'valid_samples': 0}
    
    print("Processing frames...")
    with LogReader(log_path, only_stream_id=color_stream) as log:
        for timestamp, stream_id, data in log:
            stats['total_frames'] += 1
            idx = stats['total_frames']
            
            if debug_frame != -1 and idx > debug_frame:
                break

            raw_data = deserialize(data)
            frame = decoder.decode(raw_data)
            if frame is None:
                continue

            pose = get_closest_data(timestamp, pose_history)
            depth = get_closest_data(timestamp, depth_history)
            joint = get_closest_data(timestamp, joint_history)
            
            if pose is None or depth is None or joint is None:
                stats['no_pose_depth'] += 1
                continue
                
            kp, des = orb.detectAndCompute(frame, None)
            if des is None or len(des) < 10:
                stats['no_descriptors'] += 1
                continue
                
            current_frame_data = {'frame': frame, 'kp': kp, 'des': des, 'pose': pose, 'depth': depth, 'joint': joint, 'timestamp': timestamp}
            is_debug = (idx == debug_frame)

            if last_frame_data is not None:
                # Robot pose: x, y in mm, heading in hundredths of degree
                x1, y1, h1 = last_frame_data['pose']
                x2, y2, h2 = current_frame_data['pose']
                dist_moved = math.hypot(x2 - x1, y2 - y1) / 1000.0
                
                if is_debug:
                    print(f"\n--- DEBUG FRAME {idx} ---")
                    print(f"Timestamp: {timestamp}")
                    print(f"Pose2d:    {pose}")
                    print(f"Joint:     {joint}")
                    print(f"Dist Moved: {dist_moved:.4f}m (threshold {min_dist}m)")

                if dist_moved >= min_dist or is_debug:
                    matches = bf.match(last_frame_data['des'], current_frame_data['des'])
                    if is_debug: print(f"ORB Matches: {len(matches)}")
                    
                    if len(matches) <= 20:
                        stats['low_matches'] += 1
                    else:
                        obj_pts, img_pts = [], []
                        d_h, d_w = last_frame_data['depth'].shape
                        f_h, f_w = last_frame_data['frame'].shape[:2]
                        
                        for m in matches:
                            kp_prev = last_frame_data['kp'][m.queryIdx]
                            u, v = int(kp_prev.pt[0]), int(kp_prev.pt[1])
                            ud, vd = int(u * d_w / f_w), int(v * d_h / f_h)
                            d = last_frame_data['depth'][vd, ud] if (0 <= vd < d_h and 0 <= ud < d_w) else 0
                            if d > 0:
                                z = d / 1000.0
                                obj_pts.append([(u-cx)*z/fx, (v-cy)*z/fy, z])
                                img_pts.append(current_frame_data['kp'][m.trainIdx].pt)
                                
                        if is_debug: print(f"Valid 3D Points: {len(obj_pts)}")

                        if len(obj_pts) < 15:
                            stats['insufficient_3d'] += 1
                        else:
                            obj_pts, img_pts = np.array(obj_pts, dtype=float), np.array(img_pts, dtype=float)
                            rvec, tvec, ret = None, None, False
                            
                            if use_pose:
                                yaw1, yaw2 = math.radians(h1/100.0), math.radians(h2/100.0)
                                j1, j2 = math.radians(last_frame_data['joint'][0]/100.0) + joint_offset, math.radians(current_frame_data['joint'][0]/100.0) + joint_offset

                                p1_w = np.array([x1 / 1000.0, y1 / 1000.0, 0])
                                p2_w = np.array([x2 / 1000.0, y2 / 1000.0, 0])

                                # World to Joint rotation
                                R_w_j1 = get_rotation_matrix(yaw1)
                                R_w_j2 = get_rotation_matrix(yaw2)

                                # Joint to Front rotation
                                R_j_f1 = get_rotation_matrix(j1)
                                R_j_f2 = get_rotation_matrix(j2)

                                # Combined World to Front rotation
                                R_w_f1 = R_w_j1 @ R_j_f1
                                R_w_f2 = R_w_j2 @ R_j_f2

                                # Camera World Position
                                # P_cam = P_joint + R_world_to_front @ MountOffset
                                P_c1_w = p1_w + R_w_f1 @ np.array(mount_offset)
                                P_c2_w = p2_w + R_w_f2 @ np.array(mount_offset)

                                # Camera World Rotation
                                R_c1_w = R_w_f1 @ R_front_to_cam
                                R_c2_w = R_w_f2 @ R_front_to_cam

                                # Relative transformation: Frame 1 to Frame 2 in Camera 2 coords
                                R_rel = R_c2_w.T @ R_c1_w
                                t_rel = R_c2_w.T @ (P_c1_w - P_c2_w)

                                rvec, _ = cv2.Rodrigues(R_rel)
                                tvec = t_rel.reshape(3, 1)
                                ret = True
                                if is_debug:
                                    print(f"Rel Translation (Cam Coords): {t_rel.flatten()}")
                                    angle = np.linalg.norm(rvec)
                                    print(f"Rel Rotation: {math.degrees(angle):.2f} deg")
                            else:
                                ret, rvec, tvec, inliers_indices = cv2.solvePnPRansac(obj_pts, img_pts, camera_matrix, dist_coeffs, reprojectionError=5.0, iterationsCount=100)
                                if ret: inliers = inliers_indices
                            
                            if not ret:
                                stats['pnp_failed'] += 1
                            else:
                                stats['valid_samples'] += 1
                                if use_pose: inliers = np.arange(len(obj_pts))
                                projected_pts, _ = cv2.projectPoints(obj_pts[inliers], rvec, tvec, camera_matrix, dist_coeffs)
                                projected_pts, actual_pts = projected_pts.reshape(-1, 2), img_pts[inliers].reshape(-1, 2)
                                err = np.linalg.norm(projected_pts - actual_pts, axis=1)
                                mean_err = np.mean(err)
                                errors.append(mean_err)
                                
                                if is_debug: print(f"Mean Reprojection Error: {mean_err:.2f} pixels")

                                if plot_count < num_plots or is_debug:
                                    vis_img = current_frame_data['frame'].copy()
                                    for i in range(len(actual_pts)):
                                        cv2.circle(vis_img, (int(actual_pts[i][0]), int(actual_pts[i][1])), 4, (0, 255, 0), 1)
                                        px, py = int(projected_pts[i][0]), int(projected_pts[i][1])
                                        cv2.line(vis_img, (px-4, py-4), (px+4, py+4), (0, 0, 255), 1)
                                        cv2.line(vis_img, (px+4, py-4), (px-4, py+4), (0, 0, 255), 1)
                                    out_name = f"debug_frame_{idx:03d}.png" if is_debug else f"debug_calib_{plot_count:02d}.png"
                                    cv2.imwrite(out_name, vis_img)
                                    print(f"Saved {out_name} (Error: {mean_err:.2f} px)")
                                    if not is_debug: plot_count += 1
                    
                    if is_debug: return # Exit after debug
                    last_frame_data = current_frame_data
                else:
                    stats['stationary'] += 1
            else:
                last_frame_data = current_frame_data
                
            if limit > 0 and len(errors) >= limit: break
                
    if debug_frame == -1 and errors:
        print(f"\nResults for {os.path.basename(log_path)} (dist threshold {min_dist}m):")
        print(f"  Total frames in log:    {stats['total_frames']}")
        print(f"  - No Data (Pose/Joint): {stats['no_pose_depth']}")
        print(f"  - No Descriptors:       {stats['no_descriptors']}")
        print(f"  - Stationary (<{min_dist}m):   {stats['stationary']}")
        print(f"  - Low ORB Matches:      {stats['low_matches']}")
        print(f"  - Insufficient Depth:   {stats['insufficient_3d']}")
        print(f"  - Math/PnP Failed:      {stats['pnp_failed']}")
        print(f"  = Valid Samples Used:   {stats['valid_samples']}")
        print(f"\n  Average Reprojection Error: {np.mean(errors):.2f} pixels")
        print(f"  Std Dev: {np.std(errors):.2f} pixels")
    else:
        print("No matches found to calculate error.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--plots", type=int, default=5, help="Number of visual plots to generate")
    parser.add_argument("--limit", type=int, default=50, help="Max number of samples to process (0 for all)")
    parser.add_argument("--dist", type=float, default=0.05, help="Min distance between frames (meters)")
    parser.add_argument("--use-pose", action="store_true", help="Use robot pose + joint instead of solvePnPRansac")
    parser.add_argument("--mount-x", type=float, default=0.0, help="Camera mounting X offset from joint (meters, forward)")
    parser.add_argument("--mount-y", type=float, default=0.0, help="Camera mounting Y offset (meters, left)")
    parser.add_argument("--mount-z", type=float, default=0.0, help="Camera mounting Z offset (meters, up)")
    parser.add_argument("--mount-pitch", type=float, default=0.0, help="Camera mounting pitch (degrees, down is positive)")
    parser.add_argument("--joint-offset", type=float, default=0.0, help="Joint angle calibration offset (degrees)")
    parser.add_argument("--debug-frame", type=int, default=-1)
    args = parser.parse_args()
    validate_calibration(args.logfile, args.plots, args.limit, args.dist, use_pose=args.use_pose, 
                         mount_offset=(args.mount_x, args.mount_y, args.mount_z),
                         mount_pitch=math.radians(args.mount_pitch),
                         joint_offset=math.radians(args.joint_offset),
                         debug_frame=args.debug_frame)
