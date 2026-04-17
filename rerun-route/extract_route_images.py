import cv2
import os
import math
import subprocess
import tempfile
import re
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

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

def extract_reference_data(log_path, step_meters=0.2, min_brightness=30.0, orb=None, debug_dir=None):
    """
    Extracts poses, ORB descriptors, and 3D keypoints from an OSGAR log.
    Returns: list of {'kp': keypoints, 'des': descriptors, 'pose': (x, y, h), 'kp_3d': list of (X,Y,Z)}
    """
    if debug_dir and not os.path.exists(debug_dir):
        os.makedirs(debug_dir)

    if orb is None:
        orb = cv2.ORB_create(nfeatures=2000)

    # OAK-D THE_1080_P approximate intrinsics
    fx, fy = 1400.0, 1400.0
    cx, cy = 960.0, 540.0

    print(f"Extracting video from {log_path}...")
    color_stream = lookup_stream_id(log_path, "oak.color")
    pose_stream = lookup_stream_id(log_path, "platform.pose2d")
    depth_stream = lookup_stream_id(log_path, "oak.depth")

    # 1. Extract pose2d and depth data with timestamps
    pose_history = []
    with LogReader(log_path, only_stream_id=pose_stream) as log:
        for timestamp, stream_id, data in log:
            pose_history.append((timestamp, deserialize(data)))

    depth_history = []
    try:
        with LogReader(log_path, only_stream_id=depth_stream) as log:
            for timestamp, stream_id, data in log:
                depth_history.append((timestamp, deserialize(data)))
    except Exception as e:
        print(f"Warning: No depth stream found ({e}). Translation refinement will be limited.")

    # 2. Correlate and extract features via in-memory decoding
    print("Extracting visual landmarks...")
    import av
    codec = av.CodecContext.create('hevc', 'r')

    ref_data = []
    last_x, last_y = None, None
    frame_idx = 0

    with LogReader(log_path, only_stream_id=color_stream) as log:
        for timestamp, stream_id, data in log:
            raw_data = deserialize(data)
            try:
                packets = codec.parse(raw_data)
            except av.AVError:
                continue

            frame = None
            for packet in packets:
                try:
                    frames = codec.decode(packet)
                    if frames:
                        frame = frames[-1].to_ndarray(format='bgr24')
                except av.AVError:
                    continue

            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness = cv2.mean(gray)[0]
            
            if brightness < min_brightness:
                frame_idx += 1
                continue

            pose = get_closest_data(timestamp, pose_history)
            if pose:
                x, y, h = pose[0]/1000.0, pose[1]/1000.0, math.radians(pose[2]/100.0)
                if last_x is None or math.hypot(x - last_x, y - last_y) >= step_meters:
                    kp, des = orb.detectAndCompute(frame, None)
                    if des is not None:
                        depth_frame = get_closest_data(timestamp, depth_history)
                        kp_3d = []
                        if depth_frame is not None:
                            d_h, d_w = depth_frame.shape
                            f_h, f_w = frame.shape[:2]
                            for k in kp:
                                u, v = int(k.pt[0]), int(k.pt[1])
                                # Map to depth coordinates
                                ud, vd = int(u * d_w / f_w), int(v * d_h / f_h)
                                d = depth_frame[vd, ud] if (0 <= vd < d_h and 0 <= ud < d_w) else 0
                                if d > 0:
                                    z = d / 1000.0
                                    xc = (u - cx) * z / fx
                                    yc = (v - cy) * z / fy
                                    kp_3d.append((xc, yc, z))
                                else:
                                    kp_3d.append(None)
                        
                        ref_data.append({
                            'kp': kp, 
                            'des': des, 
                            'pose': (x, y, h), 
                            'kp_3d': kp_3d,
                            'frame': frame.copy()
                        })

                        if debug_dir:
                            img_name = f"frame_{frame_idx:06d}_x{x:.2f}_y{y:.2f}.png"
                            cv2.imwrite(os.path.join(debug_dir, img_name), frame)
                    last_x, last_y = x, y
            frame_idx += 1

    print(f"Extracted {len(ref_data)} visual landmarks.")
    return ref_data

def extract_route_images(log_path, output_dir, step_meters=0.1, min_brightness=30):
    # Backward compatibility for the CLI tool
    extract_reference_data(log_path, step_meters, min_brightness, debug_dir=output_dir)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--out", default="rerun-route/data/reference_frames")
    parser.add_argument("--step", type=float, default=0.1)
    parser.add_argument("--min-brightness", type=float, default=30.0)
    args = parser.parse_args()
    
    extract_route_images(args.logfile, args.out, args.step, args.min_brightness)
