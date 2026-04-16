import cv2
import os
import math
import subprocess
import tempfile
import re
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

def get_closest_pose(ts, pose_history):
    if not pose_history:
        return None
    best_pose = pose_history[0][1]
    min_diff = abs((ts - pose_history[0][0]).total_seconds())
    for p_ts, p_data in pose_history:
        diff = abs((ts - p_ts).total_seconds())
        if diff < min_diff:
            min_diff = diff
            best_pose = p_data
        elif diff > min_diff:
            break
    return best_pose

def extract_reference_data(log_path, step_meters=0.2, min_brightness=30.0, orb=None, debug_dir=None):
    """
    Extracts poses and ORB descriptors from an OSGAR log.
    Returns: list of {'kp': keypoints, 'des': descriptors, 'pose': (x, y)}
    """
    if debug_dir and not os.path.exists(debug_dir):
        os.makedirs(debug_dir)

    if orb is None:
        orb = cv2.ORB_create(nfeatures=2000)

    # 1. Get raw video for processing
    with tempfile.NamedTemporaryFile(suffix='.h265', delete=False) as tmp:
        tmp_name = tmp.name
    
    print(f"Extracting video from {log_path}...")
    color_stream = lookup_stream_id(log_path, "oak.color")
    with open(tmp_name, 'wb') as f:
        with LogReader(log_path, only_stream_id=color_stream) as log:
            for timestamp, stream_id, data in log:
                f.write(deserialize(data))
    
    cap = cv2.VideoCapture(tmp_name)
    if not cap.isOpened():
        print("Failed to open video stream.")
        os.unlink(tmp_name)
        return []

    # 2. Extract pose2d data with timestamps
    pose_stream = lookup_stream_id(log_path, "platform.pose2d")
    color_stream = lookup_stream_id(log_path, "oak.color")
    
    pose_history = []
    with LogReader(log_path, only_stream_id=pose_stream) as log:
        for timestamp, stream_id, data in log:
            pose_history.append((timestamp, deserialize(data)))

    # 3. Second pass: correlate and extract features
    print("Extracting visual landmarks...")
    ref_data = []
    last_x, last_y = None, None
    frame_idx = 0
    
    with LogReader(log_path, only_stream_id=color_stream) as log:
        for timestamp, stream_id, data in log:
            ret, frame = cap.read()
            if not ret:
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness = cv2.mean(gray)[0]
            
            if brightness < min_brightness:
                frame_idx += 1
                continue

            pose = get_closest_pose(timestamp, pose_history)
            if pose:
                x, y, h = pose[0]/1000.0, pose[1]/1000.0, math.radians(pose[2]/100.0)
                if last_x is None or math.hypot(x - last_x, y - last_y) >= step_meters:
                    kp, des = orb.detectAndCompute(frame, None)
                    if des is not None:
                        ref_data.append({'kp': kp, 'des': des, 'pose': (x, y, h), 'frame': frame.copy()})

                        if debug_dir:
                            img_name = f"frame_{frame_idx:06d}_x{x:.2f}_y{y:.2f}.png"
                            cv2.imwrite(os.path.join(debug_dir, img_name), frame)
                    last_x, last_y = x, y
            frame_idx += 1

    cap.release()
    os.unlink(tmp_name)
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
