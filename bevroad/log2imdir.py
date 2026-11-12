#!/usr/bin/env python3
"""
Extracts images from an OSGAR log file at specified distance intervals
along with a CSV metadata overview.
"""

import argparse
import math
import os
import sys
from pathlib import Path

import av
import cv2
import numpy as np
from osgar.logger import LogReaderEx


def main():
    parser = argparse.ArgumentParser(description='Extract camera frames from OSGAR logs at specific distance intervals')
    parser.add_argument('logfile', help='Path to the input OSGAR log file')
    parser.add_argument('-o', '--out', help='Output directory (defaults to logfile name without extension)')
    parser.add_argument('--start', type=float, default=1.0, help='Start distance in meters (default: 1.0)')
    parser.add_argument('--step', type=float, default=0.5, help='Distance step in meters (default: 0.5)')
    parser.add_argument('--end', type=float, default=10.0, help='End distance in meters (default: 10.0)')
    parser.add_argument('--camera', default='oak.color', help="Camera stream name (default: 'oak.color')")
    parser.add_argument('--pose', default='platform.pose2d', help="Pose2D stream name (default: 'platform.pose2d')")
    args = parser.parse_args()

    logfile = args.logfile
    if not os.path.exists(logfile):
        print(f"Error: Log file '{logfile}' does not exist.")
        sys.exit(1)

    # Determine default output directory
    if args.out:
        output_dir = args.out
    else:
        output_dir = str(Path(logfile).stem)

    print(f'Reading log: {logfile}')
    print(f'Output directory: {output_dir}')
    print(f'Intervals: start={args.start}m, step={args.step}m, end={args.end}m')
    print(f"Streams: camera='{args.camera}', pose='{args.pose}'")

    os.makedirs(output_dir, exist_ok=True)

    try:
        reader = LogReaderEx(logfile, names=[args.camera, args.pose])
    except ValueError as e:
        print(f'Error: Specified stream name not found in log file: {e}')
        sys.exit(1)

    extract_images_from_reader(reader, output_dir, args.start, args.step, args.end, args.camera, args.pose)


def extract_images_from_reader(reader, output_dir, start, step, end, camera_stream, pose_stream):
    csv_path = Path(output_dir) / 'overview.csv'
    csv_file = open(csv_path, 'w', encoding='utf-8')

    prev_pose = None
    accumulated_dist = 0.0
    target_dist = start
    img_idx = 0

    last_frame = None
    codec_ctx = None

    try:
        for dt, stream_name, data in reader:
            if stream_name == camera_stream:
                assert isinstance(data, bytes), f'Expected bytes for camera stream, got {type(data)}'

                if data.startswith(b'\xff\xd8'):  # JPEG
                    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                else:  # Video (HEVC / H264)
                    if codec_ctx is None:
                        # Auto-detect codec: H.265 (HEVC) or H.264
                        codec_name = 'hevc' if (b'\x46\x01' in data[:30] or b'\x40\x01' in data[:30]) else 'h264'
                        codec_ctx = av.CodecContext.create(codec_name, 'r')

                    try:
                        packets = codec_ctx.parse(data)
                        frame = None
                        for packet in packets:
                            frames = codec_ctx.decode(packet)
                            if frames:
                                frame = frames[-1].to_ndarray(format='bgr24')
                    except Exception as e:
                        print(f'Warning: frame decode failed: {e}')
                        frame = None

                if frame is not None:
                    last_frame = frame

            elif stream_name == pose_stream:
                # data is expected to be [x_mm, y_mm, heading_cd]
                assert isinstance(data, list), f'Expected list for pose stream, got {type(data)}'
                assert len(data) >= 3, f'Expected at least 3 elements in pose list, got {len(data)}'

                x_mm, y_mm, heading_cd = data[:3]
                x = x_mm / 1000.0
                y = y_mm / 1000.0
                heading = math.radians(heading_cd / 100.0)

                if prev_pose is not None:
                    px, py, _ = prev_pose
                    dx = x - px
                    dy = y - py
                    accumulated_dist += math.hypot(dx, dy)

                prev_pose = (x, y, heading)

                # Check if we crossed the next target distance
                while accumulated_dist >= target_dist and target_dist <= end:
                    if last_frame is not None:
                        img_filename = f'img-{img_idx:04d}.jpg'
                        img_path = os.path.join(output_dir, img_filename)
                        cv2.imwrite(img_path, last_frame)

                        ts_sec = dt.total_seconds()
                        csv_file.write(f'{img_filename},{ts_sec:.3f},{x:.4f},{y:.4f},{heading:.6f}\n')
                        csv_file.flush()

                        print(
                            f'Extracted {img_filename} at distance {accumulated_dist:.2f}m (target {target_dist:.2f}m)'
                        )
                        img_idx += 1
                    else:
                        print(f'Warning: Reached target {target_dist:.2f}m but no camera frame has been decoded yet.')

                    target_dist += step

                if target_dist > end:
                    print('Reached final target distance limit.')
                    break

    except KeyboardInterrupt:
        print('\nProcess interrupted by user.')
    finally:
        csv_file.close()
        print(f'Finished. Extracted {img_idx} images. Metadata stored in {csv_path}')
    return img_idx


if __name__ == '__main__':
    main()
