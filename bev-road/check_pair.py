#!/usr/bin/env python3
"""
  BEV Road Mapping - Consecutive Pair Stitching & Alignment Validator.
  Loads two sequential images, warps them to BEV space, and calculates 
  the geometric alignment displacement in meters, while filtering out 
  dynamic objects (the leading robot) and 3D out-of-plane elements.
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np


def load_calibration(config_path):
    if not os.path.exists(config_path):
        print(f"Error: Calibration config file '{config_path}' does not exist.")
        sys.exit(1)
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading calibration config {config_path}: {e}")
        sys.exit(1)


def parse_csv(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: Metadata CSV file '{csv_path}' does not exist.")
        sys.exit(1)

    records = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) < 5:
                continue
            try:
                filename = parts[0]
                ts = float(parts[1])
                x = float(parts[2])
                y = float(parts[3])
                heading = float(parts[4])
                records.append({
                    'filename': filename,
                    'ts': ts,
                    'x': x,
                    'y': y,
                    'heading': heading
                })
            except ValueError:
                pass
    return records


def transform_point(x_local, y_local, robot_x, robot_y, heading):
    """
    Transforms local robot-frame coordinates (X=forward, Y=left)
    to global coordinates based on robot's position and heading (radians).
    """
    c = math.cos(heading)
    s = math.sin(heading)
    x_global = robot_x + x_local * c - y_local * s
    y_global = robot_y + x_local * s + y_local * c
    return x_global, y_global


def main():
    parser = argparse.ArgumentParser(
        description="Verify stitching & alignment quality of two consecutive frames"
    )
    parser.add_argument("csv_path", help="Path to overview.csv metadata file")
    parser.add_argument("index", type=int, help="Index of the first image (0-based)")
    parser.add_argument(
        "--config", default=None,
        help="Path to calibration bev.json file (defaults to matching JSON in CSV folder)"
    )
    parser.add_argument(
        "--road-width", type=float, default=2.0,
        help="Physical width of the road in meters (default: 2.0)"
    )
    parser.add_argument(
        "--lane-width-fraction", type=float, default=0.5,
        help="Fraction of the BEV width that the road width occupies (default: 0.5)"
    )
    parser.add_argument(
        "--near", type=float, default=1.0,
        help="Forward distance of the BEV image bottom edge from the robot center, in meters (default: 1.0)"
    )
    parser.add_argument(
        "--no-vis", action="store_true",
        help="Disable visualization window and only output statistics"
    )
    parser.add_argument(
        "--save-debug", action="store_true",
        help="Save a visualization match image to the disk"
    )
    args = parser.parse_args()

    # Locate CSV file and directories
    csv_path = Path(args.csv_path)
    imdir = csv_path.parent

    # Parse records
    records = parse_csv(csv_path)
    if not records:
        print("Error: No valid records found in overview CSV.")
        sys.exit(1)

    if args.index < 0 or args.index >= len(records) - 1:
        print(f"Error: Index {args.index} is out of bounds for {len(records)} records (0 to {len(records)-2}).")
        sys.exit(1)

    rec1 = records[args.index]
    rec2 = records[args.index + 1]

    # Paths to image files
    img1_path = imdir / rec1['filename']
    img2_path = imdir / rec2['filename']

    print(f"Analyzing consecutive pair: index {args.index} and {args.index + 1}")
    print(f"  Frame 1: {rec1['filename']} at pose ({rec1['x']:.3f}, {rec1['y']:.3f}, heading={rec1['heading']:.4f} rad)")
    print(f"  Frame 2: {rec2['filename']} at pose ({rec2['x']:.3f}, {rec2['y']:.3f}, heading={rec2['heading']:.4f} rad)")

    # Read images
    img1 = cv2.imread(str(img1_path))
    img2 = cv2.imread(str(img2_path))

    if img1 is None or img2 is None:
        print(f"Error: Failed to read image files: {img1_path} or {img2_path}")
        sys.exit(1)

    # Determine calibration config file path
    if args.config:
        config_path = args.config
    else:
        # Search for any JSON file in the same directory as the CSV
        json_files = list(imdir.glob("*.json"))
        if json_files:
            config_path = str(json_files[0])
        else:
            config_path = "bev_config.json"

    print(f"Loading calibration config: {config_path}")
    calib = load_calibration(config_path)

    # Validate calibration fields
    for field in ["matrix_M", "bev_width", "bev_height"]:
        if field not in calib:
            print(f"Error: Calibration JSON is missing required field '{field}'")
            sys.exit(1)

    M = np.array(calib["matrix_M"], dtype=np.float32)
    bev_w = calib["bev_width"]
    bev_h = calib["bev_height"]

    # Calculate local spatial parameters
    m_per_pixel = args.road_width / (bev_w * args.lane_width_fraction)
    bev_w_m = bev_w * m_per_pixel
    d_near = args.near

    # 1. Warp both input images to local BEV space
    bev1 = cv2.warpPerspective(img1, M, (bev_w, bev_h))
    bev2 = cv2.warpPerspective(img2, M, (bev_w, bev_h))

    # 2. Design the "Anomalies/3D/Occlusion Filter" Mask
    # - Top 40% of BEV is far-field where 3D trees and far leading robot reside -> mask out
    # - Middle 30% width corridor in Y=[40% to 75%] where leading robot actively drives -> mask out
    # - This keeps flat near-field ground + outer margins (lane markings/grass boundaries) where features are static.
    mask = np.full((bev_h, bev_w), 255, dtype=np.uint8)
    
    # Crop out the far field (top 40%)
    far_field_h = int(bev_h * 0.4)
    mask[0:far_field_h, :] = 0

    # Crop out the central leading-robot corridor (middle 30% from 40% height to 75% height)
    corridor_start_y = far_field_h
    corridor_end_y = int(bev_h * 0.75)
    cx = bev_w // 2
    corridor_w = int(bev_w * 0.3)
    mask[corridor_start_y:corridor_end_y, cx - corridor_w // 2 : cx + corridor_w // 2] = 0

    # 3. Detect and Match Keypoints (ORB with spatial masks)
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, des1 = orb.detectAndCompute(bev1, mask)
    kp2, des2 = orb.detectAndCompute(bev2, mask)

    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        print("Warning: Insufficient keypoints detected in the static ground mask to evaluate alignment.")
        sys.exit(0)

    # Brute Force Matcher with Cross Check (ensures mutual consistency)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # Sort matches by distance (best first)
    matches = sorted(matches, key=lambda x: x.distance)

    # Limit to top 150 best matches to keep it robust
    matches = matches[:150]

    if not matches:
        print("Error: No keypoint matches found in the static overlap region.")
        sys.exit(1)

    # 4. Compute Global Spatial Displacement (Warping keypoints to global map frame)
    displacements = []
    points_to_draw = []

    for m in matches:
        # Local pixel coordinates in BEV space
        col1, row1 = kp1[m.queryIdx].pt
        col2, row2 = kp2[m.trainIdx].pt

        # Map to local robot-frame coordinates (meters, X=forward, Y=left)
        # Pixel (0, bev_h-1) maps to (d_near, bev_w_m/2)
        xl1 = d_near + (bev_h - 1 - row1) * m_per_pixel
        yl1 = (bev_w_m / 2.0) - col1 * m_per_pixel

        xl2 = d_near + (bev_h - 1 - row2) * m_per_pixel
        yl2 = (bev_w_m / 2.0) - col2 * m_per_pixel

        # Map to global coordinates based on respective robot poses
        gx1, gy1 = transform_point(xl1, yl1, rec1['x'], rec1['y'], rec1['heading'])
        gx2, gy2 = transform_point(xl2, yl2, rec2['x'], rec2['y'], rec2['heading'])

        # Calculate spatial displacement error in meters
        disp_m = math.hypot(gx1 - gx2, gy1 - gy2)
        displacements.append(disp_m)
        points_to_draw.append(((col1, row1), (col2, row2), disp_m))

    displacements = np.array(displacements)
    mean_disp = np.mean(displacements)
    median_disp = np.median(displacements)
    std_disp = np.std(displacements)
    min_disp = np.min(displacements)
    max_disp = np.max(displacements)
    rmse_disp = np.sqrt(np.mean(displacements**2))

    # Print Detailed Statistical Analysis
    print("\nAlignment Quality Evaluation Report:")
    print("-----------------------------------------")
    print(f"  Valid Matches Evaluated : {len(displacements)}")
    print(f"  Mean Alignment Error    : {mean_disp * 100:.2f} cm ({mean_disp:.4f} m)")
    print(f"  Median Alignment Error  : {median_disp * 100:.2f} cm ({median_disp:.4f} m)")
    print(f"  RMSE (Root Mean Square) : {rmse_disp * 100:.2f} cm ({rmse_disp:.4f} m)")
    print(f"  Standard Deviation (SD) : {std_disp * 100:.2f} cm ({std_disp:.4f} m)")
    print(f"  Error Range             : [{min_disp * 100:.2f} cm, {max_disp * 100:.2f} cm]")

    # Classification of Stitching Quality
    # - Excellent: < 5 cm
    # - Acceptable: 5 - 12 cm
    # - Poor/Needs Calibration check: > 12 cm
    print("  Stitching Quality Rating: ", end="")
    if rmse_disp < 0.05:
        print("EXCELLENT (Highly Consistent)")
    elif rmse_disp < 0.12:
        print("ACCEPTABLE (Minor Drift/Slip)")
    else:
        print("POOR (Needs Calibration or Pose/Sync Check)")
    print("-----------------------------------------\n")

    # 5. Visualization
    # Create side-by-side visualization of BEV images with match lines
    bev_h_vis, bev_w_vis = bev1.shape[:2]
    vis_img = np.zeros((bev_h_vis, bev_w_vis * 2, 3), dtype=np.uint8)
    vis_img[:, :bev_w_vis] = bev1
    vis_img[:, bev_w_vis:] = bev2

    # Draw the static evaluation mask outline in semi-transparent red on both images
    mask_vis = np.zeros_like(bev1)
    mask_vis[mask == 0] = [0, 0, 100] # Dim red
    vis_img[:, :bev_w_vis] = cv2.addWeighted(vis_img[:, :bev_w_vis], 0.8, mask_vis, 0.2, 0)
    vis_img[:, bev_w_vis:] = cv2.addWeighted(vis_img[:, bev_w_vis:], 0.8, mask_vis, 0.2, 0)

    # Draw match points and connection lines
    # Color matches based on error: Green (Excellent, <5cm), Yellow (Acceptable, <12cm), Red (Poor)
    for (pt1, pt2, err) in points_to_draw:
        p1 = (int(pt1[0]), int(pt1[1]))
        p2 = (int(pt2[0] + bev_w_vis), int(pt2[1]))

        if err < 0.05:
            color = (0, 255, 0)  # Green
        elif err < 0.12:
            color = (0, 255, 255) # Yellow
        else:
            color = (0, 0, 255)  # Red

        cv2.circle(vis_img, p1, 4, color, -1)
        cv2.circle(vis_img, p2, 4, color, -1)
        cv2.line(vis_img, p1, p2, color, 1)

    # Overlay Text stats
    cv2.putText(
        vis_img, f"Frame {args.index} BEV (with Mask)", (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA
    )
    cv2.putText(
        vis_img, f"Frame {args.index+1} BEV (with Mask)", (bev_w_vis + 10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA
    )
    cv2.putText(
        vis_img, f"Mean Error: {mean_disp*100:.1f}cm | RMSE: {rmse_disp*100:.1f}cm", (10, bev_h_vis - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA
    )

    # Save to disk if requested
    if args.save_debug:
        debug_out_path = imdir / f"alignment_check_{args.index}_{args.index+1}.png"
        cv2.imwrite(str(debug_out_path), vis_img)
        print(f"Saved match visualization to: {debug_out_path}")

    # Display window
    if not args.no_vis:
        win_name = "BEV Alignment Validator (Consecutive Frame Pair)"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.imshow(win_name, vis_img)
        print("Press any key in the GUI window to exit...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
