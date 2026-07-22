#!/usr/bin/env python3
"""
Bird's Eye View (BEV) road mapping - Mosaic Stitching.
Stitches multiple BEV-warped images into a global map using pose metadata.
"""

import argparse
import os
import sys

import cv2
import numpy as np

from bevroad.utils import load_calibration, parse_csv, transform_point


def add_common_arguments(parser, include_tolerance=False, include_margin=False):
    """
    Adds common perspective, calibration, and mapping command-line arguments to an ArgumentParser.
    """
    parser.add_argument(
        '--config', default=None, help='Path to calibration bev.json file (defaults to matching JSON in CSV folder)'
    )
    parser.add_argument(
        '--road-width', type=float, default=2.0, help='Physical width of the road in meters (default: 2.0)'
    )
    parser.add_argument(
        '--lane-width-fraction',
        type=float,
        default=0.5,
        help='Fraction of the BEV width that the road width occupies (default: 0.5)',
    )
    parser.add_argument(
        '--near',
        type=float,
        default=1.0,
        help='Forward distance of the BEV image bottom edge from the robot center, in meters (default: 1.0)',
    )
    parser.add_argument(
        '--resolution',
        type=float,
        default=0.02,
        help='Resolution of the mosaic image in meters per pixel (default: 0.02)',
    )
    if include_tolerance:
        parser.add_argument(
            '--tolerance',
            type=float,
            default=15.0,
            help='Grayscale tolerance threshold (0-255) for pixel alignment comparison (default: 15.0)',
        )
    if include_margin:
        parser.add_argument(
            '--margin',
            type=float,
            default=0.0,
            help='Horizontal margin in meters to extend the BEV image on each side (default: 0.0)',
        )


def main():
    parser = argparse.ArgumentParser(description='BEV Mosaic Map Stitcher')
    parser.add_argument('imdir', help='Directory containing overview.csv and extracted images')

    add_common_arguments(parser, include_margin=True)

    parser.add_argument('-o', '--out', help='Output image path (defaults to mosaic.png inside input directory)')
    args = parser.parse_args()

    imdir = args.imdir
    csv_path = os.path.join(imdir, 'overview.csv')
    records = parse_csv(csv_path)

    if not records:
        print('Error: No valid records found in the overview CSV.')
        sys.exit(1)

    # Determine calibration config file path
    if args.config:
        config_path = args.config
    else:
        # Look for any JSON file in the folder or a default bev_config.json
        json_files = [f for f in os.listdir(imdir) if f.endswith('.json')]
        if json_files:
            config_path = os.path.join(imdir, json_files[0])
        else:
            # Fallback search in parent directory or default
            config_path = 'bev_config.json'
            if not os.path.exists(config_path):
                print('Error: No calibration config file specified, and no JSON files found in input folder.')
                sys.exit(1)

    print(f'Loading calibration from: {config_path}')
    calib = load_calibration(config_path)

    # Extract transformation matrix M and dimensions from calib
    if 'matrix_M' not in calib or 'bev_width' not in calib or 'bev_height' not in calib:
        print('Error: Calibration config is missing required fields (matrix_M, bev_width, bev_height).')
        sys.exit(1)

    M_orig = np.array(calib['matrix_M'], dtype=np.float32)
    bev_w_orig = calib['bev_width']
    bev_h = calib['bev_height']

    # Calculate local spatial calibration
    # meters_per_pixel of BEV image
    m_per_pixel = args.road_width / (bev_w_orig * args.lane_width_fraction)
    print(f'BEV local scale: {m_per_pixel:.4f} meters/pixel')

    # Apply margin to expand horizontal view if requested
    margin_pixels = int(args.margin / m_per_pixel) if args.margin > 0.0 else 0
    if margin_pixels > 0:
        # Translation matrix to shift output by margin_pixels to the right
        T = np.array([[1, 0, margin_pixels], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        M = T @ M_orig
        bev_w = bev_w_orig + 2 * margin_pixels
        print(f'Applying horizontal margin: {args.margin}m ({margin_pixels}px on each side). New BEV width: {bev_w}px')
    else:
        M = M_orig
        bev_w = bev_w_orig

    # Local dimensions in meters
    bev_w_m = bev_w * m_per_pixel
    bev_h_m = bev_h * m_per_pixel
    d_near = args.near
    d_far = d_near + bev_h_m

    # local corner coordinates (relative to robot center)
    # X is forward, Y is left
    local_corners = [
        (d_far, bev_w_m / 2.0),  # Top-Left
        (d_far, -bev_w_m / 2.0),  # Top-Right
        (d_near, -bev_w_m / 2.0),  # Bottom-Right
        (d_near, bev_w_m / 2.0),  # Bottom-Left
    ]

    # Find global bounding box of all warped images
    print('Calculating global bounding box...')
    g_xs = []
    g_ys = []

    for rec in records:
        rx, ry, heading = rec['x'], rec['y'], rec['heading']
        for lx, ly in local_corners:
            gx, gy = transform_point(lx, ly, rx, ry, heading)
            g_xs.append(gx)
            g_ys.append(gy)

    x_min, x_max = min(g_xs), max(g_xs)
    y_min, y_max = min(g_ys), max(g_ys)

    print(f'Global bounds (meters): X = [{x_min:.2f}, {x_max:.2f}], Y = [{y_min:.2f}, {y_max:.2f}]')

    # Define mosaic canvas dimensions in pixels
    res = args.resolution
    canvas_w = int((y_max - y_min) / res) + 1
    canvas_h = int((x_max - x_min) / res) + 1

    print(f'Creating global mosaic canvas of size: {canvas_w}x{canvas_h} pixels')
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    # Process and stitch each image
    for idx, rec in enumerate(records):
        filename = rec['filename']
        img_path = os.path.join(imdir, filename)
        if not os.path.exists(img_path):
            print(f"Warning: Image '{filename}' is missing. Skipping.")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read image '{filename}'. Skipping.")
            continue

        # 1. Warp input perspective image to local BEV space
        bev_img = cv2.warpPerspective(img, M, (bev_w, bev_h))

        # 2. Map BEV pixels to global canvas pixels using an affine transform
        # We define 3 control points on the BEV image
        pts_src = np.array(
            [
                [0, 0],  # Top-Left of BEV
                [bev_w - 1, 0],  # Top-Right of BEV
                [0, bev_h - 1],  # Bottom-Left of BEV
            ],
            dtype=np.float32,
        )

        # Map these 3 control points to global coordinates, and then to canvas pixel coordinates
        pts_dst = []
        rx, ry, heading = rec['x'], rec['y'], rec['heading']

        # Point 1 (Top-Left): local (d_far, bev_w_m / 2.0)
        gx1, gy1 = transform_point(d_far, bev_w_m / 2.0, rx, ry, heading)
        # Point 2 (Top-Right): local (d_far, -bev_w_m / 2.0)
        gx2, gy2 = transform_point(d_far, -bev_w_m / 2.0, rx, ry, heading)
        # Point 3 (Bottom-Left): local (d_near, bev_w_m / 2.0)
        gx3, gy3 = transform_point(d_near, bev_w_m / 2.0, rx, ry, heading)

        # Convert global coordinates (meters) to canvas pixel coordinates
        # Row corresponding to X (X increases upwards on screen, so top of screen is X_max)
        # Col corresponding to Y (Y increases leftwards on screen, so left of screen is Y_max)
        for gx, gy in [(gx1, gy1), (gx2, gy2), (gx3, gy3)]:
            row = (x_max - gx) / res
            col = (y_max - gy) / res
            pts_dst.append([col, row])

        pts_dst = np.array(pts_dst, dtype=np.float32)

        # Compute affine transform matrix from BEV to canvas
        M_affine = cv2.getAffineTransform(pts_src, pts_dst)

        # Warp local BEV image onto the global canvas size
        warped = cv2.warpAffine(bev_img, M_affine, (canvas_w, canvas_h), flags=cv2.INTER_LINEAR)

        # Overwrite global canvas (Option A)
        mask = (warped > 0).any(axis=2)
        canvas[mask] = warped[mask]

        if (idx + 1) % 10 == 0 or (idx + 1) == len(records):
            print(f'Stitched {idx + 1}/{len(records)} images...')

    # Save output mosaic map
    output_path = args.out if args.out else os.path.join(imdir, 'mosaic.png')
    cv2.imwrite(output_path, canvas)
    print(f'Successfully generated global mosaic: {output_path}')


if __name__ == '__main__':
    main()
