#!/usr/bin/env python3
"""
Bird's Eye View (BEV) road mapping prototype.
Allows interactive perspective transform calibration using trackbars.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np


def load_config(config_path):
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f'Error loading config {config_path}: {e}')
    return {}


def save_config(config_path, data):
    try:
        with open(config_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f'Config successfully saved to: {config_path}')
    except Exception as e:
        print(f'Error saving config {config_path}: {e}')


def main():
    parser = argparse.ArgumentParser(description='BEV Road Mapping Interactive Prototype')
    parser.add_argument('image', help='Path to the input road image')
    parser.add_argument('--config', help='Path to config JSON file for storing/loading calibration', default=None)
    parser.add_argument('--width', type=int, default=400, help='Output BEV image width')
    parser.add_argument('--height', type=int, default=600, help='Output BEV image height')
    args = parser.parse_args()

    image_path = args.image
    if not os.path.exists(image_path):
        print(f"Error: Image file '{image_path}' does not exist.")
        sys.exit(1)

    # Load original image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Failed to read image '{image_path}'.")
        sys.exit(1)

    h, w = img.shape[:2]
    print(f'Loaded image: {image_path} ({w}x{h})')

    # Determine config file path
    if args.config:
        config_path = args.config
    else:
        # Default config based on image name
        base, _ = os.path.splitext(image_path)
        config_path = f'{base}_bev.json'

    # Load existing calibration if available
    config = load_config(config_path)

    # Output BEV dimensions
    bev_w = args.width
    bev_h = args.height

    # Default values for trackbars
    default_top_y = config.get('top_y', int(h * 0.5))
    default_bottom_y = config.get('bottom_y', int(h * 0.95))
    default_top_width = config.get('top_width', int(w * 0.3))
    default_bottom_width = config.get('bottom_width', int(w * 0.8))
    default_offset_x = config.get('offset_x', w // 2)
    default_show_grid = config.get('show_grid', 1)

    # Setup OpenCV Windows
    src_win = 'Source Image (Calibration)'
    bev_win = "Bird's Eye View (BEV)"
    cv2.namedWindow(src_win, cv2.WINDOW_NORMAL)
    cv2.namedWindow(bev_win, cv2.WINDOW_NORMAL)

    # Define trackbar callback (does nothing, we query values in loop)
    def nothing(x):
        pass

    # Create trackbars
    cv2.createTrackbar('Top Y', src_win, default_top_y, h - 1, nothing)
    cv2.createTrackbar('Bottom Y', src_win, default_bottom_y, h - 1, nothing)
    cv2.createTrackbar('Top Width', src_win, default_top_width, w, nothing)
    cv2.createTrackbar('Bottom Width', src_win, default_bottom_width, w, nothing)
    cv2.createTrackbar('Offset X', src_win, default_offset_x, w, nothing)
    cv2.createTrackbar('Show Grid (0/1)', src_win, default_show_grid, 1, nothing)

    print('\nInteractive BEV Calibration Instructions:')
    print('-----------------------------------------')
    print('  Use trackbars to adjust the perspective trapezoid.')
    print('  Keep road lane lines parallel to the vertical grid lines in BEV view.')
    print("  Press 's' to save the current calibration parameters.")
    print("  Press 'r' to reset trackbars to defaults.")
    print("  Press 'q' or 'ESC' to exit.")
    print(f'Calibration will be saved to: {config_path}\n')

    while True:
        # Get current trackbar positions
        top_y = cv2.getTrackbarPos('Top Y', src_win)
        bottom_y = cv2.getTrackbarPos('Bottom Y', src_win)
        top_width = cv2.getTrackbarPos('Top Width', src_win)
        bottom_width = cv2.getTrackbarPos('Bottom Width', src_win)
        offset_x = cv2.getTrackbarPos('Offset X', src_win)
        show_grid = cv2.getTrackbarPos('Show Grid (0/1)', src_win)

        # Calculate offset shift from image center
        cx = w // 2
        tx = cx + (offset_x - cx)

        # Define 4 source points of the perspective trapezoid
        pt_tl = [tx - top_width // 2, top_y]
        pt_tr = [tx + top_width // 2, top_y]
        pt_br = [tx + bottom_width // 2, bottom_y]
        pt_bl = [tx - bottom_width // 2, bottom_y]

        pts_src = np.array([pt_tl, pt_tr, pt_br, pt_bl], dtype=np.float32)

        # Define 4 destination points of the BEV rectangle
        pts_dst = np.array([[0, 0], [bev_w - 1, 0], [bev_w - 1, bev_h - 1], [0, bev_h - 1]], dtype=np.float32)

        # Calculate homography matrix and warp
        M = cv2.getPerspectiveTransform(pts_src, pts_dst)
        bev_img = cv2.warpPerspective(img, M, (bev_w, bev_h))

        # Draw grid on BEV to assist alignment
        if show_grid:
            grid_color = (255, 0, 0)  # Blue grid
            # Draw vertical grid lines
            for pct in [0.25, 0.5, 0.75]:
                gx = int(bev_w * pct)
                cv2.line(bev_img, (gx, 0), (gx, bev_h), grid_color, 1)
            # Draw a few horizontal lines for perspective verification
            for pct in [0.25, 0.5, 0.75]:
                gy = int(bev_h * pct)
                cv2.line(bev_img, (0, gy), (bev_w, gy), grid_color, 1)

        # Draw calibration visualization on original image
        overlay = img.copy()
        pts_poly = np.array([pt_tl, pt_tr, pt_br, pt_bl], dtype=np.int32)

        # Fill trapezoid with semi-transparent green
        cv2.fillPoly(overlay, [pts_poly], (0, 255, 0))
        img_vis = cv2.addWeighted(img, 0.7, overlay, 0.3, 0)

        # Draw outline and points
        cv2.polylines(img_vis, [pts_poly], isClosed=True, color=(0, 255, 0), thickness=2)
        for pt in [pt_tl, pt_tr, pt_br, pt_bl]:
            cv2.circle(img_vis, (int(pt[0]), int(pt[1])), 5, (0, 0, 255), -1)

        # Show images
        cv2.imshow(src_win, img_vis)
        cv2.imshow(bev_win, bev_img)

        # Handle keyboard input
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or ESC
            break
        elif key == ord('s'):
            # Save configuration
            calib_data = {
                'top_y': top_y,
                'bottom_y': bottom_y,
                'top_width': top_width,
                'bottom_width': bottom_width,
                'offset_x': offset_x,
                'show_grid': show_grid,
                'bev_width': bev_w,
                'bev_height': bev_h,
                'matrix_M': M.tolist(),
            }
            save_config(config_path, calib_data)
        elif key == ord('r'):
            # Reset to defaults
            cv2.setTrackbarPos('Top Y', src_win, int(h * 0.5))
            cv2.setTrackbarPos('Bottom Y', src_win, int(h * 0.95))
            cv2.setTrackbarPos('Top Width', src_win, int(w * 0.3))
            cv2.setTrackbarPos('Bottom Width', src_win, int(w * 0.8))
            cv2.setTrackbarPos('Offset X', src_win, w // 2)
            cv2.setTrackbarPos('Show Grid (0/1)', src_win, 1)
            print('Trackbars reset to default positions.')

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
