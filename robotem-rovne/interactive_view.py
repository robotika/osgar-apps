import os
import sys

import av
import cv2
import numpy as np
from osgar.logger import LogReaderEx


def load_log_data(logfile):
    print(f'Loading and decoding {logfile}...')
    streams = ['oak.depth', 'oak.nn_mask', 'oak.color']

    # We'll store processed side-by-side frames in memory
    # To save space, we resize RGB to match depth height (400px)
    processed_frames = []

    last_depth = None
    last_mask = None

    codec = av.CodecContext.create('hevc', 'r')

    # Pre-scan to count frames for progress (optional but nice)
    with LogReaderEx(logfile, names=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                last_depth = data
            elif stream_id == 'oak.nn_mask':
                last_mask = data
            elif stream_id == 'oak.color':
                if last_depth is None or last_mask is None:
                    continue

                # Decode RGB
                try:
                    packets = codec.parse(data)
                    for packet in packets:
                        frames = codec.decode(packet)
                        if not frames:
                            continue

                        rgb_frame = frames[-1].to_ndarray(format='bgr24')

                        # Process visualization
                        h, w = last_depth.shape  # 400, 640

                        # 1. Depth + Mask Overlay
                        depth_vis = np.clip(last_depth, 0, 3000)
                        depth_vis = (depth_vis / 3000 * 255).astype(np.uint8)
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                        mask_resized = cv2.resize(last_mask, (w, h), interpolation=cv2.INTER_NEAREST)

                        # ROI visualization
                        roi_y_start, roi_y_end = int(h * 0.4), int(h * 0.7)
                        roi_x_start, roi_x_end = int(w * 0.4), int(w * 0.6)
                        cv2.rectangle(depth_vis, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (255, 255, 255), 2)

                        colored_mask = np.zeros_like(depth_vis)
                        colored_mask[mask_resized == 1] = [0, 255, 0]
                        overlay = cv2.addWeighted(depth_vis, 0.7, colored_mask, 0.3, 0)

                        # 2. Resize RGB to match depth height
                        rgb_h, rgb_w = rgb_frame.shape[:2]
                        new_rgb_w = int(rgb_w * h / rgb_h)
                        rgb_resized = cv2.resize(rgb_frame, (new_rgb_w, h))

                        # 3. Combine
                        combined = np.hstack((rgb_resized, overlay))

                        processed_frames.append({'ts': timestamp.total_seconds(), 'img': combined})
                except Exception:
                    pass

            if len(processed_frames) % 50 == 0 and len(processed_frames) > 0:
                print(f' Loaded {len(processed_frames)} frames...', end='\r')

    print(f'\nFinished loading {len(processed_frames)} frames.')
    return processed_frames


def main():
    if len(sys.argv) < 2:
        print('Usage: python interactive_view.py <logfile>')
        return

    logfile = sys.argv[1]
    frames = load_log_data(logfile)

    if not frames:
        print('No frames found to display.')
        return

    window_name = f'OSGAR Interactive View: {os.path.basename(logfile)}'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    def on_trackbar(val):
        frame = frames[val]
        img = frame['img'].copy()

        # Add timestamp text
        ts_text = f'Time: {frame["ts"]:.2f}s | Frame: {val}/{len(frames) - 1}'
        cv2.putText(img, ts_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow(window_name, img)

    cv2.createTrackbar('Frame', window_name, 0, len(frames) - 1, on_trackbar)

    # Show first frame
    on_trackbar(0)

    print("Use the slider to navigate. Press 'q' or 'Esc' to exit.")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

        # Optional: Arrow keys for fine-tuning
        if key == 83:  # Right arrow
            pos = cv2.getTrackbarPos('Frame', window_name)
            cv2.setTrackbarPos('Frame', window_name, min(pos + 1, len(frames) - 1))
        if key == 81:  # Left arrow
            pos = cv2.getTrackbarPos('Frame', window_name)
            cv2.setTrackbarPos('Frame', window_name, max(pos - 1, 0))

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
