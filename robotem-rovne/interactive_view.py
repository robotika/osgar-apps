import argparse
import os

import av
import cv2
import numpy as np
from osgar.logger import LogReaderEx

# Global state for interactive features
state = {'show_mask': True, 'current_val': 0, 'frames': []}


def load_log_data(logfile):
    print(f'Loading and decoding {logfile}...')
    streams = ['oak.depth', 'oak.nn_mask', 'oak.color']

    frames_data = []
    last_depth = None
    last_mask = None

    codec = av.CodecContext.create('hevc', 'r')

    # Target height for visualization
    TARGET_H = 600

    with LogReaderEx(logfile, names=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                last_depth = data
            elif stream_id == 'oak.nn_mask':
                last_mask = data
            elif stream_id == 'oak.color':
                if last_depth is None or last_mask is None:
                    continue

                try:
                    packets = codec.parse(data)
                    for packet in packets:
                        frames = codec.decode(packet)
                        if not frames:
                            continue

                        rgb_frame = frames[-1].to_ndarray(format='bgr24')

                        # Process visualization components
                        # 1. Depth Base
                        orig_h, orig_w = last_depth.shape
                        new_depth_w = int(orig_w * TARGET_H / orig_h)

                        depth_base = np.clip(last_depth, 0, 3000)
                        depth_base = (depth_base / 3000 * 255).astype(np.uint8)
                        depth_base = cv2.applyColorMap(depth_base, cv2.COLORMAP_JET)
                        depth_base = cv2.resize(depth_base, (new_depth_w, TARGET_H))

                        # 2. Mask
                        mask_resized = cv2.resize(last_mask, (new_depth_w, TARGET_H), interpolation=cv2.INTER_NEAREST)

                        # 3. RGB
                        rgb_h, rgb_w = rgb_frame.shape[:2]
                        new_rgb_w = int(rgb_w * TARGET_H / rgb_h)
                        rgb_resized = cv2.resize(rgb_frame, (new_rgb_w, TARGET_H))

                        frames_data.append(
                            {
                                'ts': timestamp.total_seconds(),
                                'rgb': rgb_resized,
                                'depth_base': depth_base,
                                'mask': mask_resized,
                            }
                        )
                except Exception:
                    pass

            if len(frames_data) % 50 == 0 and len(frames_data) > 0:
                print(f' Loaded {len(frames_data)} frames...', end='\r')

    print(f'\nFinished loading {len(frames_data)} frames.')
    return frames_data


def render_frame(window_name):
    val = state['current_val']
    if val >= len(state['frames']):
        return

    data = state['frames'][val]
    depth_vis = data['depth_base'].copy()

    if state['show_mask']:
        # ROI visualization
        h, w = depth_vis.shape[:2]
        roi_y_start, roi_y_end = int(h * 0.4), int(h * 0.7)
        roi_x_start, roi_x_end = int(w * 0.4), int(w * 0.6)
        cv2.rectangle(depth_vis, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (255, 255, 255), 2)

        colored_mask = np.zeros_like(depth_vis)
        colored_mask[data['mask'] == 1] = [0, 255, 0]
        depth_vis = cv2.addWeighted(depth_vis, 0.7, colored_mask, 0.3, 0)

    # Combine side-by-side
    combined = np.hstack((data['rgb'], depth_vis))

    # Add status text
    mask_status = 'ON' if state['show_mask'] else 'OFF'
    ts_text = f'Time: {data["ts"]:.2f}s | Frame: {val}/{len(state["frames"]) - 1} | Mask (m): {mask_status}'
    cv2.putText(combined, ts_text, (22, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4)
    cv2.putText(combined, ts_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    cv2.imshow(window_name, combined)


def main():
    parser = argparse.ArgumentParser(description='OSGAR Interactive Log Viewer')
    parser.add_argument('logfile', help='Path to the OSGAR log file')
    parser.add_argument('--windowed', action='store_true', help='Start in windowed mode instead of fullscreen')
    args = parser.parse_args()

    state['frames'] = load_log_data(args.logfile)

    if not state['frames']:
        print('No frames found to display.')
        return

    window_name = f'OSGAR Interactive View: {os.path.basename(args.logfile)}'

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # Ensure aspect ratio is kept when scaling
    cv2.setWindowProperty(window_name, cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_KEEPRATIO)

    # Start in Fullscreen by default
    if not args.windowed:
        try:
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        except Exception:
            pass

    def on_trackbar(val):
        state['current_val'] = val
        render_frame(window_name)

    cv2.createTrackbar('Frame', window_name, 0, len(state['frames']) - 1, on_trackbar)

    # Show first frame
    render_frame(window_name)

    print('\nControls:')
    print('  m                : Toggle Mask')
    print('  a / d            : +/- 1 frame')
    print('  w / s            : +/- 10 frames')
    print('  Home / End       : First / Last frame')
    print('  q or Esc         : Exit')

    while True:
        key = cv2.waitKeyEx(0)

        if key == ord('q') or key == 27:
            break

        if key == ord('m'):
            state['show_mask'] = not state['show_mask']
            render_frame(window_name)
            continue

        pos = cv2.getTrackbarPos('Frame', window_name)
        new_pos = pos

        # Keys: a/d or arrows
        if key in [ord('d'), 83, 2555904, 65363]:  # Right
            new_pos = min(pos + 1, len(state['frames']) - 1)
        elif key in [ord('a'), 81, 2424832, 65361]:  # Left
            new_pos = max(pos - 1, 0)
        elif key in [ord('s'), 84, 2621440, 65364]:  # Down
            new_pos = min(pos + 10, len(state['frames']) - 1)
        elif key in [ord('w'), 82, 2490368, 65362]:  # Up
            new_pos = max(pos - 10, 0)
        elif key in [ord('f'), 65367]:  # End
            new_pos = len(state['frames']) - 1
        elif key in [ord('h'), 65360]:  # Home
            new_pos = 0

        if new_pos != pos:
            cv2.setTrackbarPos('Frame', window_name, new_pos)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
