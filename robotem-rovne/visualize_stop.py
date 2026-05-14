
import datetime
import cv2
import numpy as np
import sys
import os
import av
from osgar.logger import LogReaderEx
from osgar.lib.serialize import deserialize

def visualize_at_time(logfile, target_sec, output_prefix='debug_stop'):
    streams = ['oak.depth', 'oak.nn_mask', 'oak.color']

    last_depth = None
    last_mask = None
    last_frame = None
    
    target_time = datetime.timedelta(seconds=target_sec)
    codec = av.CodecContext.create('hevc', 'r')

    with LogReaderEx(logfile, names=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                last_depth = data
            elif stream_id == 'oak.nn_mask':
                last_mask = data
            elif stream_id == 'oak.color':
                # Video data from LogReaderEx might be raw bytes or already handled
                # but PyAV needs the raw buffer. If it's already deserialized into bytes:
                raw_data = data
                try:
                    packets = codec.parse(raw_data)
                    for packet in packets:
                        frames = codec.decode(packet)
                        if frames:
                            last_frame = frames[-1].to_ndarray(format='bgr24')
                except Exception as e:
                    pass # Silently skip decoding errors

            # If we are past target time and have all data, generate visualization
            if timestamp >= target_time and last_depth is not None and last_mask is not None and last_frame is not None:
                h, w = last_depth.shape
                
                # 1. Process Depth + Mask Overlay
                depth_vis = np.clip(last_depth, 0, 3000)
                depth_vis = (depth_vis / 3000 * 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                mask_resized = cv2.resize(last_mask, (w, h), interpolation=cv2.INTER_NEAREST)
                
                # ROI visualization (40-70% height, 40-60% width)
                roi_y_start, roi_y_end = int(h * 0.4), int(h * 0.7)
                roi_x_start, roi_x_end = int(w * 0.4), int(w * 0.6)
                cv2.rectangle(depth_vis, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (255, 255, 255), 2)

                colored_mask = np.zeros_like(depth_vis)
                colored_mask[mask_resized == 1] = [0, 255, 0]  # Road in Green
                overlay = cv2.addWeighted(depth_vis, 0.7, colored_mask, 0.3, 0)

                # 2. Process RGB
                rgb_h, rgb_w = last_frame.shape[:2]
                new_rgb_w = int(rgb_w * h / rgb_h)
                rgb_resized = cv2.resize(last_frame, (new_rgb_w, h))

                # 3. Side-by-Side
                combined = np.hstack((rgb_resized, overlay))
                
                filename = f"{output_prefix}_{target_sec:.1f}s.png"
                cv2.imwrite(filename, combined)
                print(f"Saved {filename} (Timestamp: {timestamp})")
                return True
    return False

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python visualize_stop.py <logfile> <seconds>")
    else:
        visualize_at_time(sys.argv[1], float(sys.argv[2]))
