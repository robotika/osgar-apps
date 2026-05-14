
import datetime
import cv2
import numpy as np
import sys
import os
from osgar.logger import LogReaderEx

def visualize_at_time(logfile, target_sec, output_prefix='debug_stop'):
    streams = ['oak.depth', 'oak.nn_mask']

    last_depth = None
    last_mask = None
    
    target_time = datetime.timedelta(seconds=target_sec)

    with LogReaderEx(logfile, names=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                last_depth = data
            elif stream_id == 'oak.nn_mask':
                last_mask = data

            # If we are past target time and have data, generate one image and exit
            if timestamp >= target_time and last_depth is not None and last_mask is not None:
                # Normalize depth for visualization (0-3000mm)
                # Obstacles triggering the stop are < 1200mm
                depth_vis = np.clip(last_depth, 0, 3000)
                depth_vis = (depth_vis / 3000 * 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

                # Resize mask to depth size for overlay
                # Depth is 640x400
                h, w = last_depth.shape
                mask_resized = cv2.resize(last_mask, (w, h), interpolation=cv2.INTER_NEAREST)
                
                # ROI visualization (40-70% height, 40-60% width)
                roi_y_start, roi_y_end = int(h * 0.4), int(h * 0.7)
                roi_x_start, roi_x_end = int(w * 0.4), int(w * 0.6)
                cv2.rectangle(depth_vis, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (255, 255, 255), 2)

                colored_mask = np.zeros_like(depth_vis)
                colored_mask[mask_resized == 1] = [0, 255, 0]  # Road in Green

                overlay = cv2.addWeighted(depth_vis, 0.7, colored_mask, 0.3, 0)
                
                filename = f"{output_prefix}_{target_sec:.1f}s.png"
                cv2.imwrite(filename, overlay)
                print(f"Saved {filename} (Timestamp: {timestamp})")
                return True
    return False

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python visualize_stop.py <logfile> <seconds>")
    else:
        visualize_at_time(sys.argv[1], float(sys.argv[2]))
