import cv2
import numpy as np
from osgar.logger import LogReaderEx
import datetime

def visualize_collision(logfile, output_prefix="collision"):
    streams = ['oak.depth', 'oak.nn_mask', 'oak.color']
    
    last_depth = None
    last_mask = None
    
    # Target time is near the end, around 29s
    target_time = datetime.timedelta(seconds=29.0)
    
    with LogReaderEx(logfile, names=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                last_depth = data
            elif stream_id == 'oak.nn_mask':
                last_mask = data
            elif stream_id == 'oak.color':
                # Skip color decoding for speed if not at target
                if timestamp < target_time:
                    continue
                
                # For the sake of this script, let's just use the last known depth and mask
                if last_depth is not None and last_mask is not None:
                    # Normalize depth for visualization (0-255)
                    # Trees should be close, say < 5 meters (5000mm)
                    depth_vis = np.clip(last_depth, 0, 5000)
                    depth_vis = (depth_vis / 5000 * 255).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    
                    # Resize mask to depth size for overlay
                    mask_resized = cv2.resize(last_mask, (640, 400), interpolation=cv2.INTER_NEAREST)
                    colored_mask = np.zeros_like(depth_vis)
                    colored_mask[mask_resized == 1] = [0, 255, 0] # Road in Green
                    
                    # Overlay
                    overlay = cv2.addWeighted(depth_vis, 0.7, colored_mask, 0.3, 0)
                    
                    filename = f"{output_prefix}_{timestamp.total_seconds():.2f}.png"
                    cv2.imwrite(filename, overlay)
                    print(f"Saved {filename}")
            
            if timestamp > target_time + datetime.timedelta(seconds=1):
                break

if __name__ == "__main__":
    visualize_collision("robotem-rovne/data/m04-matty-redroad-260501_105531.log")
