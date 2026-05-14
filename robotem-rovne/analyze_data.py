import cv2
import numpy as np
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

def analyze_log(logfile):
    depth_stream = lookup_stream_id(logfile, 'oak.depth')
    nn_mask_stream = lookup_stream_id(logfile, 'oak.nn_mask')
    
    with LogReader(logfile, only_stream_id=[depth_stream, nn_mask_stream]) as log:
        for timestamp, stream_id, data in log:
            obj = deserialize(data)
            if stream_id == depth_stream:
                print(f"[{timestamp}] Depth shape: {obj.shape if hasattr(obj, 'shape') else len(obj)}, dtype: {obj.dtype if hasattr(obj, 'dtype') else 'N/A'}")
                print(f"Depth min: {np.min(obj)}, max: {np.max(obj)}, mean: {np.mean(obj)}")
                # Save one frame for inspection if needed
                # np.save("debug_depth.npy", obj)
            elif stream_id == nn_mask_stream:
                print(f"[{timestamp}] NN Mask shape: {obj.shape}, dtype: {obj.dtype}")
                print(f"NN Mask unique values: {np.unique(obj)}")
            
            # Just check first few frames
            if timestamp.total_seconds() > 7:
                break

if __name__ == "__main__":
    analyze_log("robotem-rovne/data/m04-matty-redroad-260501_105531.log")
