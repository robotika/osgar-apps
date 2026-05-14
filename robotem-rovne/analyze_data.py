import cv2
import numpy as np
from osgar.logger import LogReaderEx

def analyze_log(logfile):
    streams = ['oak.depth', 'oak.nn_mask']
    
    with LogReaderEx(logfile, only_stream_id=streams) as log:
        for timestamp, stream_id, data in log:
            if stream_id == 'oak.depth':
                print(f"[{timestamp}] Depth shape: {data.shape if hasattr(data, 'shape') else len(data)}, dtype: {data.dtype if hasattr(data, 'dtype') else 'N/A'}")
                print(f"Depth min: {np.min(data)}, max: {np.max(data)}, mean: {np.mean(data)}")
            elif stream_id == 'oak.nn_mask':
                print(f"[{timestamp}] NN Mask shape: {data.shape}, dtype: {data.dtype}")
                print(f"NN Mask unique values: {np.unique(data)}")
            
            # Just check first few frames
            if timestamp.total_seconds() > 7:
                break

if __name__ == "__main__":
    analyze_log("robotem-rovne/data/m04-matty-redroad-260501_105531.log")
