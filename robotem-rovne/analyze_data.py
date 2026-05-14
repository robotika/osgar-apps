import cv2
import numpy as np
from osgar.logger import LogReaderEx

def analyze_log(logfile):
    streams = ['oak.depth', 'oak.nn_mask']
    
    for target_time in [7.0, 29.5]:
        print(f"\n--- Analyzing at {target_time}s ---")
        with LogReaderEx(logfile, names=streams) as log:
            for timestamp, stream_id, data in log:
                if timestamp.total_seconds() < target_time:
                    continue
                if stream_id == 'oak.depth':
                    # Analyze depth in horizontal bands
                    # Image is 400 rows high.
                    # Band 1: 40-50% (rows 160-200)
                    # Band 2: 50-60% (rows 200-240)
                    # Band 3: 60-70% (rows 240-280)
                    for start_pct in [0.4, 0.5, 0.6, 0.7, 0.8]:
                        r1, r2 = int(400 * start_pct), int(400 * (start_pct + 0.1))
                        band = data[r1:r2, :]
                        valid = band[band > 0]
                        if len(valid) > 0:
                            print(f"Band {int(start_pct*100)}-{int((start_pct+0.1)*100)}%: 10th-perc: {np.percentile(valid, 10):.0f}mm, min: {np.min(valid)}mm")
                    break

if __name__ == "__main__":
    analyze_log("robotem-rovne/data/m04-matty-redroad-260501_105531.log")
