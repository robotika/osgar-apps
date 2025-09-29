"""
  Analyze noisy depth data
"""
import argparse

import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def extract_depth(log_path, stream_name):
    line = 400 // 2
    line_end = 400 // 2 + 30
    box_width = 160

    arr = []
    stream_id = lookup_stream_id(log_path, stream_name)
    with LogReader(log_path, only_stream_id=stream_id) as log:
        index = 320
        for i, (timestamp, stream, raw) in enumerate(log):
            data = deserialize(raw)
            mask = data[line:line_end, index:box_width + index] != 0
            if mask.max():
                dist5 = int(np.percentile( data[line:line_end, index:box_width + index][mask], 5))
                dist10 = int(np.percentile( data[line:line_end, index:box_width + index][mask], 10))
                dist50 = int(np.percentile( data[line:line_end, index:box_width + index][mask], 50))
            else:
                dist5, dist10, dist50 = 0, 0, 0
#            print(timestamp, dist5, dist10, dist50)
            arr.append((timestamp.total_seconds(), dist5, dist10, dist50))
            if i > 100:
                break
        return arr


def draw(arr):
    import matplotlib.pyplot as plt
    for i in range(len(arr[0]) - 1):
        x = [v[0] for v in arr]
        y = [v[i+1]/1000.0 for v in arr]
        plt.plot(x, y)
    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", help="Path to the OSGAR log file.")
    parser.add_argument("--stream-name", default="oak.depth", help="Name of the depth data stream")
    args = parser.parse_args()

    tmp = extract_depth(
        log_path=args.log_path,
        stream_name=args.stream_name,
    )
    draw(tmp)
