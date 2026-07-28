import argparse
from datetime import timedelta

import numpy as np

from osgar.logger import LogReaderEx


def analyze_scan(scan):
    assert len(scan)==1800, len(scan)
    diff = np.diff(scan) #[450:-450])
    draw_scan(diff)


def draw_scan(scan):
    import matplotlib.pyplot as plt

    plt.plot(scan)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Analyze smoothness of the road/scan10')
    parser.add_argument('logfile', help='logfile path')
    args = parser.parse_args()

    with LogReaderEx(args.logfile, ['vanjee.scan10']) as log:
        for timestamp, name, data in log:
            if timestamp < timedelta(seconds=100):
                continue
            print(timestamp, len(data))
            analyze_scan(data)
            break



if __name__ == '__main__':
    main()
