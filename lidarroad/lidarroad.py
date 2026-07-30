import argparse
from datetime import timedelta

import numpy as np

from osgar.logger import LogReaderEx


def get_best_match(mask, window_size):
    best_i = 0
    best_sum = None
    for i in range(0, len(mask) - window_size):
        value = sum(mask[i:i+window_size])
        if best_sum is None or value > best_sum:
            best_sum = value
            best_i = i
    return best_i, best_i + window_size


def analyze_scan(scan, tolerance=10, window_size = 300):
    assert len(scan)==1800, len(scan)
    diff = np.diff(scan) #[450:-450])
    mask = np.abs(diff) < tolerance
    from_i, to_i = get_best_match(mask, window_size)
    return diff, from_i, to_i


def draw_scan(scan, tolerance=None, interval=None):
    import matplotlib.pyplot as plt

    plt.plot(scan)
    if tolerance is not None:
        plt.axhline(y=tolerance, color='r', linestyle='--')
        plt.axhline(y=-tolerance, color='r', linestyle='--')
    if interval is not None:
        from_i, to_i = interval
        plt.axvline(x=from_i, color='g', linestyle='--')
        plt.axvline(x=to_i, color='g', linestyle='--')
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Analyze smoothness of the road/scan10')
    parser.add_argument('logfile', help='logfile path')
    parser.add_argument('--jump', '-j', help='jump in seconds', type=float)
    parser.add_argument('--width', '-w', help='width in meters', type=float, default=3.0)
    parser.add_argument('--tolerance', '-t', help='tolerance in millimeters', type=int, default=10)
    args = parser.parse_args()

    window_size = int(args.width * 100)  # simplified conversion to scan indexes - TODO proper calibration
    tolerance = args.tolerance

    with LogReaderEx(args.logfile, ['vanjee.scan10']) as log:
        for timestamp, name, data in log:
            if args.jump is not None and timestamp < timedelta(seconds=args.jump):
                continue
            print(timestamp, len(data))
            diff, from_i, to_i = analyze_scan(data, tolerance=tolerance, window_size=window_size)
            print(from_i, to_i)
            draw_scan(diff, tolerance=tolerance, interval=(from_i, to_i))
            break



if __name__ == '__main__':
    main()
