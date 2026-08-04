import argparse
from datetime import timedelta

import numpy as np
from osgar.logger import LogReaderEx


def slow_get_best_match(mask, window_size):
    best_i = 0
    best_sum = None
    for i in range(0, len(mask) - window_size + 1):
        value = sum(mask[i:i+window_size])
        if best_sum is None or value > best_sum:
            best_sum = value
            best_i = i
    return best_i, best_i + window_size


def get_best_match(mask, window_size):
    if len(mask) <= window_size:
        return 0, window_size
    cum = np.cumsum(np.asarray(mask))
    window_sums = np.empty(len(mask) - window_size + 1, dtype=cum.dtype)
    window_sums[0] = cum[window_size - 1]
    window_sums[1:] = cum[window_size:] - cum[:-window_size]
    best_i = int(np.argmax(window_sums))
    return best_i, best_i + window_size


def analyze_scan(scan, tolerance=10, window_size = 300, fast=False):
    diff = np.diff(scan)
    mask = np.abs(diff) < tolerance
    from_i, to_i = get_best_match(mask, window_size)
    if not fast:
        from_i_slow, to_i_slow = slow_get_best_match(mask, window_size)
        assert (from_i, to_i) == (from_i_slow, to_i_slow), (
            f"Optimization mismatch: {(from_i, to_i)} != {(from_i_slow, to_i_slow)}"
        )
    return diff, from_i, to_i


def draw_scan(scan, tolerance=None, interval=None, original_scan=None):
    import matplotlib.pyplot as plt

    if original_scan is not None:
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharex=True)

        ax1.plot(original_scan)
        ax1.set_title('Original Scan')
        ax1.set_xlabel('Scan Index')
        ax1.set_ylabel('Range')
        if interval is not None:
            from_i, to_i = interval
            ax1.axvline(x=from_i, color='g', linestyle='--')
            ax1.axvline(x=to_i, color='g', linestyle='--')

        ax2.plot(scan)
        ax2.set_title('np.diff')
        ax2.set_xlabel('Scan Index')
        ax2.set_ylabel('Difference')
        if tolerance is not None:
            ax2.axhline(y=tolerance, color='r', linestyle='--')
            ax2.axhline(y=-tolerance, color='r', linestyle='--')
        if interval is not None:
            from_i, to_i = interval
            ax2.axvline(x=from_i, color='g', linestyle='--')
            ax2.axvline(x=to_i, color='g', linestyle='--')

        if tolerance is not None and interval is not None:
            mask = np.abs(scan) < tolerance
            from_i, to_i = interval
            window_size = to_i - from_i

            if len(mask) > window_size:
                cum = np.cumsum(np.asarray(mask))
                window_sums = np.empty(len(mask) - window_size + 1, dtype=cum.dtype)
                window_sums[0] = cum[window_size - 1]
                window_sums[1:] = cum[window_size:] - cum[:-window_size]

                ax3.plot(window_sums, color='b', label='Cost Function')
                ax3.set_title('Window Cost Function')
                ax3.set_xlabel('Window Start Index')
                ax3.set_ylabel('Points in Window')
                if 0 <= from_i < len(window_sums):
                    ax3.plot(from_i, window_sums[from_i], 'ro', label=f'Argmax ({from_i})')
                    ax3.axvline(x=from_i, color='g', linestyle='--')
                    ax3.legend()
            else:
                ax3.text(0.5, 0.5, 'Window size too large', ha='center', va='center')
                ax3.set_title('Window Cost Function')
    else:
        plt.plot(scan)
        plt.title('np.diff')
        plt.xlabel('Scan Index')
        plt.ylabel('Difference')
        if tolerance is not None:
            plt.axhline(y=tolerance, color='r', linestyle='--')
            plt.axhline(y=-tolerance, color='r', linestyle='--')
        if interval is not None:
            from_i, to_i = interval
            plt.axvline(x=from_i, color='g', linestyle='--')
            plt.axvline(x=to_i, color='g', linestyle='--')
    plt.show()


def draw_batch(timestamps, from_indices, to_indices):
    import matplotlib.pyplot as plt

    plt.plot(timestamps, from_indices, 'g.-', label='from_i')
    plt.plot(timestamps, to_indices, 'b.-', label='to_i')
    plt.xlabel('Time (s)')
    plt.ylabel('Scan Index')
    plt.legend()
    plt.title('Best Matching Interval Boundaries Over Time')
    plt.show()


def batch_processing(log, start_sec, end_sec, tolerance, window_size, fast=False):
    start_time = timedelta(seconds=start_sec)
    end_time = timedelta(seconds=end_sec)

    times = []
    from_indices = []
    to_indices = []

    for timestamp, name, data in log:
        if timestamp < start_time:
            continue
        if timestamp > end_time:
            break

        diff, from_i, to_i = analyze_scan(data, tolerance=tolerance, window_size=window_size, fast=fast)
        print(timestamp, len(data), from_i, to_i)

        times.append(timestamp.total_seconds())
        from_indices.append(from_i)
        to_indices.append(to_i)

    return times, from_indices, to_indices


def main():
    parser = argparse.ArgumentParser(description='Analyze smoothness of the road/scan10')
    parser.add_argument('logfile', help='logfile path')
    parser.add_argument('--jump', '-j', help='jump in seconds', type=float)
    parser.add_argument('--width', '-w', help='width in meters', type=float, default=3.0)
    parser.add_argument('--tolerance', '-t', help='tolerance in millimeters', type=int, default=10)
    parser.add_argument(
        '--batch', nargs=2, type=float, metavar=('START', 'END'),
        help='run in batch mode for time range in seconds'
    )
    parser.add_argument('--fast', action='store_true', help='skip slow match calculation and assertion')
    parser.add_argument('--show-original', action='store_true', help='show original range data next to np.diff')
    args = parser.parse_args()

    window_size = int(args.width * 100)  # simplified conversion to scan indexes - TODO proper calibration
    tolerance = args.tolerance
    fast = args.fast

    with LogReaderEx(args.logfile, ['vanjee.scan10']) as log:
        if args.batch is not None:
            start_sec, end_sec = args.batch
            times, from_indices, to_indices = batch_processing(
                log, start_sec, end_sec, tolerance, window_size, fast=fast
            )
            if times:
                draw_batch(times, from_indices, to_indices)
            else:
                print("No scans found in the specified time range.")
        else:
            for timestamp, name, data in log:
                if args.jump is not None and timestamp < timedelta(seconds=args.jump):
                    continue
                print(timestamp, len(data))
                selected = data[450:-450]
                diff, from_i, to_i = analyze_scan(selected, tolerance=tolerance, window_size=window_size, fast=fast)
                print(from_i, to_i)
                draw_scan(
                    diff,
                    tolerance=tolerance,
                    interval=(from_i, to_i),
                    original_scan=selected if args.show_original else None
                )
                break



if __name__ == '__main__':
    main()
