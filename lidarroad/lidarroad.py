import argparse
from datetime import timedelta

import numpy as np
from osgar.logger import LogReaderEx


def slow_get_best_match(mask, window_size, prev_from_i=None, penalty_weight=0.0):
    best_i = 0
    best_sum = None
    for i in range(0, len(mask) - window_size + 1):
        value = sum(mask[i:i+window_size])
        if prev_from_i is not None and penalty_weight > 0.0:
            value -= penalty_weight * abs(i - prev_from_i)
        if best_sum is None or value > best_sum:
            best_sum = value
            best_i = i
    return best_i, best_i + window_size


def get_best_match(mask, window_size, prev_from_i=None, penalty_weight=0.0):
    if len(mask) <= window_size:
        return 0, window_size
    cum = np.cumsum(np.asarray(mask))
    window_sums = np.empty(len(mask) - window_size + 1, dtype=cum.dtype)
    window_sums[0] = cum[window_size - 1]
    window_sums[1:] = cum[window_size:] - cum[:-window_size]

    if prev_from_i is not None and penalty_weight > 0.0:
        indices = np.arange(len(window_sums))
        penalty = penalty_weight * np.abs(indices - prev_from_i)
        penalized_sums = window_sums - penalty
        best_i = int(np.argmax(penalized_sums))
    else:
        best_i = int(np.argmax(window_sums))
    return best_i, best_i + window_size


def analyze_scan(scan, tolerance=10, window_size = 300, fast=False, prev_from_i=None, penalty_weight=0.0):
    diff = np.diff(scan)
    mask = np.abs(diff) < tolerance
    from_i, to_i = get_best_match(
        mask, window_size, prev_from_i=prev_from_i, penalty_weight=penalty_weight
    )
    if not fast:
        from_i_slow, to_i_slow = slow_get_best_match(
            mask, window_size, prev_from_i=prev_from_i, penalty_weight=penalty_weight
        )
        assert (from_i, to_i) == (from_i_slow, to_i_slow), (
            f"Optimization mismatch: {(from_i, to_i)} != {(from_i_slow, to_i_slow)}"
        )
    return diff, from_i, to_i


def calculate_road_width(data, from_i, to_i, tilt_deg=10.0, is_sliced=True):
    """Calculate the road width in meters based on the matching window boundaries."""
    if is_sliced:
        i_L = 450 + from_i
        i_R = 450 + to_i
        theta_L = np.pi * (0.5 - from_i / 900.0)
        theta_R = np.pi * (0.5 - to_i / 900.0)
    else:
        i_L = from_i
        i_R = to_i
        theta_L = np.pi * (1.0 - i_L / 900.0)
        theta_R = np.pi * (1.0 - i_R / 900.0)

    r_L = data[i_L] / 1000.0  # Convert mm to meters
    r_R = data[i_R] / 1000.0

    proj_factor = np.cos(np.radians(tilt_deg))

    x_L = r_L * proj_factor * np.cos(theta_L)
    y_L = r_L * proj_factor * np.sin(theta_L)

    x_R = r_R * proj_factor * np.cos(theta_R)
    y_R = r_R * proj_factor * np.sin(theta_R)

    width = np.hypot(x_L - x_R, y_L - y_R)
    return width


def draw_scan(scan, tolerance=None, interval=None, original_scan=None, prev_from_i=None, penalty_weight=0.0):
    import matplotlib.pyplot as plt

    if original_scan is not None:
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharex=True)

        ax1.plot(original_scan, label='Scan')
        ax1.set_title('Original Scan')
        ax1.set_xlabel('Scan Index')
        ax1.set_ylabel('Range')

        ax2.plot(scan)
        ax2.set_title('np.diff')
        ax2.set_xlabel('Scan Index')
        ax2.set_ylabel('Difference')
        if tolerance is not None:
            ax2.axhline(y=tolerance, color='r', linestyle='--')
            ax2.axhline(y=-tolerance, color='r', linestyle='--')

        if tolerance is not None and interval is not None:
            mask = np.abs(scan) < tolerance
            from_i_tracked, to_i_tracked = interval
            window_size = to_i_tracked - from_i_tracked

            if len(mask) > window_size:
                cum = np.cumsum(np.asarray(mask))
                window_sums = np.empty(len(mask) - window_size + 1, dtype=cum.dtype)
                window_sums[0] = cum[window_size - 1]
                window_sums[1:] = cum[window_size:] - cum[:-window_size]

                # 1. Unpenalized Global argmax
                from_i_global = int(np.argmax(window_sums))
                to_i_global = from_i_global + window_size

                # Draw Global match (blue)
                ax1.axvline(x=from_i_global, color='b', linestyle=':', label='Global Match')
                ax1.axvline(x=to_i_global, color='b', linestyle=':')
                ax2.axvline(x=from_i_global, color='b', linestyle=':', label='Global Match')
                ax2.axvline(x=to_i_global, color='b', linestyle=':')

                # Draw Tracked match (green)
                ax1.axvline(x=from_i_tracked, color='g', linestyle='--', label='Tracked Match')
                ax1.axvline(x=to_i_tracked, color='g', linestyle='--')
                ax2.axvline(x=from_i_tracked, color='g', linestyle='--', label='Tracked Match')
                ax2.axvline(x=to_i_tracked, color='g', linestyle='--')

                # Cost functions plot
                ax3.plot(window_sums, color='gray', linestyle=':', label='Unpenalized Cost')

                penalized_sums = window_sums
                if prev_from_i is not None and penalty_weight > 0.0:
                    indices = np.arange(len(window_sums))
                    penalty = penalty_weight * np.abs(indices - prev_from_i)
                    penalized_sums = window_sums - penalty
                    ax3.plot(penalized_sums, color='b', label='Penalized Cost')

                ax3.set_title('Window Cost Function')
                ax3.set_xlabel('Window Start Index')
                ax3.set_ylabel('Points / Cost')

                # Highlight global argmax peak
                if 0 <= from_i_global < len(window_sums):
                    ax3.plot(
                        from_i_global, window_sums[from_i_global], 'bo',
                        label=f'Global Argmax ({from_i_global})'
                    )
                    ax3.axvline(x=from_i_global, color='b', linestyle=':')

                # Highlight tracked argmax peak
                if 0 <= from_i_tracked < len(penalized_sums):
                    ax3.plot(
                        from_i_tracked, penalized_sums[from_i_tracked], 'ro',
                        label=f'Tracked Argmax ({from_i_tracked})'
                    )
                    ax3.axvline(x=from_i_tracked, color='g', linestyle='--')

                # Highlight previous tracker position if applicable
                if prev_from_i is not None:
                    ax3.axvline(x=prev_from_i, color='y', linestyle='-.', label=f'Prev Pos ({prev_from_i})')

                ax3.legend()
            else:
                ax3.text(0.5, 0.5, 'Window size too large', ha='center', va='center')
                ax3.set_title('Window Cost Function')
        ax1.legend()
        ax2.legend()
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


def batch_processing(
    log, start_sec, end_sec, tolerance, window_size, fast=False, penalty_weight=0.0, initial_prev=None
):
    start_time = timedelta(seconds=start_sec)
    end_time = timedelta(seconds=end_sec)

    times = []
    from_indices = []
    to_indices = []

    prev_from_i = initial_prev

    for timestamp, name, data in log:
        if timestamp < start_time:
            continue
        if timestamp > end_time:
            break

        diff, from_i, to_i = analyze_scan(
            data, tolerance=tolerance, window_size=window_size, fast=fast,
            prev_from_i=prev_from_i, penalty_weight=penalty_weight
        )
        width = calculate_road_width(data, from_i, to_i, is_sliced=False)
        print(timestamp, len(data), from_i, to_i, f"width: {width:.2f}m")

        times.append(timestamp.total_seconds())
        from_indices.append(from_i)
        to_indices.append(to_i)
        prev_from_i = from_i

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
    parser.add_argument(
        '--weight', type=float, default=0.0,
        help='penalty weight for distance from previous window position'
    )
    parser.add_argument(
        '--prev', type=int,
        help='simulate starting window index from previous scan'
    )
    args = parser.parse_args()

    window_size = int(args.width * 100)  # simplified conversion to scan indexes - TODO proper calibration
    tolerance = args.tolerance
    fast = args.fast

    with LogReaderEx(args.logfile, ['vanjee.scan10']) as log:
        if args.batch is not None:
            start_sec, end_sec = args.batch
            times, from_indices, to_indices = batch_processing(
                log, start_sec, end_sec, tolerance, window_size, fast=fast,
                penalty_weight=args.weight, initial_prev=args.prev
            )
            if times:
                draw_batch(times, from_indices, to_indices)
            else:
                print("No scans found in the specified time range.")
        else:
            prev_from_i = args.prev
            for timestamp, name, data in log:
                if args.jump is not None and timestamp < timedelta(seconds=args.jump):
                    continue
                print(timestamp, len(data))
                selected = data[450:-450]
                diff, from_i, to_i = analyze_scan(
                    selected, tolerance=tolerance, window_size=window_size, fast=fast,
                    prev_from_i=prev_from_i, penalty_weight=args.weight
                )
                print(from_i, to_i)
                width = calculate_road_width(data, from_i, to_i, is_sliced=True)
                print(f"Calculated Road Width: {width:.2f}m")
                draw_scan(
                    diff,
                    tolerance=tolerance,
                    interval=(from_i, to_i),
                    original_scan=selected if args.show_original else None,
                    prev_from_i=prev_from_i,
                    penalty_weight=args.weight
                )
                prev_from_i = from_i
                if args.weight == 0.0:
                    break



if __name__ == '__main__':
    main()
