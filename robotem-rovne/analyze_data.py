import argparse

import numpy as np
from osgar.logger import LogReaderEx


def analyze_log(logfile, target_times):
    streams = ['oak.depth']

    # Sort times to potentially allow single-pass reading in the future,
    # but for now we'll stick to the simple per-time seek for clarity.
    for target_time in sorted(target_times):
        print(f'\n--- Analyzing at {target_time}s ---')
        found = False
        with LogReaderEx(logfile, names=streams) as log:
            for timestamp, stream_id, data in log:
                if timestamp.total_seconds() < target_time:
                    continue

                if stream_id == 'oak.depth':
                    # Analyze depth in horizontal bands
                    # Resolution is 400x640
                    height = data.shape[0]
                    for start_pct in [0.4, 0.5, 0.6, 0.7, 0.8]:
                        r1, r2 = int(height * start_pct), int(height * (start_pct + 0.1))
                        band = data[r1:r2, :]
                        valid = band[band > 0]
                        if len(valid) > 0:
                            print(
                                f'Band {int(start_pct * 100)}-{int((start_pct + 0.1) * 100)}%: '
                                f'10th-perc: {np.percentile(valid, 10):.0f}mm, '
                                f'min: {np.min(valid)}mm'
                            )
                        else:
                            print(f'Band {int(start_pct * 100)}-{int((start_pct + 0.1) * 100)}%: NO VALID DATA')
                    found = True
                    break
        if not found:
            print(f'No depth data found at or after {target_time}s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Analyze OSGAR log depth profiles in horizontal bands.')
    parser.add_argument('logfile', help='Path to the OSGAR log file')
    parser.add_argument(
        '--times', type=float, nargs='+', help='List of times (seconds) to analyze', default=[7.0, 29.5]
    )

    args = parser.parse_args()
    analyze_log(args.logfile, args.times)
