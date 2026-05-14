import glob
import os
import subprocess
import sys

from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_id


def get_steering_stream_name(log_path):
    # Try common stream names
    for name in ['app.desired_steering', 'ros.desired_steering']:
        try:
            lookup_stream_id(log_path, name)
            return name
        except Exception:
            pass
    # Try any stream ending in desired_steering
    try:
        reader = LogReader(log_path)
        for stream_id in reader.streams():
            name = reader.request_uri(stream_id)
            if name.endswith('.desired_steering'):
                return name
    except Exception:
        pass
    return None


def get_steering_data(log_path, stream_name):
    try:
        stream_id = lookup_stream_id(log_path, stream_name)
    except Exception:
        return {}

    data = {}
    for dt, channel, raw_data in LogReader(log_path, only_stream_id=stream_id):
        # Store by timestamp (dt is timedelta)
        data[dt.total_seconds()] = deserialize(raw_data)
    return data


def analyze_log(log_path, config_path):
    print(f'Analyzing {log_path}...')
    temp_log = 'stats_tmp.log'
    if os.path.exists(temp_log):
        os.remove(temp_log)

    cmd = [
        'uv',
        'run',
        'python',
        '-m',
        'osgar.replay',
        log_path,
        '--module',
        'app',
        '--config',
        config_path,
        '--force',
        '--output',
        temp_log,
    ]

    # Run replay to generate new steering
    subprocess.run(cmd, capture_output=True)

    if not os.path.exists(temp_log):
        return {'error': 'Replay failed to produce output log'}

    orig_stream = get_steering_stream_name(log_path)
    if not orig_stream:
        if os.path.exists(temp_log):
            os.remove(temp_log)
        return {'error': 'Could not find any .desired_steering stream in original log'}

    orig_steering = get_steering_data(log_path, orig_stream)
    new_steering = get_steering_data(temp_log, 'desired_steering')

    if os.path.exists(temp_log):
        os.remove(temp_log)

    if not orig_steering or not new_steering:
        return {'error': f'Could not find steering data (orig:{len(orig_steering)}, new:{len(new_steering)})'}

    stats = {'stops': [], 'slowdowns': [], 'total_new_msgs': len(new_steering)}

    # Match by timestamps
    orig_keys = sorted(orig_steering.keys())
    for ts, new_val in sorted(new_steering.items()):
        # Find closest timestamp in original using binary search or simple min if keys are few
        # For simplicity and robustness with small counts:
        closest_ts = min(orig_keys, key=lambda x: abs(x - ts))
        if abs(closest_ts - ts) > 0.1:  # Too far apart
            continue

        old_val = orig_steering[closest_ts]
        new_speed = new_val[0]
        old_speed = old_val[0]

        if new_speed == 0 and old_speed > 0:
            stats['stops'].append(ts)
        elif new_speed < old_speed and new_speed > 0:
            stats['slowdowns'].append(ts)

    return stats


def main():
    if len(sys.argv) < 2:
        print('Usage: python report_stats.py <log_dir> [config_path] [limit]')
        return

    log_dir = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != '-' else 'robotem-rovne/config/matty-redroad.json'
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else None

    # Filter for redroad logs as they are likely the navigation ones
    log_files = glob.glob(os.path.join(log_dir, '*redroad*.log'))
    if not log_files:
        log_files = glob.glob(os.path.join(log_dir, '*.log'))

    if not log_files:
        print(f'No log files found in {log_dir}')
        return

    log_files.sort()
    if limit:
        log_files = log_files[:limit]

    print(f'Processing {len(log_files)} log files.')

    for log_file in log_files:
        stats = analyze_log(log_file, config_path)
        print(f'\nResults for {os.path.basename(log_file)}:')
        if 'error' in stats:
            print(f'  ERROR: {stats["error"]}')
            continue

        print(f'  New Stops:     {len(stats["stops"])}')
        if stats['stops']:
            events = []
            start = stats['stops'][0]
            prev = start
            for s in stats['stops'][1:]:
                if s - prev > 0.5:
                    events.append((start, prev))
                    start = s
                prev = s
            events.append((start, prev))
            for start, end in events:
                print(f'    - From {start:.1f}s to {end:.1f}s')

        print(f'  New Slowdowns: {len(stats["slowdowns"])}')
        if stats['slowdowns']:
            events = []
            start = stats['slowdowns'][0]
            prev = start
            for s in stats['slowdowns'][1:]:
                if s - prev > 0.5:
                    events.append((start, prev))
                    start = s
                prev = s
            events.append((start, prev))
            for start, end in events:
                print(f'    - From {start:.1f}s to {end:.1f}s')


if __name__ == '__main__':
    main()
