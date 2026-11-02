"""Inspect camera IMU, depth and platform data before developing Matty pose3d."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from osgar.lib import quaternion
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_names

IMU_WIDTHS = {'oak.orientation_list': 6, 'oak.acc_raw_list': 4, 'oak.gyro_raw_list': 4}
OTHER = ('oak.depth_seq', 'platform.rpy', 'platform.joint_angle', 'platform.pose2d')


def sample_statistics(samples, malformed=0):
    """Samples are (host receipt time, individual device time, values...)."""
    if not samples:
        return dict(samples=0, malformed=malformed)
    values = np.asarray(samples, dtype=float)
    times = values[:, 1]
    gaps = np.diff(times)
    positive = gaps[gaps > 0]
    span = times[-1] - times[0]
    # Only variation, not latency: host and device clocks have different epochs.
    relative_offset = values[:, 0] - times
    return dict(samples=len(samples), malformed=malformed,
                first_device_seconds=float(times[0]), last_device_seconds=float(times[-1]),
                span_seconds=float(span), mean_hz=(len(times) - 1) / span if span > 0 else None,
                median_period_seconds=float(np.median(positive)) if len(positive) else None,
                max_gap_seconds=float(positive.max()) if len(positive) else None,
                nonincreasing_timestamps=int((gaps <= 0).sum()),
                gaps_over_100ms=int((gaps > 0.1 + 1e-9).sum()),
                receipt_minus_device_span_seconds=float(np.ptp(relative_offset)))


def read_log(path):
    names = lookup_stream_names(str(path))
    wanted = (*IMU_WIDTHS, *OTHER)
    streams = {names.index(n) + 1: n for n in wanted if n in names}
    data = {n: [] for n in wanted}
    malformed = {n: 0 for n in IMU_WIDTHS}
    batches = {n: 0 for n in IMU_WIDTHS}
    payloads = {'oak.color': 0, 'oak.depth': 0}
    for name in payloads:
        if name in names:
            streams[names.index(name) + 1] = name
    with LogReader(str(path), only_stream_id=list(streams)) as log:
        start_time = log.start_time.isoformat()
        for stamp, stream, raw in log:
            name = streams[stream]
            host = stamp.total_seconds()
            if name in payloads:
                payloads[name] += 1  # Do not decompress depth or decode color just to count frames.
                continue
            value = deserialize(raw)
            if name in IMU_WIDTHS:
                batches[name] += 1
                if not isinstance(value, (list, tuple)):
                    malformed[name] += 1
                    continue
                for sample in value:
                    try:
                        valid = len(sample) == IMU_WIDTHS[name] and np.isfinite(sample).all()
                    except (TypeError, ValueError):
                        valid = False
                    if not valid:
                        malformed[name] += 1
                        continue
                    data[name].append([host, *sample])
            else:
                data[name].append([host, *value])
    stats = {n: sample_statistics(data[n], malformed[n]) for n in IMU_WIDTHS}
    issues = []
    for n, summary in stats.items():
        summary['batches'] = batches[n]
        if not summary['samples']:
            issues.append(f'{n}: no samples')
        elif summary['samples'] < 2:
            issues.append(f'{n}: too few samples to check timing')
        if summary['malformed']:
            issues.append(f'{n}: malformed/nonfinite samples')
        if summary.get('nonincreasing_timestamps', 0):
            issues.append(f'{n}: nonincreasing device timestamps')
        if summary.get('gaps_over_100ms', 0):
            issues.append(f'{n}: device sample gaps over 100 ms')
    orientations = np.asarray(data['oak.orientation_list'])
    if len(orientations):
        norms = np.linalg.norm(orientations[:, 3:7], axis=1)
        deviation = np.abs(norms - 1)
        stats['oak.orientation_list']['max_quaternion_norm_error'] = float(deviation.max())
        if (deviation > 0.05).any():
            issues.append('oak.orientation_list: quaternion norm differs from 1 by more than 0.05')
    if not payloads['oak.depth']:
        issues.append('oak.depth: no frames')
    if not payloads['oak.color']:
        issues.append('oak.color: no frames')
    if not data['oak.depth_seq']:
        issues.append('oak.depth_seq: no capture metadata')
    if not data['platform.pose2d']:
        issues.append('platform.pose2d: no samples')
    if not data['platform.joint_angle']:
        issues.append('platform.joint_angle: no samples')
    report = dict(logfile=str(path), start_time=start_time, imu=stats,
                  messages={n: len(data[n]) for n in OTHER}, frames=payloads,
                  capture_ready=not issues, issues=issues,
                  limitations=['Checks recorded sensor data, not calibrated pose accuracy or hardware safety',
                               'OAK axes are not transformed into the front-body frame',
                               'Device timestamps and host receipt time have separate clock domains',
                               'Receipt-minus-device variation is not an absolute latency measurement'])
    return report, data


def orientation_degrees(samples):
    """Diagnostic Euler angles in the IMU's own reported frame; q and -q agree."""
    result = []
    for sample in samples:
        q = np.asarray(sample[3:7], dtype=float)
        norm = np.linalg.norm(q)
        if norm == 0:
            result.append([math.nan] * 3)
            continue
        yaw, pitch, roll = quaternion.euler_zyx((q / norm).tolist())
        result.append([roll, pitch, yaw])
    angles = np.asarray(result).reshape(-1, 3)
    for axis in range(3):
        valid = np.isfinite(angles[:, axis])
        angles[valid, axis] = np.unwrap(angles[valid, axis])
    return np.degrees(angles)


def render(path, report, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(13, 10), constrained_layout=True)
    imu_start = min((s[1] for n in IMU_WIDTHS for s in data[n]), default=0)
    for ax, name, title in [(axes[0, 0], 'oak.orientation_list', 'OAK orientation: raw sensor-frame Euler angles'),
                             (axes[1, 0], 'oak.acc_raw_list', 'OAK accelerometer: native driver values'),
                             (axes[2, 0], 'oak.gyro_raw_list', 'OAK gyroscope: native driver values')]:
        samples = np.asarray(data[name])
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('OAK device seconds relative to first IMU sample')
        if len(samples):
            values = orientation_degrees(samples) if name.endswith('orientation_list') else samples[:, 2:5]
            labels = ['roll', 'pitch', 'yaw'] if name.endswith('orientation_list') else ['x', 'y', 'z']
            for i, label in enumerate(labels):
                ax.plot(samples[:, 1] - imu_start, values[:, i], label=label, linewidth=.8)
            ax.legend(loc='best')
        else:
            ax.text(.5, .5, 'No samples', transform=ax.transAxes, ha='center')
        ax.grid(alpha=.25)
    ax = axes[0, 1]
    ax.set_title('ESP32 angles: own frame, not aligned to OAK', fontsize=10)
    samples = np.asarray(data['platform.rpy'])
    if len(samples):
        values = np.degrees(np.unwrap(np.radians(samples[:, 1:4] / 100), axis=0))
        for i, label in enumerate(['roll', 'pitch', 'yaw']):
            ax.plot(samples[:, 0], values[:, i], label=label, linewidth=.8)
        ax.legend()
    ax.set(xlabel='Host log seconds', ylabel='Degrees')
    ax = axes[1, 1]
    ax.set_title('Measured articulation', fontsize=10)
    samples = np.asarray(data['platform.joint_angle'])
    if len(samples):
        ax.plot(samples[:, 0], samples[:, 1] / 100)
    ax.set(xlabel='Host log seconds', ylabel='Degrees')
    ax = axes[2, 1]
    ax.set_title('Recorded planar odometry (origin not calibrated)', fontsize=10)
    samples = np.asarray(data['platform.pose2d'])
    if len(samples):
        ax.plot(samples[:, 1] / 1000, samples[:, 2] / 1000)
        ax.plot(samples[0, 1] / 1000, samples[0, 2] / 1000, 'go', label='start')
        ax.legend()
    ax.set(xlabel='Local X (m)', ylabel='Local Y (m)', aspect='equal')
    for ax in axes[:, 1]:
        ax.grid(alpha=.25)
    fig.suptitle(Path(report['logfile']).name + '\nSensor capture check; not a calibrated pose estimate')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='Directory for report.json and sensors.png')
    parser.add_argument('--require-oak-imu', action='store_true', help='Exit nonzero if capture checks fail')
    args = parser.parse_args()
    report, data = read_log(args.logfile)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    render(args.output / 'sensors.png', report, data)
    for name, stats in report['imu'].items():
        print(f'{name}: {stats["samples"]} samples in {stats["batches"]} batches; '
              f'rate={stats.get("mean_hz")} Hz, max gap={stats.get("max_gap_seconds")} s')
    for issue in report['issues']:
        print(issue)
    print(f'Report: {args.output}')
    if args.require_oak_imu and not report['capture_ready']:
        parser.exit(1, 'Capture checks failed; see report.json\n')


if __name__ == '__main__':
    main()
