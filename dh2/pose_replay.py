"""Build a passive Matty pose replay log from historical DH2 data."""

import argparse
import hashlib
import json
from datetime import timedelta
from pathlib import Path

import numpy as np
from osgar.lib import quaternion as quat
from osgar.lib.serialize import deserialize, serialize
from osgar.logger import LogReader, LogWriter, lookup_stream_names

from dh2.encoder_audit import read_recording
from matty.pose3d import MattyPose3D

RAW_TO_FUSED = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
OPTICAL_TO_BODY = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def settings_from_calibration(path):
    raw = Path(path).read_bytes()
    calibration = json.loads(raw)
    queries = calibration['queried']
    imu_to_left = np.asarray(queries['imu_to_left_metres'])[:3, :3]
    rectification = np.asarray(queries['left_rectification_rotation'])
    front_from_fused = OPTICAL_TO_BODY @ rectification @ imu_to_left @ RAW_TO_FUSED.T
    np.testing.assert_allclose(front_from_fused.T @ front_from_fused, np.eye(3), atol=1e-5)
    if np.linalg.det(front_from_fused) < 0:
        raise ValueError('Calibration rotation has negative determinant')
    return dict(front_from_fused=quat.from_rotation_matrix(front_from_fused.tolist()),
                rear_from_esp=quat.identity(), wheel_order=[0, 1, 2, 3],
                front_axle=.16, rear_axle=.16, track=.315, housing_offset=[.21, 0, .16],
                geometry_verified=False,
                geometry_id=f'{calibration["device_id"]}:factory-{hashlib.sha256(raw).hexdigest()[:12]}:'
                            'nominal-front-mount-firmware-wheel-order-planar-v1')


class ReplayWriter:
    def __init__(self, writer):
        self.writer = writer
        self.outputs = {}
        self.time = timedelta(0)
        self.records = []

    def register(self, *names):
        for name in names:
            self.outputs[name] = self.writer.register('pose.' + name, dt=timedelta(0))

    def publish(self, channel, data):
        self.writer.write(self.outputs[channel], serialize(data), dt=self.time)
        self.records.append((self.time.total_seconds(), channel, data))
        return self.time


def build_pose_log(path, calibration, output, config_override=None):
    settings = settings_from_calibration(calibration)
    settings.update(config_override or {})
    _, decoder, outgoing, _ = read_recording(path)
    if outgoing.resets:
        raise ValueError('Explicit ESP32 reset in recording; split into separately initialized segments')
    if any(status.rpy is None for status in decoder.statuses):
        raise ValueError('This mode requires attitude-bearing ESP32 status packets')
    events = []
    for status in decoder.statuses:
        roll, pitch, yaw = status.rpy
        events.append((status.time, 0, 'rotation', [9000 - yaw, -pitch, roll]))
        events.append((status.time, 1, 'odometry', [status.flags, status.speed, status.joint, status.encoders]))
    names = lookup_stream_names(str(path))
    if 'oak.orientation_list' not in names:
        raise ValueError('No OAK orientation stream; use an explicitly designed historical mode')
    sid = names.index('oak.orientation_list') + 1
    with LogReader(str(path), only_stream_id=sid) as log:
        for stamp, _, raw in log:
            values = deserialize(raw)
            if values:
                events.append((stamp.total_seconds(), 2, 'orientation_list', values))
    if not any(channel == 'orientation_list' for _, _, channel, _ in events):
        raise ValueError('No OAK orientation samples in recording')
    events.sort(key=lambda e: (e[0], e[1]))
    config = dict(version=2, robot=dict(
        modules={'pose': dict(driver='matty.pose3d:MattyPose3D', init=settings)},
        links=[[f'input.{channel}', f'pose.{channel}'] for channel in ['rotation', 'odometry', 'orientation_list']]))
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / 'pose.log'
    with LogWriter(filename=str(log_path), note='Matty shadow pose offline reproduction', dt=timedelta(0)) as writer:
        writer.write(0, repr(config).encode('ascii'), dt=timedelta(0))
        inputs = {n: writer.register('input.' + n, dt=timedelta(0))
                  for n in ['rotation', 'odometry', 'orientation_list']}
        bus = ReplayWriter(writer)
        node = MattyPose3D(settings, bus)
        for t, _, channel, data in events:
            timestamp = timedelta(seconds=t)
            writer.write(inputs[channel], serialize(data), dt=timestamp)
            bus.time = node.time = timestamp
            getattr(node, 'on_' + channel)(data)
    qualities = [v for _, n, v in bus.records if n == 'pose_quality']
    reasons = {}
    for quality in qualities:
        for reason in quality['reasons']:
            reasons[reason] = reasons.get(reason, 0) + 1
    rms = [v['model_rms_m'] for v in qualities if v['model_rms_m'] is not None]
    report = dict(source=str(path), calibration=str(calibration), settings=settings,
                  pose_messages=len(qualities), integrated_intervals=sum(v['integrated'] for v in qualities),
                  skipped_inputs=node.skipped, pending_tail_inputs=len(node.pending), reasons=reasons,
                  joint_final=node.model.position.tolist(),
                  translation_continuous=node.model.continuous,
                  model_rms_median_m=float(np.median(rms)) if rms else None,
                  model_rms_p90_m=float(np.percentile(rms, 90)) if rms else None,
                  limitations=['Shadow only; no motion command outputs',
                               'Joint Z held at zero, not an observed terrain elevation',
                               'Wheel order, rear geometry and front/rear mounts are provisional',
                               'IMU device-to-host offset estimated once from first batch receipt',
                               'Raw-to-fused axis rotation verified only for reviewed M04 setup',
                               'Housing pose is not a calibrated rectified optical pose',
                               'Lost or rejected intervals create unknown translation, not corrected slip'])
    (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    render(output / 'pose.png', bus.records)
    return report


def render(path, records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    front = [(t, v) for t, n, v in records if n == 'pose3d']
    rear = [v for _, n, v in records if n == 'rear_pose3d']
    housing = [v for _, n, v in records if n == 'housing_pose3d']
    quality = [v for _, n, v in records if n == 'pose_quality']
    if not front:
        raise ValueError('No synchronized pose outputs; inspect clocks and input coverage')
    times = [q['data_time_s'] for q in quality]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    joint = np.array([v[0] for _, v in front])
    house = np.array([v[0] for v in housing])
    axes[0, 0].plot(*joint[:, :2].T, label='Estimated joint')
    axes[0, 0].plot(*house[:, :2].T, label='Nominal housing reference')
    axes[0, 0].set(xlabel='X (m)', ylabel='Y (m)', aspect='equal', title='Planar model, not scene-validated')
    axes[0, 0].legend()
    for values, label in [(front, 'Front'), (list(zip(times, rear)), 'Rear')]:
        angles = np.degrees(np.unwrap([quat.euler_zyx(v[1])[0] for _, v in values]))
        axes[0, 1].plot(times, angles, label=label)
    axes[0, 1].set(ylabel='Yaw (degrees)', title='Independent, initially aligned section attitudes')
    axes[0, 1].legend()
    axes[1, 0].plot(times, house[:, 2])
    axes[1, 0].set(ylabel='Housing reference Z (m)', title='Lever arm rotates; joint Z is assumed zero')
    axes[1, 1].plot(times, [q['model_rms_m'] or 0 for q in quality], label='Constraint RMS')
    bad = [q['data_time_s'] for q in quality if not q['integrated']]
    axes[1, 1].plot(bad, [0] * len(bad), 'rx', label='No integration')
    axes[1, 1].set(ylabel='Metres per update', title='Model disagreement, not a complete slip detector')
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.grid(alpha=.2)
    for ax in [axes[0, 1], *axes[1]]:
        ax.set_xlabel('Input host-log seconds')
    fig.suptitle(path.parent.name + '\nShadow articulated pose — provisional geometry, approximate clock alignment')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    for path in args.logs:
        report = build_pose_log(path, args.calibration, args.output / path.stem)
        print(f'{path.name}: {report["pose_messages"]} poses, '
              f'joint={report["joint_final"]}, rejected={report["reasons"]}')


if __name__ == '__main__':
    main()
