"""Offline Matty wheel-distance audit; no replacement odometry or driving output."""

import argparse
import csv
import json
import struct
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_config, lookup_stream_names
from osgar.platforms.matty import ESC, SYNC, Matty, RobotStatus, remove_esc_chars


@dataclass
class Status:
    time: float
    counter: int
    flags: int
    speed: int
    joint: int
    encoders: list
    rpy: list | None


class OfflineBus:
    def register(self, *names):
        pass


class PacketDecoder(Matty):
    """Temporary offline adapter: reuse Matty escaping, checksum and status constants.

    Complete packets at their final byte, rather than waiting for the next SYNC
    as the legacy driver does. The last packet is therefore included at EOF.
    Never starts a thread or invokes the platform's motor/odometry handlers.
    """

    def __init__(self):
        super().__init__({}, OfflineBus())
        self.frame = bytearray()
        self.escaped = False
        self.statuses = []
        self.resets = []
        self.versions = []
        self.info_counters = []
        self.counts = Counter()
        self.packet_time = 0

    def feed(self, data, timestamp):
        self.packet_time = timestamp
        for value in data:
            if value == SYNC:
                if self.frame:
                    self.counts['resynchronizations'] += 1
                self.frame = bytearray([SYNC])
                self.escaped = False
                continue
            if not self.frame:
                self.counts['bytes_before_sync'] += 1
                continue
            self.frame.append(value)
            if value == ESC and not self.escaped:
                self.escaped = True
                continue
            if self.escaped and value not in ((~SYNC) & 255, (~ESC) & 255):
                self.counts['invalid_escapes'] += 1
                self.escaped = False
                self.frame.clear()
                continue
            self.escaped = False
            decoded = remove_esc_chars(bytes(self.frame))
            if len(decoded) >= 2 and len(decoded) == decoded[1] + 3:
                before = self.counts['valid_packets']
                self.process_msg(decoded)  # Core Matty verifies the checksum.
                if self.counts['valid_packets'] == before:
                    self.counts['checksum_rejections'] += 1
                self.frame.clear()

    def process_esp_packet(self, packet):
        self.counts['valid_packets'] += 1
        if len(packet) < 2:
            self.counts['short_packets'] += 1
            return
        kind = chr(packet[1])
        self.counts['type_' + kind] += 1
        if kind in ('I', 'P', 'V'):
            self.info_counters.append(packet[0])
        if kind == 'I' and len(packet) in (20, 26):
            counter, _, flags, _, _, _, speed, joint, *enc = struct.unpack('<BBBBHHhh4H', packet[:20])
            rpy = list(struct.unpack('<hhh', packet[20:])) if len(packet) == 26 else None
            self.statuses.append(Status(self.packet_time, counter, flags, speed, joint, enc, rpy))
        elif kind == 'I':
            self.counts['unsupported_status_size'] += 1
        elif kind == 'R':
            self.resets.append(self.packet_time)
        elif kind == 'V' and len(packet) == 4:
            self.versions.append(dict(time=self.packet_time, robot=packet[2], version=packet[3]))


def signed_delta(current, previous):
    return [(int(c) - int(p) + 32768) % 65536 - 32768 for c, p in zip(current, previous)]


def analyze_intervals(statuses, resets=(), *, max_gap=0.5, max_wheel_speed=1.5):
    """Compare like intervals; rejected intervals never become accepted travel."""
    if not (0 < max_gap < float('inf') and 0 < max_wheel_speed < float('inf')
            and max_gap * max_wheel_speed * 1000 + 2 < 32768):
        raise ValueError('Positive finite bounds must limit wheel travel below half the 16-bit modulus')
    rows = []
    cumulative = np.zeros(4)
    total_speed = total_fixed = 0.0
    resets = sorted(resets)
    for i, sample in enumerate(statuses):
        previous = statuses[i - 1] if i else None
        reasons = []
        dt = sample.time - previous.time if previous else 0
        delta = signed_delta(sample.encoders, previous.encoders) if previous else [0] * 4
        step = (sample.counter - previous.counter) % 256 if previous else 0
        # Equal host timestamps cannot establish command/status ordering: reject
        # both adjoining intervals conservatively rather than miss a reset.
        reset_between = previous is not None and any(previous.time <= t <= sample.time for t in resets)
        if previous is None:
            reasons.append('first_sample')
        else:
            if dt <= 0:
                reasons.append('nonpositive_interval')
            elif dt > max_gap:
                reasons.append('time_gap')
            if step == 0 or step >= 128:
                reasons.append('counter_repeat_or_restart')
            if reset_between:
                reasons.append('reset_command')
            if sample.flags & RobotStatus.ERROR_ENCODER.value:
                reasons.append('encoder_error')
            if previous.flags & RobotStatus.ERROR_ENCODER.value:
                reasons.append('after_encoder_error')
            if sample.flags & RobotStatus.ERROR_POWER.value or previous.flags & RobotStatus.ERROR_POWER.value:
                reasons.append('power_error')
            if dt > 0 and max(abs(x) for x in delta) > max_wheel_speed * 1000 * dt + 2:
                reasons.append('encoder_jump_or_reset')
        accepted = not reasons
        speed_mm = sample.speed * dt if previous else 0
        fixed_mm = sample.speed * .1 if previous else 0
        if accepted:
            cumulative += delta
            total_speed += speed_mm
            total_fixed += fixed_mm
        row = dict(time_s=sample.time, counter=sample.counter, flags=sample.flags,
                   speed_mm_s=sample.speed, joint_deg=sample.joint / 100,
                   dt_s=dt, counter_step=step, accepted=accepted, reason=';'.join(reasons),
                   wheel_mean_delta_mm=float(np.mean(delta)), speed_dt_mm=speed_mm,
                   residual_mm=float(np.mean(delta)) - speed_mm,
                   cumulative_wheel_mean_mm=float(np.mean(cumulative)),
                   cumulative_speed_dt_mm=total_speed, cumulative_fixed_100ms_mm=total_fixed)
        for wheel in range(4):
            row[f'encoder_{wheel}_mm'] = sample.encoders[wheel]
            row[f'delta_{wheel}_mm'] = delta[wheel]
            row[f'cumulative_{wheel}_mm'] = cumulative[wheel]
        rows.append(row)
    return rows


def incoming_stream(config, names, override=None):
    if override:
        if override not in names:
            raise ValueError(f'No stream {override}')
        return override
    sources = [src for src, dst in config.get('robot', {}).get('links', [])
               if dst == 'platform.esp_data' and src in names]
    if len(sources) != 1:
        raise ValueError('Cannot identify incoming Matty status stream; pass --stream explicitly')
    return sources[0]


def read_recording(path, stream=None):
    names = lookup_stream_names(str(path))
    config = lookup_config(str(path)) or {}
    source = incoming_stream(config, names, stream)
    telemetry = ('platform.pose2d', 'app.desired_steering', 'platform.bumpers_front',
                 'platform.bumpers_rear', 'platform.encoders', 'oak.gyro_raw_list')
    selected = [source, *[n for n in telemetry if n in names]]
    if 'platform.esp_data' in names and source != 'platform.esp_data':
        selected.append('platform.esp_data')
    ids = {names.index(n) + 1: n for n in selected}
    data = {n: [] for n in telemetry}
    decoder, outgoing = PacketDecoder(), PacketDecoder()
    with LogReader(str(path), only_stream_id=list(ids)) as log:
        for timestamp, channel, raw in log:
            sec = timestamp.total_seconds()
            name = ids[channel]
            value = deserialize(raw)
            if name == source:
                decoder.feed(value, sec)
            elif name == 'platform.esp_data':
                outgoing.feed(value, sec)
            elif name == 'oak.gyro_raw_list':
                data[name].extend([sec, *sample] for sample in value)
            else:
                data[name].append([sec, value])
    return source, decoder, outgoing, data


def audit(path, output, *, stream=None, max_gap=0.5, max_wheel_speed=1.5):
    source, decoder, outgoing, data = read_recording(path, stream)
    if len(decoder.statuses) < 2:
        raise ValueError(f'{path}: fewer than two status packets on {source}')
    rows = analyze_intervals(decoder.statuses, outgoing.resets, max_gap=max_gap, max_wheel_speed=max_wheel_speed)
    valid = [r for r in rows if r['accepted']]
    rejected = Counter(reason for r in rows for reason in r['reason'].split(';') if reason)
    residuals = np.array([r['residual_mm'] for r in valid])
    counters = sum(max(0, r['counter_step'] - 1) for r in rows[1:] if 0 < r['counter_step'] < 128)
    info_steps = [(b - a) % 256 for a, b in zip(decoder.info_counters, decoder.info_counters[1:])]
    summary = dict(logfile=str(path), incoming_stream=source, status_packets=len(rows),
                   encoder_unit='unsigned cumulative millimetres modulo 65536; firmware index order',
                   parser=dict(decoder.counts), incomplete_tail_bytes=len(decoder.frame),
                   firmware_versions=decoder.versions, reset_commands=outgoing.resets,
                   accepted_intervals=len(valid), rejected_reasons=dict(rejected),
                   status_counter_extra_steps=counters,
                   shared_info_counter_missing=sum(s - 1 for s in info_steps if 1 < s < 128),
                   shared_info_counter_repeats_or_restarts=sum(s == 0 or s >= 128 for s in info_steps),
                   encoder_error_samples=sum(bool(s.flags & RobotStatus.ERROR_ENCODER.value) for s in decoder.statuses),
                   power_error_samples=sum(bool(s.flags & RobotStatus.ERROR_POWER.value) for s in decoder.statuses),
                   interval_seconds=dict(median=float(np.median([r['dt_s'] for r in rows[1:]])),
                                         maximum=max(r['dt_s'] for r in rows[1:])),
                   accepted_wheel_signed_mm=[rows[-1][f'cumulative_{w}_mm'] for w in range(4)],
                   accepted_wheel_mean_mm=rows[-1]['cumulative_wheel_mean_mm'],
                   accepted_speed_dt_mm=rows[-1]['cumulative_speed_dt_mm'],
                   accepted_fixed_100ms_mm=rows[-1]['cumulative_fixed_100ms_mm'],
                   residual_mm=dict(median=float(np.median(residuals)),
                                    abs_p90=float(np.percentile(abs(residuals), 90)),
                                    abs_max=float(max(abs(residuals)))) if len(valid) else None,
                   settings=dict(max_gap=max_gap, max_wheel_speed=max_wheel_speed, quantization_allowance_mm=2),
                   limitations=['Wheel indices are firmware order; physical front/rear mapping needs confirmation',
                                'Only accepted intervals enter cumulative comparisons; rejected travel is unknown',
                                'Wheel mean is not a validated joint-centre or camera displacement',
                                'Counter gaps can cover recoverable cumulative travel; resets can remain ambiguous',
                                'Status counter also advances for GPS/version packets; extra steps are not packet loss',
                                'Host receive batches can contain multiple statuses with no individual capture time',
                                'No slip correction or absolute ground-truth distance is inferred'])
    output.mkdir(parents=True, exist_ok=True)
    (output / 'report.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    with (output / 'intervals.csv').open('w') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output / 'status.jsonl').open('w') as file:
        for status in decoder.statuses:
            file.write(json.dumps(asdict(status)) + '\n')
    render(output / 'audit.png', rows, decoder.statuses, data)
    return summary


def render(path, rows, statuses, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 2, figsize=(14, 12), constrained_layout=True)
    times = np.array([r['time_s'] for r in rows])
    valid = np.array([r['accepted'] for r in rows])
    cumulative = np.array([[r[f'cumulative_{w}_mm'] for w in range(4)] for r in rows])
    ax = axes[0, 0]
    for i in range(4):
        ax.plot(times, cumulative[:, i], label=f'wheel {i}')
    ax.set_title('Accepted signed wheel travel (firmware indices)')
    ax.set_ylabel('mm')
    ax.legend()
    ax = axes[0, 1]
    for key, label in [('cumulative_wheel_mean_mm', 'Wheel mean'),
                       ('cumulative_speed_dt_mm', 'Reported speed x actual dt'),
                       ('cumulative_fixed_100ms_mm', 'Reported speed x fixed 0.1 s')]:
        ax.plot(times, [r[key] for r in rows], label=label)
    ax.set_title('Comparisons on accepted intervals only')
    ax.set_ylabel('mm')
    ax.legend()
    ax = axes[1, 0]
    ax.plot(times, (cumulative[:, 0] + cumulative[:, 1]) / 2, label='indices 0/1 mean')
    ax.plot(times, (cumulative[:, 2] + cumulative[:, 3]) / 2, label='indices 2/3 mean')
    ax.plot(times, cumulative[:, 1] - cumulative[:, 0], label='1 minus 0')
    ax.plot(times, cumulative[:, 3] - cumulative[:, 2], label='3 minus 2')
    ax.set_title('Pair means and differences (not slip labels)')
    ax.set_ylabel('mm')
    ax.legend(fontsize=8)
    ax = axes[1, 1]
    residual = np.array([r['residual_mm'] for r in rows])
    ax.plot(times[valid], residual[valid], '.', ms=2)
    ax.plot(times[~valid], np.zeros(sum(~valid)), 'rx', label='rejected interval')
    ax.set_title('Wheel mean increment minus reported speed x dt')
    ax.set_ylabel('mm')
    ax.legend()
    ax = axes[2, 0]
    ax.plot(times, [s.joint / 100 for s in statuses], label='measured joint')
    ax.set_ylabel('degrees')
    gyro = np.asarray(data['oak.gyro_raw_list'])
    if len(gyro):
        twin = ax.twinx()
        twin.plot(gyro[:, 0], np.linalg.norm(gyro[:, 2:5], axis=1), c='orange', alpha=.5)
        twin.set_ylabel('Raw gyro norm (rad/s), host receipt time')
    ax.set_title('Articulation and camera angular motion')
    ax = axes[2, 1]
    rpy = [(s.time, *s.rpy) for s in statuses if s.rpy is not None]
    if rpy:
        rp = np.asarray(rpy)
        ax.plot(rp[:, 0], rp[:, 1] / 100, label='ESP roll')
        ax.plot(rp[:, 0], rp[:, 2] / 100, label='ESP pitch')
        ax.legend()
    ax.set_title('ESP32 attitude (its own frame)')
    ax.set_ylabel('degrees')
    ax = axes[3, 0]
    poses = data['platform.pose2d']
    if poses:
        xy = np.array([v[:2] for _, v in poses]) / 1000
        ax.plot(xy[:, 0], xy[:, 1])
        ax.set_aspect('equal', adjustable='datalim')
    ax.set(title='Legacy pose2d (not ground truth)', xlabel='X (m)', ylabel='Y (m)')
    ax = axes[3, 1]
    ax.plot(times, [s.speed for s in statuses], label='reported speed')
    commands = data['app.desired_steering']
    if commands:
        ax.step([t for t, _ in commands], [v[0] for _, v in commands], where='post', label='desired speed')
    for name, color in [('platform.bumpers_front', 'red'), ('platform.bumpers_rear', 'purple')]:
        for t, pressed in data[name]:
            if pressed:
                ax.axvline(t, color=color, alpha=.5)
    ax.set_title('Commands / reported speed; contacts red/purple')
    ax.set_ylabel('mm/s')
    ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=.2)
        if ax is not axes[3, 0]:
            ax.set_xlabel('Host log seconds (status packet completion)')
    fig.suptitle(Path(path.parent).name + '\nEncoder audit: no reconstructed joint pose or slip correction')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', type=Path, nargs='+')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stream', help='Override incoming wire stream (normally derived as serial.raw)')
    parser.add_argument('--max-gap', type=float, default=.5, help='Largest accepted receive interval, seconds')
    parser.add_argument('--max-wheel-speed', type=float, default=1.5, help='Plausibility bound per wheel, m/s')
    args = parser.parse_args()
    if not (0 < args.max_gap < float('inf') and 0 < args.max_wheel_speed < float('inf')
            and args.max_gap * args.max_wheel_speed * 1000 + 2 < 32768):
        parser.error('Positive finite bounds must limit wheel travel below half the 16-bit modulus')
    for path in args.logs:
        report = audit(path, args.output / path.stem, stream=args.stream,
                       max_gap=args.max_gap, max_wheel_speed=args.max_wheel_speed)
        print(f'{path.name}: {report["status_packets"]} statuses, '
              f'{report["accepted_intervals"]} accepted intervals, '
              f'wheel mean={report["accepted_wheel_mean_mm"]:.2f} mm, '
              f'speed integral={report["accepted_speed_dt_mm"]:.2f} mm, '
              f'rejected={report["rejected_reasons"]}')


if __name__ == '__main__':
    main()
