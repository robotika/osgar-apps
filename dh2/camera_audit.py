"""Audit recorded OAK clock variation and depth quantization without fitting geometry."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_config, lookup_stream_names


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(samples=0)
    return dict(samples=len(values), minimum=float(np.min(values)), median=float(np.median(values)),
                p90=float(np.percentile(values, 90)), maximum=float(np.max(values)))


def clock_variation(receipts, acquisitions):
    """Relative receipt variation only: separate clock origins and unknown latency."""
    differences = np.asarray(receipts) - np.asarray(acquisitions)
    if len(differences) < 2 or not np.isfinite(differences).all():
        raise ValueError('Need at least two finite receipt/acquisition pairs')
    periods = np.diff(acquisitions)
    if np.any(periods <= 0):
        raise ValueError('Acquisition clock repeated or reset; analyze separate segments')
    return dict(receipt_minus_acquisition_s=distribution(differences),
                excess_above_minimum_s=distribution(differences - np.min(differences)),
                first_excess_s=float(differences[0] - np.min(differences)),
                acquisition_period_s=distribution(periods))


def calibration_geometry(path, shape):
    raw = path.read_bytes()
    calibration = json.loads(raw)
    eeprom = calibration['eeprom']
    left = eeprom['stereoRectificationData']['leftCameraSocket']
    right = eeprom['stereoRectificationData']['rightCameraSocket']
    camera = dict(eeprom['cameraData'])[left]
    extrinsics = camera['extrinsics']
    if extrinsics['toCameraSocket'] != right:
        raise ValueError('Expected directly calibrated left-to-right extrinsics')
    factors = {0: 1., 1: .01, 2: .001}  # DepthAI LengthUnit enum: metre, centimetre, millimetre.
    scale = factors[extrinsics['lengthUnit']]
    translation = [extrinsics['translation'][axis] * scale for axis in 'xyz']
    baseline = float(np.linalg.norm(translation))
    height, width = shape
    if camera['width'] * height != camera['height'] * width:
        raise ValueError('Recorded aspect ratio differs from factory image; crop must be reviewed')
    resized = np.asarray(camera['intrinsicMatrix'], dtype=float).copy()
    resized[0] *= width / camera['width']
    resized[1] *= height / camera['height']
    return dict(device_id=calibration['device_id'], calibration_sha256=hashlib.sha256(raw).hexdigest(),
                depthai_capture_version=calibration['depthai_version'],
                physical_left_resized_intrinsics=resized.tolist(), baseline_m=baseline,
                focal_baseline_m_pixel=float(resized[0, 0] * baseline),
                verified_rectified_projection=False)


def quantization_check(histogram, focal_baseline):
    """Test candidate integer disparity levels, retaining millimetre rounding error."""
    millimetres = np.flatnonzero(histogram)
    counts = histogram[millimetres]
    depth = millimetres / 1000
    disparity = np.maximum(1, np.rint(focal_baseline / depth))
    prediction = focal_baseline / disparity
    errors = np.abs(depth - prediction)
    total = int(np.sum(counts))
    top = np.argsort(counts)[-12:][::-1]
    return dict(sampled_points=total,
                fraction_within_1mm_integer_disparity=float(np.sum(counts[errors <= .001]) / total) if total else None,
                common_depths=[dict(depth_m=float(depth[i]), count=int(counts[i]),
                                    candidate_disparity=int(disparity[i]), residual_m=float(errors[i])) for i in top])


def audit(logfile, calibration):
    names = lookup_stream_names(str(logfile))
    streams = ['oak.depth_seq', 'oak.color_seq', 'oak.orientation_list', 'oak.depth']
    ids = {names.index(n) + 1: n for n in streams}
    times = {n: [] for n in streams[:-1]}
    source_times = {n: [] for n in streams[:-1]}
    sequences = {n: [] for n in streams[:2]}
    histogram = np.zeros(3001, dtype=np.int64)
    pending_depth = None
    payload_lags = []
    unpaired = 0
    shape = None
    with LogReader(str(logfile), only_stream_id=list(ids)) as log:
        for stamp, sid, raw in log:
            name = ids[sid]
            host = stamp.total_seconds()
            data = deserialize(raw)
            if name == 'oak.depth':
                if pending_depth is None:
                    unpaired += 1
                else:
                    payload_lags.append(host - pending_depth)
                    pending_depth = None
                depth = np.asarray(data)
                if depth.ndim != 2 or (shape is not None and shape != depth.shape):
                    raise ValueError('Expected constant two-dimensional depth shape')
                shape = depth.shape
                samples = depth[::8, ::8]
                valid = samples[(samples >= 300) & (samples <= 3000)]
                if np.any(valid != np.floor(valid)):
                    raise ValueError('Expected integer millimetre depth values')
                histogram += np.bincount(valid.astype(int), minlength=len(histogram))
            elif name == 'oak.orientation_list':
                if data:
                    times[name].append(host)
                    source_times[name].append(data[-1][0])
            else:
                times[name].append(host)
                source_times[name].append(data[1] / 1e6)
                sequences[name].append(data[0])
                if name == 'oak.depth_seq':
                    if pending_depth is not None:
                        unpaired += 1
                    pending_depth = host
    if shape is None:
        raise ValueError('No depth frames')
    geometry = calibration_geometry(calibration, shape)
    clock = {n: clock_variation(times[n], source_times[n]) for n in times}
    for name, values in sequences.items():
        steps = np.diff(values)
        clock[name]['skipped_sequence_numbers'] = int(np.sum(np.maximum(0, steps - 1)))
        clock[name]['sequence_repeats_or_resets'] = int(np.sum(steps <= 0))
    settings = lookup_config(str(logfile))['robot']['modules']['oak']['init']
    return dict(logfile=str(logfile), depth_shape=list(shape), oak_settings=settings,
                geometry=geometry, clocks=clock, sequence_to_payload_log_lag_s=distribution(payload_lags),
                unpaired_depth_messages=unpaired + int(pending_depth is not None),
                quantization=quantization_check(histogram, geometry['focal_baseline_m_pixel']),
                limitations=['Receipt variation does not identify absolute acquisition latency',
                             'Depth SDK-host and IMU device clock relationship is absent from old logs',
                             'Sequence/payload pairing follows driver order, not a payload sequence identifier',
                             'Integer disparity consistency does not verify principal point or rectified projection',
                             'Current reviewed v3 driver reads depth_profile, not stereo_mode',
                             'Factory capture SDK version does not establish the recording SDK version'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reports = [audit(path, args.calibration) for path in args.logs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(reports, indent=2, allow_nan=False) + '\n')
    for report in reports:
        print(Path(report['logfile']).name, report['quantization']['fraction_within_1mm_integer_disparity'])


if __name__ == '__main__':
    main()
