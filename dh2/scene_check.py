"""Offline projective depth consistency check; no registration or pose fitting."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from osgar.lib import quaternion as quat
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_config, lookup_stream_names

from dh2.pose_replay import OPTICAL_TO_BODY
from matty.pose3d import unit_quaternion


def read_poses(path):
    names = lookup_stream_names(str(path))
    ids = {names.index('pose.' + n) + 1: n for n in ['pose3d', 'pose_quality']}
    times, positions, rotations, continuous = [], [], [], []
    latest = None
    with LogReader(str(path), only_stream_id=list(ids)) as log:
        for _, sid, raw in log:
            value = deserialize(raw)
            if ids[sid] == 'pose3d':
                latest = value
            elif latest is not None:
                times.append(value['data_time_s'])
                positions.append(latest[0])
                rotations.append(unit_quaternion(latest[1]))
                continuous.append(value['translation_continuous'])
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError('Need increasing pose acquisition times')
    return np.array(times), np.array(positions), rotations, continuous


def pose_at(t, poses, max_gap=.3):
    times, positions, rotations, continuous = poses
    i = int(np.searchsorted(times, t))
    if i < len(times) and times[i] == t:
        return (positions[i], rotations[i]) if continuous[i] else None
    if i == 0 or i == len(times) or times[i] - times[i - 1] > max_gap:
        return None
    if not continuous[i - 1] or not continuous[i]:
        return None
    fraction = (t - times[i - 1]) / (times[i] - times[i - 1])
    return (positions[i - 1] * (1 - fraction) + positions[i] * fraction,
            unit_quaternion(quat.slerp(rotations[i - 1], rotations[i], fraction)))


def camera_pose(pose, offset):
    position, q = pose
    rotation = np.asarray(quat.rotation_matrix(q))
    return position + rotation @ offset, rotation @ OPTICAL_TO_BODY


def smooth_depth(depth, min_range, max_range, edge_limit):
    """Only interior finite depths with four valid, similar neighbours."""
    valid = np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    mask = np.zeros(depth.shape, dtype=bool)
    center = depth[1:-1, 1:-1]
    interior = valid[1:-1, 1:-1].copy()
    for neighbour, good in [(depth[:-2, 1:-1], valid[:-2, 1:-1]),
                             (depth[2:, 1:-1], valid[2:, 1:-1]),
                             (depth[1:-1, :-2], valid[1:-1, :-2]),
                             (depth[1:-1, 2:], valid[1:-1, 2:])]:
        interior &= good & (np.abs(center - neighbour) <= edge_limit)
    mask[1:-1, 1:-1] = interior
    return mask


def projective_errors(source, target, source_pose, target_pose, intrinsics, *,
                      stride=4, min_range=.3, max_range=3, edge_limit=.08):
    """Warp source points into target; return signed optical-Z residuals.

    Validity/edge masks are independent of residual magnitude. No ICP, residual
    clipping or fitted correction can make a bad pose look registered here.
    Nearest-pixel association includes occlusion and moving-object errors.
    """
    fx, fy, cx, cy = intrinsics
    mask = smooth_depth(source, min_range, max_range, edge_limit)
    v, u = np.mgrid[0:source.shape[0]:stride, 0:source.shape[1]:stride]
    good = mask[::stride, ::stride]
    u, v = u[good], v[good]
    z = source[v, u]
    points = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    sp, sr = source_pose
    tp, tr = target_pose
    predicted = ((points @ sr.T + sp - tp) @ tr)
    positive = predicted[:, 2] > 0
    predicted = predicted[positive]
    uu = np.rint(fx * predicted[:, 0] / predicted[:, 2] + cx).astype(int)
    vv = np.rint(fy * predicted[:, 1] / predicted[:, 2] + cy).astype(int)
    inside = (uu >= 0) & (uu < target.shape[1]) & (vv >= 0) & (vv < target.shape[0])
    uu, vv, predicted = uu[inside], vv[inside], predicted[inside]
    match = smooth_depth(target, min_range, max_range, edge_limit)[vv, uu]
    return predicted[match, 2] - target[vv[match], uu[match]], len(points)


def depth_frames(path, poses, args):
    names = lookup_stream_names(str(path))
    last = -np.inf
    shape = None
    with LogReader(str(path), only_stream_id=names.index('oak.depth') + 1) as log:
        for timestamp, _, raw in log:
            host = timestamp.total_seconds()
            if host < args.start or (args.end is not None and host > args.end) or host - last < args.frame_step:
                continue
            t = host + args.depth_time_offset
            pose = pose_at(t, poses)
            if pose is None:
                continue
            depth = np.asarray(deserialize(raw), dtype=float) / 1000
            if depth.ndim != 2 or (shape is not None and shape != depth.shape):
                raise ValueError('Depth must have a constant two-dimensional shape')
            shape = depth.shape
            last = host
            yield t, depth, pose


def summarize(values):
    if not len(values):
        return dict(matches=0, median_abs_m=None, p90_abs_m=None, signed_median_m=None)
    return dict(matches=len(values), median_abs_m=float(np.median(np.abs(values))),
                p90_abs_m=float(np.percentile(np.abs(values), 90)), signed_median_m=float(np.median(values)))


def check_scene(args):
    poses = read_poses(args.pose_log)
    offset = np.asarray(args.optical_offset)
    rows, collected = [], {n: [] for n in ['fixed_camera', 'fixed_joint', 'articulated']}
    reference = None
    for timestamp, depth, pose in depth_frames(args.logfile, poses, args):
        if reference is None:
            reference = timestamp, depth, pose
            continue
        t0, depth0, pose0 = reference
        if depth.shape != depth0.shape:
            raise ValueError('Depth shape changed')
        first = camera_pose(pose0, offset)
        modes = dict(fixed_camera=first, fixed_joint=camera_pose((pose0[0], pose[1]), offset),
                     articulated=camera_pose(pose, offset))
        for name, current in modes.items():
            # Symmetric comparison exposes losses of overlap in either direction.
            forward, n0 = projective_errors(depth0, depth, first, current, args.intrinsics,
                                           stride=args.pixel_stride)
            backward, n1 = projective_errors(depth, depth0, current, first, args.intrinsics,
                                             stride=args.pixel_stride)
            errors = np.concatenate((forward, backward))
            stats = summarize(errors)
            row = dict(reference_time=t0, time=timestamp, mode=name,
                       yaw_change_deg=float(np.degrees(quat.heading(pose[1]) - quat.heading(pose0[1]))),
                       candidate_points=n0 + n1, overlap_fraction=len(errors) / max(1, n0 + n1), **stats)
            rows.append(row)
            collected[name].extend(errors)
    if not rows:
        raise ValueError('No frame pairs with continuous pose coverage in requested interval')
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / 'pairs.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = dict(settings={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                  pose_settings=lookup_config(str(args.pose_log))['robot']['modules']['pose']['init'],
                  usable_for_driving=False, acceptance_threshold_m=None,
                  projection=dict(min_range_m=.3, max_range_m=3, neighbour_edge_limit_m=.08,
                                  depth_units='millimetres', target_sampling='nearest_pixel',
                                  depth_frame='nominal_rectified_left'),
                  summaries={name: summarize(values) for name, values in collected.items()},
                  reference_time=reference[0], pairs=len(rows) // 3,
                  limitations=['Diagnostic only; no acceptance threshold or motion protection',
                               'Explicit supplied intrinsics and optical centre offset are assumptions',
                               'Nominal rectified-left optical rotation relative to front body',
                               'Depth host receipt time plus offset; exact acquisition alignment unverified',
                               'Joint Z assumed; unknown-translation intervals excluded',
                               'Occlusions, moving objects, depth noise and nearest-pixel errors included',
                               'Planar surfaces alone cannot constrain all translation components',
                               'Pooled pixel statistics are correlated, not independent accuracy samples'])
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    render(args.output / 'residuals.png', rows)
    return report


def render(path, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    for mode in ['fixed_camera', 'fixed_joint', 'articulated']:
        values = [r for r in rows if r['mode'] == mode]
        for ax, key in zip(axes, ['median_abs_m', 'p90_abs_m', 'overlap_fraction']):
            ax.plot([r['time'] for r in values], [r[key] for r in values], label=mode)
    for ax, title in zip(axes, ['Median absolute depth residual (m)', '90th percentile (m)', 'Valid overlap fraction']):
        ax.set_ylabel(title)
        ax.grid(alpha=.2)
        ax.legend()
    axes[-1].set_xlabel('Depth host-log seconds plus supplied offset')
    fig.suptitle('Static-scene diagnostic: fixed reference, no fitted alignment\n'
                 'Provisional projection and timing; occlusion/moving objects included')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', type=Path)
    parser.add_argument('--pose-log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--intrinsics', type=float, nargs=4, required=True, metavar=('FX', 'FY', 'CX', 'CY'))
    parser.add_argument('--optical-offset', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'),
                        help='Joint to rectified-left optical centre in front forward/left/up metres')
    parser.add_argument('--depth-time-offset', type=float, default=0)
    parser.add_argument('--frame-step', type=float, default=.5)
    parser.add_argument('--pixel-stride', type=int, default=4)
    parser.add_argument('--start', type=float, default=0)
    parser.add_argument('--end', type=float)
    args = parser.parse_args()
    numbers = [*args.intrinsics, *args.optical_offset, args.depth_time_offset, args.frame_step, args.start]
    if (not np.isfinite(numbers).all() or min(args.intrinsics[:2]) <= 0 or args.frame_step <= 0
            or args.pixel_stride < 1 or (args.end is not None and
                                        (not np.isfinite(args.end) or args.end <= args.start))):
        parser.error('Expected finite geometry/time values, positive focal lengths and sampling steps')
    result = check_scene(args)
    print(json.dumps(result['summaries'], indent=2))


if __name__ == '__main__':
    main()
