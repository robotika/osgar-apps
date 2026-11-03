"""Measure local planarity and cross-frame consistency of manually selected depth patches."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_config, lookup_stream_names

from dh2.scene_check import camera_pose, pose_at, read_poses, smooth_depth, summarize


def fit_plane(points):
    """Least-squares plane; all points retained, with no robust trimming."""
    if len(points) < 30 or not np.isfinite(points).all():
        raise ValueError('Need at least 30 finite patch points')
    center = np.mean(points, axis=0)
    _, values, vectors = np.linalg.svd(points - center, full_matrices=False)
    if values[1] < 1e-6:
        raise ValueError('Patch points do not span a plane')
    normal = vectors[-1]
    if normal[2] > 0:
        normal = -normal
    residuals = (points - center) @ normal
    return center, normal, residuals


def patch_points(depth, roi, intrinsics):
    x0, y0, x1, y1 = roi
    h, w = depth.shape
    if not (0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h):
        raise ValueError(f'Patch ROI outside depth image: {roi}')
    mask = smooth_depth(depth, .3, 3, .08)
    v, u = np.mgrid[y0:y1, x0:x1]
    valid = mask[v, u]
    u, v = u[valid], v[valid]
    z = depth[v, u]
    fx, fy, cx, cy = intrinsics
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)), len(valid.ravel())


def selected_frames(path, requested, tolerance=.06):
    """Nearest recorded depth frame to each annotation; return actual host times."""
    names = lookup_stream_names(str(path))
    selected = {}
    with LogReader(str(path), only_stream_id=names.index('oak.depth') + 1) as log:
        for stamp, _, raw in log:
            t = stamp.total_seconds()
            near = [q for q in requested if abs(t - q) <= tolerance
                    and (q not in selected or abs(t - q) < abs(selected[q][0] - q))]
            if near:
                depth = np.asarray(deserialize(raw), dtype=float) / 1000
                for q in near:
                    selected[q] = t, depth
    missing = set(requested) - selected.keys()
    if missing:
        raise ValueError(f'No depth frames within {tolerance}s of {sorted(missing)}')
    return selected


def measure_patch(points, pose, reference, offset):
    center, normal, residuals = fit_plane(points)
    local = summarize(residuals)
    world_p, world_r = camera_pose(pose, offset)
    world = points @ world_r.T + world_p
    world_center = center @ world_r.T + world_p
    world_normal = world_r @ normal
    if reference is None:
        reference = dict(center=world_center, normal=world_normal, joint=pose[0].copy())
    rows = []
    for mode, position in [('fixed_joint', reference['joint']), ('articulated', pose[0])]:
        p, r = camera_pose((position, pose[1]), offset)
        projected = points @ r.T + p
        errors = (projected - reference['center']) @ reference['normal']
        cross = summarize(errors)
        rows.append(dict(mode=mode, local_median_abs_m=local['median_abs_m'],
                         local_p90_abs_m=local['p90_abs_m'], reference_median_abs_m=cross['median_abs_m'],
                         reference_p90_abs_m=cross['p90_abs_m'], reference_signed_median_m=cross['signed_median_m']))
    diagnostics = dict(points=len(points), local_center_optical=center.tolist(),
                       normal_world=world_normal.tolist(), centroid_world=world_center.tolist(),
                       normal_change_deg=float(np.degrees(np.arccos(np.clip(
                           abs(world_normal @ reference['normal']), 0, 1)))),
                       median_world_z_m=float(np.median(world[:, 2])),
                       normal_angle_from_vertical_deg=float(np.degrees(np.arccos(np.clip(
                           abs(world_normal[2]), 0, 1)))))
    depths, counts = np.unique(points[:, 2], return_counts=True)
    diagnostics.update(median_optical_z_m=float(np.median(points[:, 2])),
                       distinct_depth_values=len(depths), dominant_depth_m=float(depths[np.argmax(counts)]),
                       dominant_depth_fraction=float(np.max(counts) / len(points)))
    return rows, diagnostics, reference


def check_patches(args):
    definition = json.loads(args.annotations.read_text())
    intrinsics = definition['intrinsics']
    offset = np.asarray(definition['optical_offset'])
    all_rows, results, pose_settings = [], [], {}
    args.output.mkdir(parents=True, exist_ok=True)
    for run in definition['runs']:
        source = args.logs_dir / run['logfile']
        pose_path = args.pose_dir / source.stem / 'pose.log'
        pose_settings[source.name] = lookup_config(str(pose_path))['robot']['modules']['pose']['init']
        poses = read_poses(pose_path)
        requested = sorted({obs['time'] for patch in run['patches'] for obs in patch['observations']})
        frames = selected_frames(source, requested)
        annotations = {}
        for patch in run['patches']:
            if np.any(np.diff([obs['time'] for obs in patch['observations']]) <= 0):
                raise ValueError('Patch observation times must increase')
            reference = None
            for observation in patch['observations']:
                actual, depth = frames[observation['time']]
                if list(depth.shape) != definition['depth_shape']:
                    raise ValueError('Annotation image dimensions differ from recording')
                pose = pose_at(actual + args.depth_time_offset, poses)
                if pose is None:
                    raise ValueError(f'No continuous pose coverage at {source.name} {actual}')
                points, candidates = patch_points(depth, observation['roi'], intrinsics)
                rows, diagnostic, reference = measure_patch(points, pose, reference, offset)
                diagnostic.update(logfile=source.name, patch=patch['name'], kind=patch['kind'],
                                  requested_time=observation['time'], actual_time=actual, roi=observation['roi'],
                                  valid_fraction=len(points) / candidates,
                                  approximate_one_pixel_disparity_step_z_m=float(
                                      np.median(points[:, 2]) ** 2 / (intrinsics[0] * definition['baseline_m'])))
                results.append(diagnostic)
                for row in rows:
                    row.update(logfile=source.name, patch=patch['name'], kind=patch['kind'], time=actual,
                               points=len(points), normal_change_deg=diagnostic['normal_change_deg'])
                    all_rows.append(row)
                annotations.setdefault(actual, []).append((patch['name'], observation['roi']))
        render_annotations(args.output / (source.stem + '-patches.png'), frames, annotations)
    with (args.output / 'patches.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    heights = []
    for observation in results:
        if observation['kind'] != 'raised':
            continue
        floors = [o for o in results if o['kind'] == 'floor' and o['logfile'] == observation['logfile']
                  and o['actual_time'] == observation['actual_time']]
        if len(floors) != 1:
            continue
        floor = floors[0]
        normal = np.asarray(floor['normal_world'])
        if normal[2] < 0:
            normal = -normal
        height = (np.asarray(observation['centroid_world']) - floor['centroid_world']) @ normal
        heights.append(dict(logfile=observation['logfile'], time=observation['actual_time'],
                            raised_patch=observation['patch'], floor_patch=floor['patch'],
                            centroid_height_above_local_floor_m=float(height)))
    report = dict(definition=definition, depth_time_offset=args.depth_time_offset, pose_settings=pose_settings,
                  inputs=dict(logs_dir=str(args.logs_dir), pose_dir=str(args.pose_dir)),
                  observations=results, comparisons=all_rows, usable_for_driving=False,
                  local_floor_heights=heights,
                  limitations=['Manual semantic patch labels; RGB and depth are not pixel-aligned',
                               'Provisional intrinsics, optical offset and host-time alignment',
                               'Local plane fitting is diagnostic, never applied as a pose correction',
                               'Reference plane contains depth noise; local planarity is not depth accuracy',
                               'Quantized constant-depth plateaus can have zero plane-fit residual',
                               'Different ROIs sample the same assumed surface, not tracked identical points',
                               'One-pixel disparity estimate is a scale illustration, not an error bound',
                               'Planes do not constrain translation parallel to their surface'])
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    render_errors(args.output / 'errors.png', all_rows)
    return report


def render_annotations(path, frames, annotations):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    unique = {t: depth for t, depth in frames.values()}
    fig, axes = plt.subplots(len(annotations), 1, figsize=(9, len(annotations) * 4), squeeze=False,
                             constrained_layout=True)
    for ax, (t, patches) in zip(axes[:, 0], sorted(annotations.items())):
        depth = unique[t]
        ax.imshow(np.ma.masked_where(depth == 0, depth), cmap='turbo_r', vmin=.3, vmax=3)
        for name, (x0, y0, x1, y1) in patches:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor='white', linewidth=2))
            ax.text(x0, y0 - 4, name, color='white', fontsize=9, bbox=dict(facecolor='black', alpha=.6))
        ax.set(title=f'{path.stem}, depth host {t:.6f}s', xlabel='Depth column', ylabel='Depth row')
    fig.savefig(path, dpi=120)
    plt.close(fig)


def render_errors(path, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    groups = sorted({(r['logfile'], r['patch']) for r in rows})
    fig, axes = plt.subplots(len(groups), 1, figsize=(10, len(groups) * 3.2), squeeze=False,
                             constrained_layout=True)
    for ax, (logfile, patch) in zip(axes[:, 0], groups):
        selected = [r for r in rows if r['logfile'] == logfile and r['patch'] == patch]
        for mode in ['fixed_joint', 'articulated']:
            values = [r for r in selected if r['mode'] == mode]
            ax.plot([r['time'] for r in values], [r['reference_p90_abs_m'] * 1000 for r in values],
                    'o-', label=mode + ' reference-plane P90')
        values = [r for r in selected if r['mode'] == 'articulated']
        ax.plot([r['time'] for r in values], [r['local_p90_abs_m'] * 1000 for r in values],
                'k.--', label='Within-patch planarity P90')
        ax.set(title=f'{logfile}: {patch}', xlabel='Native depth host-log seconds', ylabel='mm')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--annotations', type=Path, required=True)
    parser.add_argument('--logs-dir', type=Path, default=Path('/data/matty'))
    parser.add_argument('--pose-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--depth-time-offset', type=float, default=0)
    args = parser.parse_args()
    if not np.isfinite(args.depth_time_offset):
        parser.error('depth-time-offset must be finite')
    report = check_patches(args)
    print(f'{len(report["observations"])} patch observations written to {args.output}')


if __name__ == '__main__':
    main()
