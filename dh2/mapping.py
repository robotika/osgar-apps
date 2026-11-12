"""Offline depth/odometry surface map. This is not SLAM or a free-space map."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from osgar.lib.serialize import deserialize
from osgar.logger import LogReader, lookup_stream_names


def interpolate(timestamp, times, values, max_gap):
    """Interpolate already-unwrapped angles; reject gaps and extrapolation."""
    index = int(np.searchsorted(times, timestamp))
    if index < len(times) and times[index] == timestamp:
        return values[index]
    if index == 0 or index == len(times) or times[index] - times[index - 1] > max_gap:
        return None
    ratio = (timestamp - times[index - 1]) / (times[index] - times[index - 1])
    return values[index - 1] * (1 - ratio) + values[index] * ratio


def project_depth(depth, intrinsics, pose, joint, *, height, pitch, forward, left, joint_scale,
                  stride=4, min_range=0.15, max_range=4.0):
    """Depth optical Z (mm) -> local map XYZ (m), X forward, Y left, Z up.

    Positive pitch looks downward. Extrinsics and joint convention must be verified
    on the robot; no IMU compensation, distortion correction or loop closure here.
    """
    fx, fy, cx, cy = intrinsics
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(float) / 1000
    valid = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    z, u, v = z[valid], u[valid], v[valid]
    right = (u - cx) * z / fx
    down = (v - cy) * z / fy
    camera_x = z * math.cos(pitch) - down * math.sin(pitch) + forward
    camera_y = -right + left
    camera_z = height - z * math.sin(pitch) - down * math.cos(pitch)
    yaw = pose[2] + joint_scale * joint
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((pose[0] + c * camera_x - s * camera_y,
                            pose[1] + s * camera_x + c * camera_y, camera_z))


class SurfaceGrid:
    """Sparse grid of observed endpoints only; no ray clearing or occupancy claims."""

    def __init__(self, resolution):
        self.resolution = resolution
        self.cells = {}

    def add(self, points):
        if not len(points):
            return
        indices = np.floor(points[:, :2] / self.resolution).astype(np.int64)
        cells, inverse = np.unique(indices, axis=0, return_inverse=True)
        counts = np.bincount(inverse)
        sums = np.bincount(inverse, weights=points[:, 2])
        minima = np.full(len(cells), np.inf)
        maxima = np.full(len(cells), -np.inf)
        np.minimum.at(minima, inverse, points[:, 2])
        np.maximum.at(maxima, inverse, points[:, 2])
        for index, cell in enumerate(cells):
            key = tuple(cell)
            old = self.cells.get(key)
            row = [int(counts[index]), float(sums[index]), float(minima[index]), float(maxima[index]), 1]
            if old is None:
                self.cells[key] = row
            else:
                old[0] += row[0]
                old[1] += row[1]
                old[2] = min(old[2], row[2])
                old[3] = max(old[3], row[3])
                old[4] += 1

    def arrays(self):
        keys = sorted(self.cells)
        cells = np.array(keys, dtype=np.int64).reshape(-1, 2)
        stats = np.array([self.cells[key] for key in keys], dtype=float).reshape(-1, 5)
        return dict(cell_indices=cells, centers=(cells + 0.5) * self.resolution,
                    points=stats[:, 0].astype(np.int64), mean_height=stats[:, 1] / stats[:, 0],
                    min_height=stats[:, 2], max_height=stats[:, 3], frames=stats[:, 4].astype(np.int64))


def read_motion(logfile, names):
    streams = {names.index(name) + 1: name for name in ('platform.pose2d', 'platform.joint_angle')}
    times, poses, joint_times, joints, reports = [], [], [], [], []
    if 'app.defender' in names:
        streams[names.index('app.defender') + 1] = 'app.defender'
    with LogReader(str(logfile), only_stream_id=streams) as log:
        for timestamp, stream, raw in log:
            data = deserialize(raw)
            sec = timestamp.total_seconds()
            if streams[stream] == 'platform.pose2d':
                times.append(sec)
                poses.append([data[0] / 1000, data[1] / 1000, math.radians(data[2] / 100)])
            elif streams[stream] == 'platform.joint_angle':
                joint_times.append(sec)
                joints.append(math.radians(data[0] / 100))
            else:
                reports.append(dict(time=sec, report=data))
    if not poses or not joints:
        raise ValueError('Mapping requires recorded pose2d and measured joint_angle samples')
    poses = np.asarray(poses)
    poses[:, 2] = np.unwrap(poses[:, 2])
    return np.asarray(times), poses, np.asarray(joint_times), np.unwrap(joints), reports


def build_map(args):
    names = lookup_stream_names(str(args.logfile))
    times, poses, joint_times, joints, reports = read_motion(args.logfile, names)
    grid = SurfaceGrid(args.resolution)
    used_times, used_poses = [], []
    counters = dict(depth_frames=0, mapped_frames=0, skipped_sync=0, sampled_points=0)
    intrinsics = None
    shape = None
    last_used = -math.inf
    stream = names.index('oak.depth') + 1
    with LogReader(str(args.logfile), only_stream_id=stream) as log:
        for timestamp, _, raw in log:
            counters['depth_frames'] += 1
            sec = timestamp.total_seconds() + args.depth_time_offset
            if sec < args.start or (args.end is not None and sec > args.end):
                continue
            if sec - last_used < args.frame_step:
                continue
            pose = interpolate(sec, times, poses, args.max_pose_gap)
            joint = interpolate(sec, joint_times, joints, args.max_pose_gap)
            if pose is None or joint is None:
                counters['skipped_sync'] += 1
                continue
            depth = np.asarray(deserialize(raw))
            if depth.ndim != 2:
                raise ValueError(f'Expected a depth image, got {depth.shape}')
            if shape is None:
                shape = depth.shape
                h, w = shape
                if args.intrinsics:
                    intrinsics = args.intrinsics
                else:
                    focal = w / (2 * math.tan(math.radians(args.hfov_deg) / 2))
                    intrinsics = (focal, focal, (w - 1) / 2, (h - 1) / 2)
            elif depth.shape != shape:
                raise ValueError('Depth resolution changed within recording')
            points = project_depth(depth, intrinsics, pose, float(joint), height=args.camera_height,
                                   pitch=math.radians(args.pitch_deg), forward=args.camera_forward,
                                   left=args.camera_left, joint_scale=args.joint_yaw_scale,
                                   stride=args.pixel_stride, min_range=args.min_range, max_range=args.max_range)
            points = points[(points[:, 2] >= args.min_height) & (points[:, 2] <= args.max_height)]
            grid.add(points)
            counters['sampled_points'] += len(points)
            counters['mapped_frames'] += 1
            used_times.append(sec)
            used_poses.append(pose)
            last_used = sec
    if not grid.cells:
        raise ValueError('No mapped points: check depth presence, time interval, alignment and range limits')
    arrays = grid.arrays()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / 'surfaces.npz', **arrays, resolution=args.resolution,
                        pose_times=np.asarray(used_times), poses=np.asarray(used_poses))
    settings = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    metadata = dict(settings=settings, intrinsics=list(intrinsics), depth_shape=list(shape),
                    counters=counters, cells=len(grid.cells), reports=reports,
                    limitations=['Observed surfaces, not free space or verified traversability',
                                 'No SLAM, loop closure, IMU compensation, or cross-run alignment',
                                 'Camera extrinsics and pose/joint convention need validation',
                                 'Pose synchronization uses host log times plus configured offset',
                                 'Depth zeros are omitted; they do not clear map cells'])
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    render_map(output / 'map.png', arrays, np.asarray(used_poses), args, reports)
    return metadata


def render_map(path, arrays, poses, args, reports):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm

    mask = arrays['frames'] >= args.min_frames
    if not mask.any():
        raise ValueError('No cells meet --min-frames; use more data or lower the threshold')
    centers = arrays['centers'][mask]
    mean = arrays['mean_height'][mask]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    for ax, values, label, cmap in zip(axes, [mean, arrays['frames'][mask]],
                                       ['Mean observed height (m)', 'Depth-frame observations'],
                                       ['coolwarm', 'viridis']):
        plot = ax.scatter(centers[:, 0], centers[:, 1], c=values, s=8, marker='s', cmap=cmap,
                          **(dict(norm=TwoSlopeNorm(vmin=-0.5, vcenter=0, vmax=1.0)) if ax is axes[0] else {}))
        fig.colorbar(plot, ax=ax, label=label)
        ax.plot(poses[:, 0], poses[:, 1], color='black', linewidth=1, label='Odometry path')
        ax.scatter(*poses[0, :2], marker='>', c='lime', edgecolors='black', label='Start')
        for item in reports:
            x, y = item['report']['position']
            ax.plot(x, y, 'kx')
            ax.annotate(f"report {item['report']['id']}", (x, y))
        ax.set(xlabel='Local X (m)', ylabel='Local Y (m)', aspect='equal')
        ax.legend(loc='best')
    geometry = 'HFOV approximation' if args.hfov_deg is not None else 'supplied intrinsics'
    fig.suptitle(f'{args.logfile.name}\nSurface-map preview — {geometry}; unobserved space blank')
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    geometry = parser.add_mutually_exclusive_group(required=True)
    geometry.add_argument('--intrinsics', type=float, nargs=4, metavar=('FX', 'FY', 'CX', 'CY'),
                          help='Rectified depth intrinsics in pixels at the recorded depth resolution')
    geometry.add_argument('--hfov-deg', type=float, help='Explicit approximate horizontal depth FOV')
    parser.add_argument('--camera-height', type=float, default=0.2)
    parser.add_argument('--pitch-deg', type=float, default=0, help='Positive downward')
    parser.add_argument('--camera-forward', type=float, default=0, help='Approximate lever arm from pose origin (m)')
    parser.add_argument('--camera-left', type=float, default=0, help='Depth-camera lateral offset (m)')
    parser.add_argument('--joint-yaw-scale', type=float, default=0.5, help='Assumed front yaw relative to pose heading')
    parser.add_argument('--depth-time-offset', type=float, default=0)
    parser.add_argument('--max-pose-gap', type=float, default=0.3)
    parser.add_argument('--resolution', type=float, default=0.1)
    parser.add_argument('--pixel-stride', type=int, default=4)
    parser.add_argument('--frame-step', type=float, default=0.5)
    parser.add_argument('--min-range', type=float, default=0.15)
    parser.add_argument('--max-range', type=float, default=4)
    parser.add_argument('--min-height', type=float, default=-1)
    parser.add_argument('--max-height', type=float, default=1.5)
    parser.add_argument('--min-frames', type=int, default=3)
    parser.add_argument('--start', type=float, default=0)
    parser.add_argument('--end', type=float)
    args = parser.parse_args()
    if (args.resolution <= 0 or args.pixel_stride < 1 or args.frame_step < 0 or args.min_frames < 1
            or args.max_pose_gap <= 0 or not 0 <= args.min_range < args.max_range
            or args.min_height >= args.max_height):
        parser.error('Invalid sampling, range or grid settings')
    if args.hfov_deg is not None and not 0 < args.hfov_deg < 180:
        parser.error('HFOV must lie between 0 and 180 degrees')
    if args.intrinsics and (args.intrinsics[0] <= 0 or args.intrinsics[1] <= 0):
        parser.error('Focal lengths must be positive')
    result = build_map(args)
    print(json.dumps(dict(output=str(args.output), **result['counters'], cells=result['cells']), indent=2))


if __name__ == '__main__':
    main()
