"""
  Convert logfile into "mapfile" = array of 360deg scans from VanJee
"""
import math

import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

def create_map(logfile, stream_lidar, stream_odom, outfile,
               start_time_sec=0, end_time_sec=None, step_dist=1.0):
    lidar_stream_id = lookup_stream_id(logfile, stream_lidar)
    odom_stream_id = lookup_stream_id(logfile, stream_odom)
    scans = []
    poses = []
    last_pose = None
    save_scan = False
    with LogReader(logfile, only_stream_id=[lidar_stream_id, odom_stream_id]) as log:
        for timestamp, stream_id, data in log:
            if timestamp.total_seconds() < start_time_sec:
                continue
            if end_time_sec is not None and timestamp.total_seconds() > end_time_sec:
                break
            buf = deserialize(data)
            if stream_id == odom_stream_id:
                pose = buf[0]/1000.0, buf[1]/1000.0, math.radians(buf[2]/100.0)
                if last_pose is None or math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]) > step_dist:
                    save_scan = True
                    last_pose = pose
                    poses.append(buf)
            elif stream_id == lidar_stream_id:
                if save_scan:
                    save_scan = False
                    scans.append(buf)
            else:
                assert 0, f'Not supported stream {stream_id}'
    assert len(scans) == len(poses), (len(poses), len(scans))
    np.savez_compressed(outfile, poses=poses, scans=scans)
    return poses, scans


def get_xy_for_scan(pose, scan):
    x0, y0, a0 = pose
    x = [x0 / 1000.0 + math.cos(math.radians(a0 / 100 + 180 - 360 * i / 1800)) * dist / 1000.0 for i, dist in
         enumerate(scan)]
    y = [y0 / 1000.0 + math.sin(math.radians(a0 / 100 + 180 - 360 * i / 1800)) * dist / 1000.0 for i, dist in
         enumerate(scan)]
    return x, y

def draw(poses, scans):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    fig, ax = plt.subplots()

    scan_i = 0

    x = [p[0]/1000.0 for p in poses]
    y = [p[1]/1000.0 for p in poses]
    ax.plot(x, y, '-o', color='orange')

    x, y = get_xy_for_scan(poses[0], scans[0])
    scan_xy_prev, = ax.plot(x, y, 'o', color='gray')
    x, y = get_xy_for_scan(poses[1], scans[1])
    scan_xy, = ax.plot(x, y, 'o', color='blue')

    current, = ax.plot([poses[0][0]/1000.0], [poses[0][1]/1000.0], 'o', color='red')
    ax.axis('equal')

    # adjust the main plot to make room for the sliders
    fig.subplots_adjust(bottom=0.25)

    # recttuple (left, bottom, width, height)
    # The dimensions (left, bottom, width, height) of the new Axes. All quantities are in fractions of figure width and height.
    axfreq = fig.add_axes([0.1, 0.1, 0.8, 0.03])
    freq_slider = Slider(
        ax=axfreq,
        label='Frame',
        valmin=0,
        valmax=len(scans) - 1,
        valinit=scan_i,
        valstep=1,
    )

    def update(val):
        scan_i = int(freq_slider.val)
        if scan_i > 0:
            x, y = get_xy_for_scan(poses[scan_i - 1], scans[scan_i - 1])
            scan_xy_prev.set_xdata(x)
            scan_xy_prev.set_ydata(y)
        x, y = get_xy_for_scan(poses[scan_i], scans[scan_i])
        scan_xy.set_xdata(x)
        scan_xy.set_ydata(y)
        current.set_xdata([poses[scan_i][0]/1000.0])
        current.set_ydata([poses[scan_i][1]/1000.0])

    freq_slider.on_changed(update)
    plt.show()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Convert logfile to scans map')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='lidar scan stream name', default='vanjee.scan')
    parser.add_argument('--odom', help='odometry stream name', default='platform.pose2d')
    parser.add_argument('--out', '-o', help='output map file', default='scan-map.npz')
    parser.add_argument('--start-time-sec', '-s', help='start time (sec)',
                        type=float, default=0.0)
    parser.add_argument('--end-time-sec', '-e', help='stop mapping at (sec)',
                        type=float, default=None)
    parser.add_argument('--step', help='distance in meters', type=float, default=1.0)
    parser.add_argument('--draw', help='draw scans map', action='store_true')
    args = parser.parse_args()

    poses, scans = create_map(args.logfile, args.stream, args.odom, args.out,
                              start_time_sec=args.start_time_sec, end_time_sec=args.end_time_sec,
                              step_dist=args.step)
    num_scans = len(scans)
    print(f'Num scans = {num_scans}')
    if args.draw:
        draw(poses, scans)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4

