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
    return len(scans)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Convert logfile to AVI video')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='lidar scan stream name', default='vanjee.scan')
    parser.add_argument('--odom', help='odometry stream name', default='platform.pose2d')
    parser.add_argument('--out', '-o', help='output map file', default='scan-map.npz')
    parser.add_argument('--start-time-sec', '-s', help='start time (sec)',
                        type=float, default=0.0)
    parser.add_argument('--end-time-sec', '-e', help='stop mapping at (sec)',
                        type=float, default=None)
    parser.add_argument('--step', help='distance in meters', type=float, default=1.0)
    args = parser.parse_args()

    num_scans = create_map(args.logfile, args.stream, args.odom, args.out,
               start_time_sec=args.start_time_sec, end_time_sec=args.end_time_sec,
               step_dist=args.step)
    print(f'Num scans = {num_scans}')


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4

