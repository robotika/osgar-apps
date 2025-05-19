#!/usr/bin/python
"""
  Log info (time and traveled distance)
"""
import math
import pathlib

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def get_time_and_dist(logfile, pose2d_stream):
    """
    Get log time (duration) and distance traveled
    """
    timestamp = None
    dist = 0.0
    prev = [0, 0, 0]
    with LogReader(logfile, only_stream_id=lookup_stream_id(logfile, pose2d_stream)) as log:
        for timestamp, stream_id, data in log:
            pose2d = deserialize(data)
            dist += math.hypot((pose2d[0] - prev[0])/1000.0, (pose2d[1] - prev[1])/1000.0)
            prev = pose2d
    return timestamp, dist


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', nargs='+', help='recorded log file')
    parser.add_argument('--pose2d', help='pose2d stream', default='platform.pose2d')
    args = parser.parse_args()

    for logfile in args.logfile:
        duration, dist = get_time_and_dist(logfile, pose2d_stream=args.pose2d)
        sec = int(duration.total_seconds())
        name = pathlib.Path(logfile).name
        print(f'{name} - {(sec // 60):02d}:{(sec % 60):02d} - {dist:.1f}m')


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 
