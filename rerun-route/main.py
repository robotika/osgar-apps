"""
  Rerun Route from OSGAR log
"""
import math
from osgar.node import Node
from osgar.followpath import FollowPath, Route
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


class RerunRoute(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.logfile = config.get('logfile')
        self.pose2d_stream = config.get('pose2d_stream', 'platform.pose2d')
        
        # Load path from log file
        self.path = self.extract_path(self.logfile, self.pose2d_stream)
        if not self.path:
            # We can't raise exception here as it might crash the whole system
            # but we can print and keep empty path
            print(f"ERROR: No path extracted from {self.logfile}")
        else:
            print(f"Extracted {len(self.path)} points from {self.logfile}")

        self.app = FollowPath(config, bus)
        self.app.route = Route(pts=self.path)
        
        # Override app methods to use this node's bus
        self.app.publish = self.my_publish
        self.app.listen = self.my_listen
        self.app.update = self.my_update

    def my_publish(self, name, data):
        self.publish(name, data)

    def my_listen(self):
        return self.listen()

    def my_update(self):
        return self.update()

    def extract_path(self, logfile, pose2d_stream):
        if logfile is None:
            return []
        path = []
        try:
            stream_id = lookup_stream_id(logfile, pose2d_stream)
        except Exception as e:
            print(f"Error looking up stream {pose2d_stream} in {logfile}: {e}")
            return []

        with LogReader(logfile, only_stream_id=stream_id) as log:
            for timestamp, stream_id, data in log:
                pose = deserialize(data)
                # pose is typically [x, y, heading] in mm and hundredths of degree
                x, y = pose[0]/1000.0, pose[1]/1000.0
                if len(path) == 0 or math.hypot(path[-1][0] - x, path[-1][1] - y) > 0.1:
                    path.append((x, y))
        return path

    def on_pose2d(self, data):
        self.app.on_pose2d(data)

    def on_emergency_stop(self, data):
        self.app.on_emergency_stop(data)

    def on_obstacle(self, data):
        if hasattr(self.app, 'on_obstacle'):
            self.app.on_obstacle(data)

    def run(self):
        self.app.run()

# vim: expandtab sw=4 ts=4
