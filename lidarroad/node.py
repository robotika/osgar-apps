from osgar.node import Node


class LidarRoad(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
#        bus.register('desired_steering')
        bus.register('desired_speed')

    def on_emergency_stop(self, data):
        pass

    def on_pose2d(self, data):
        pass

    def on_scan(self, data):
        pass
