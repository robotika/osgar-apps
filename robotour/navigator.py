"""
  Robotour project - Navigator Node
  Trigger pathfinding and provide guidance to junctions.
"""
import math
import os

from osgar.node import Node
from robotour.osm_path import OSMPath

def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    # pos is (lon, lat) in milliseconds
    x_scale = math.cos(math.radians(pos1[1]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale

def geo_angle(pos1, pos2):
    if geo_length(pos1, pos2) < 0.1:
        return None
    x_scale = math.cos(math.radians(pos1[1]/3600000))
    return math.atan2(pos2[1] - pos1[1], (pos2[0] - pos1[0])*x_scale)

class Navigator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('navigator_info', 'debug_path')
        self.osm_file = config.get('osm_file', 'stromovka.json')
        if not os.path.isabs(self.osm_file):
            self.osm_file = os.path.join(os.path.dirname(__file__), self.osm_file)
        
        self.osm_path = OSMPath(self.osm_file)
        self.destination = config.get('destination')  # (lat, lon)
        self.path_nodes = None
        self.next_node_index = None

    def on_gps(self, data):
        # data is [lat, lon] in 1/10^7 degrees
        lat, lon = data[0] / 10.0, data[1] / 10.0  # Convert to milliseconds
        curr_pos = (lon, lat)
        
        if self.path_nodes is None and self.destination:
            # Trigger initial pathfinding
            start_gps = (lat / 3600000.0, lon / 3600000.0)
            dest_gps = (self.destination[0], self.destination[1])
            self.path_nodes = self.osm_path.find_path(start_gps, dest_gps)
            if self.path_nodes:
                # Debug publish path as GPS waypoints
                debug_gps_path = [self.osm_path.get_node_gps(node_id) for node_id in self.path_nodes]
                self.publish('debug_path', debug_gps_path)
                self.next_node_index = 1 # Start with first segment
        
        if self.path_nodes:
            self.update_navigation(curr_pos)

    def update_navigation(self, curr_pos):
        if self.next_node_index >= len(self.path_nodes):
            return

        # Current target node
        target_node_id = self.path_nodes[self.next_node_index]
        target_lon, target_lat = self.osm_path.get_node_gps(target_node_id)
        target_pos = (int(round(target_lon*3600000)), int(round(target_lat*3600000)))
        
        dist = geo_length(curr_pos, target_pos)
        azimuth = geo_angle(curr_pos, target_pos)
        
        # Check if we reached the target node
        if dist < 2.0:  # 2 meters threshold
            self.next_node_index += 1
            if self.next_node_index < len(self.path_nodes):
                # Report junction info if the new target node or current node is a junction
                self.report_junction(target_node_id, self.path_nodes[self.next_node_index])
        
        if azimuth is not None:
            self.publish('navigator_info', {
                'dist': dist,
                'azimuth': math.degrees(azimuth),
            })

    def report_junction(self, node_id, next_node_id):
        neighbors = self.osm_path.get_neighbors(node_id)
        if len(neighbors) > 2:
            # This is a junction
            curr_node_gps = self.osm_path.get_node_gps(node_id)
            next_node_gps = self.osm_path.get_node_gps(next_node_id)
            
            curr_pos = (int(round(curr_node_gps[0]*3600000)), int(round(curr_node_gps[1]*3600000)))
            next_pos = (int(round(next_node_gps[0]*3600000)), int(round(next_node_gps[1]*3600000)))
            
            exit_azimuth = geo_angle(curr_pos, next_pos)
            
            ignored_roads = []
            for neighbor_id in neighbors:
                if neighbor_id == next_node_id:
                    continue
                # For simplicity, prev node in path could also be ignored, but let's list all others
                neighbor_gps = self.osm_path.get_node_gps(neighbor_id)
                neighbor_pos = (int(round(neighbor_gps[0]*3600000)), int(round(neighbor_gps[1]*3600000)))
                neighbor_azimuth = geo_angle(curr_pos, neighbor_pos)
                if neighbor_azimuth is not None:
                    ignored_roads.append(math.degrees(neighbor_azimuth))

            self.publish('navigator_info', {
                'junction': True,
                'exit_azimuth': math.degrees(exit_azimuth) if exit_azimuth is not None else None,
                'ignored_roads': ignored_roads
            })

if __name__ == "__main__":
    pass
