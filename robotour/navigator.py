"""
  Robotour project - Navigator Node
  Trigger pathfinding and provide guidance to junctions.
"""
import math
import os

from osgar.node import Node
from robotour.osm_path import OSMPath, geo_length, geo_angle

class Navigator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('navigator_info', 'debug_path')
        self.osm_file = config.get('osm_file', 'stromovka.json')
        if not os.path.isabs(self.osm_file):
            self.osm_file = os.path.join(os.path.dirname(__file__), self.osm_file)
        
        self.osm_path = OSMPath(self.osm_file)
        self.destination = config.get('destination')  # (lat, lon) in float degrees
        self.path_nodes = None
        self.next_node_index = None

    def on_gps(self, data):
        # OSGAR GPS data is [lat, lon] in 1/10^7 degrees
        lat, lon = data[0] / 10000000.0, data[1] / 10000000.0
        curr_pos = (lon, lat)  # (lon, lat) in float degrees
        
        if self.path_nodes is None and self.destination:
            # Trigger initial pathfinding
            start_gps = (lat, lon)
            dest_gps = (self.destination[0], self.destination[1])
            self.path_nodes = self.osm_path.find_path(start_gps, dest_gps)
            if self.path_nodes:
                # Debug publish path as GPS waypoints (lat, lon)
                debug_gps_path = []
                for node_id in self.path_nodes:
                    node_lon, node_lat = self.osm_path.get_node_gps(node_id)
                    debug_gps_path.append([node_lat, node_lon])
                self.publish('debug_path', debug_gps_path)
                self.next_node_index = 1 # Start with first segment
        
        if self.path_nodes:
            self.update_navigation(curr_pos)

    def update_navigation(self, curr_pos):
        if self.next_node_index >= len(self.path_nodes):
            return

        # Current target node
        target_node_id = self.path_nodes[self.next_node_index]
        target_pos = self.osm_path.get_node_gps(target_node_id) # (lon, lat)
        
        dist = geo_length(curr_pos, target_pos)
        azimuth = geo_angle(curr_pos, target_pos)
        
        # Check if we reached the target node
        if dist < 2.0:  # 2 meters threshold
            self.next_node_index += 1
            if self.next_node_index < len(self.path_nodes):
                # Report junction info if the new target node is a junction
                self.report_junction(target_node_id, self.path_nodes[self.next_node_index])
                
                # Recalculate guidance for the new target node
                target_node_id = self.path_nodes[self.next_node_index]
                target_pos = self.osm_path.get_node_gps(target_node_id)
                dist = geo_length(curr_pos, target_pos)
                azimuth = geo_angle(curr_pos, target_pos)
            else:
                # We reached the final destination
                dist = 0.0
                azimuth = None
        
        if azimuth is not None:
            self.publish('navigator_info', {
                'dist': dist,
                'azimuth': math.degrees(azimuth),
            })

    def report_junction(self, node_id, next_node_id):
        neighbors = self.osm_path.get_neighbors(node_id)
        if len(neighbors) > 2:
            # This is a junction
            curr_pos = self.osm_path.get_node_gps(node_id) # (lon, lat)
            next_pos = self.osm_path.get_node_gps(next_node_id) # (lon, lat)
            
            exit_azimuth = geo_angle(curr_pos, next_pos)
            
            ignored_roads = []
            for neighbor_id in neighbors:
                if neighbor_id == next_node_id:
                    continue
                neighbor_pos = self.osm_path.get_node_gps(neighbor_id)
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
