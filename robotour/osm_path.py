import json
import os
import math
import networkx as nx

def geo_length(pos1, pos2):
    "return distance on sphere for two integer positions in milliseconds"
    # pos is (lon, lat) in milliseconds
    x_scale = math.cos(math.radians(pos1[1]/3600000))
    scale = 40000000/(360*3600000)
    return math.hypot((pos2[0] - pos1[0])*x_scale, pos2[1] - pos1[1]) * scale

def latlon2ms(lat, lon):
    return int(round(lon*3600000)), int(round(lat*3600000))

class OSMPath:
    def __init__(self, osm_file):
        with open(osm_file, 'r') as f:
            self.data = json.load(f)
        
        self.nodes = {}
        self.graph = nx.Graph()
        self._build_graph()

    def _build_graph(self):
        # Extract nodes
        for element in self.data['elements']:
            if element['type'] == 'node':
                self.nodes[element['id']] = (element['lon'], element['lat'])
        
        # Extract ways and build edges
        for element in self.data['elements']:
            if element['type'] == 'way':
                if 'highway' in element.get('tags', {}):
                    nodes = element['nodes']
                    for i in range(len(nodes) - 1):
                        u, v = nodes[i], nodes[i+1]
                        if u in self.nodes and v in self.nodes:
                            pos1 = latlon2ms(self.nodes[u][1], self.nodes[u][0])
                            pos2 = latlon2ms(self.nodes[v][1], self.nodes[v][0])
                            dist = geo_length(pos1, pos2)
                            self.graph.add_edge(u, v, weight=dist)

    def find_nearest_node(self, lat, lon):
        min_dist = float('inf')
        nearest_node = None
        pos = latlon2ms(lat, lon)
        for node_id, (node_lon, node_lat) in self.nodes.items():
            if node_id not in self.graph:
                continue
            node_pos = latlon2ms(node_lat, node_lon)
            dist = geo_length(pos, node_pos)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id
        return nearest_node

    def find_path(self, start_gps, end_gps):
        start_node = self.find_nearest_node(*start_gps)
        end_node = self.find_nearest_node(*end_gps)
        
        if not start_node or not end_node:
            return None
        
        try:
            path_nodes = nx.shortest_path(self.graph, source=start_node, target=end_node, weight='weight')
            return [self.nodes[node_id] for node_id in path_nodes]
        except nx.NetworkXNoPath:
            return None

if __name__ == "__main__":
    osm_file = os.path.join(os.path.dirname(__file__), "stromovka.json")
    osm_path = OSMPath(osm_file)
    
    # Example: From near Planetarium to near Crossroad
    start = (50.1055, 14.4285)
    end = (50.1085, 14.4150)
    
    path = osm_path.find_path(start, end)
    if path:
        print(f"Path found with {len(path)} waypoints")
        for lon, lat in path:
            print(f"{lat}, {lon}")
    else:
        print("No path found")
