import json
import math
import csv
import os
from osm_path import OSMPath, geo_length

def point_to_segment_dist(p, s1, s2):
    """
    Calculate the distance from point p to line segment (s1, s2).
    p, s1, s2 are (lon, lat) tuples.
    Returns distance in meters.
    """
    # Rough approximation: convert to local meters relative to s1
    x_scale = math.cos(math.radians(s1[1]))
    scale = 40000000.0 / 360.0
    
    def to_meters(pt):
        return ((pt[0] - s1[0]) * x_scale * scale, (pt[1] - s1[1]) * scale)
    
    # p_m, s2_m are local coordinates in meters, s1_m is (0,0)
    p_m = to_meters(p)
    s2_m = to_meters(s2)
    
    # Vector S2 - S1 (in meters)
    dx, dy = s2_m
    l2 = dx*dx + dy*dy
    if l2 == 0:
        return math.hypot(p_m[0], p_m[1])
    
    # Projection factor t
    t = max(0, min(1, (p_m[0] * dx + p_m[1] * dy) / l2))
    
    # Closest point on segment
    closest_x = t * dx
    closest_y = t * dy
    
    return math.hypot(p_m[0] - closest_x, p_m[1] - closest_y)

def calculate_distances(osm_file, unidroids_file, output_csv):
    osm_path = OSMPath(osm_file)
    
    with open(unidroids_file, 'r') as f:
        unidroids = json.load(f)
    
    edges = []
    for u, v in osm_path.graph.edges():
        edges.append((osm_path.nodes[u], osm_path.nodes[v]))
    
    results = []
    for node in unidroids['nodes']:
        p = (node['lon'], node['lat'])
        min_dist = float('inf')
        
        # This is O(N_waypoints * N_osm_edges), which is acceptable for this scale
        for s1, s2 in edges:
            d = point_to_segment_dist(p, s1, s2)
            if d < min_dist:
                min_dist = d
        
        results.append({
            'id': node['id'],
            'label': node.get('label', ''),
            'lat': node['lat'],
            'lon': node['lon'],
            'dist_m': min_dist
        })
    
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['id', 'label', 'lat', 'lon', 'dist_m'])
        writer.writeheader()
        writer.writerows(results)
    
    print(f"Distances saved to {output_csv}")

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    osm_file = os.path.join(current_dir, 'unidroids-stromovka.json')
    unidroids_file = os.path.join(current_dir, 'unidroids-map.json')
    output_csv = os.path.join(current_dir, 'unidroids_distances.csv')
    
    calculate_distances(osm_file, unidroids_file, output_csv)
