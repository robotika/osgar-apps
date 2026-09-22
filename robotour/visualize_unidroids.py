import json
import matplotlib.pyplot as plt
import os
import math
from osm_path import OSMPath

def visualize_combined(osm_file, unidroids_file, distance_csv, output_img):
    # Load real OSM data
    osm_path = OSMPath(osm_file)
    
    plt.figure(figsize=(12, 10))
    
    # Plot real OSM road segments in gray
    for u, v in osm_path.graph.edges():
        lon1, lat1 = osm_path.nodes[u]
        lon2, lat2 = osm_path.nodes[v]
        plt.plot([lon1, lon2], [lat1, lat2], 'gray', alpha=0.3, linewidth=1)
    
    # Load unidroids map
    with open(unidroids_file, 'r') as f:
        unidroids = json.load(f)
    
    # Load distances from CSV
    distances = {}
    import csv
    with open(distance_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            distances[row['id']] = float(row['dist_m'])

    nodes = {n['id']: (n['lon'], n['lat']) for n in unidroids['nodes']}
    
    # Plot unidroids edges in blue
    for edge in unidroids.get('edges', []):
        u, v = edge['from'], edge['to']
        if u in nodes and v in nodes:
            lon1, lat1 = nodes[u]
            lon2, lat2 = nodes[v]
            plt.plot([lon1, lon2], [lat1, lat2], 'b', alpha=0.4, linewidth=1, zorder=2)
            
    # Separate nodes by distance threshold
    green_lons, green_lats = [], []
    red_lons, red_lats = [], []
    
    for node in unidroids['nodes']:
        dist = distances.get(node['id'], 999.0)
        if dist < 1.0:
            green_lons.append(node['lon'])
            green_lats.append(node['lat'])
        else:
            red_lons.append(node['lon'])
            red_lats.append(node['lat'])

    plt.scatter(green_lons, green_lats, c='green', s=15, zorder=5, label='Dist < 1m')
    plt.scatter(red_lons, red_lats, c='red', s=15, zorder=6, label='Dist >= 1m')
    
    # Zoom to unidroids bounding box
    all_lons = [n['lon'] for n in unidroids['nodes']]
    all_lats = [n['lat'] for n in unidroids['nodes']]
    margin = 0.0005
    plt.xlim(min(all_lons) - margin, max(all_lons) + margin)
    plt.ylim(min(all_lats) - margin, max(all_lats) + margin)

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Unidroids Waypoints Overlay: Green < 1m error, Red >= 1m')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save with higher DPI
    plt.savefig(output_img, dpi=200)
    print(f"Visualization saved to {output_img}")

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    osm_file = os.path.join(current_dir, 'unidroids-stromovka.json')
    unidroids_file = os.path.join(current_dir, 'unidroids-map.json')
    distance_csv = os.path.join(current_dir, 'unidroids_distances.csv')
    output_img = os.path.join(current_dir, 'unidroids_visualization.png')
    
    visualize_combined(osm_file, unidroids_file, distance_csv, output_img)
