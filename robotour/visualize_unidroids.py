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
            
    # Separate nodes by multi-level distance thresholds
    categories = [
        {'label': '< 1.0m', 'color': 'green', 'lons': [], 'lats': [], 'threshold': 1.0},
        {'label': '1.0m - 1.5m', 'color': 'blue', 'lons': [], 'lats': [], 'threshold': 1.5},
        {'label': '1.5m - 2.0m', 'color': 'pink', 'lons': [], 'lats': [], 'threshold': 2.0},
        {'label': '2.0m - 2.5m', 'color': 'magenta', 'lons': [], 'lats': [], 'threshold': 2.5},
        {'label': '> 2.5m', 'color': 'red', 'lons': [], 'lats': [], 'threshold': float('inf')}
    ]
    
    for node in unidroids['nodes']:
        dist = distances.get(node['id'], 999.0)
        for cat in categories:
            if dist <= cat['threshold']:
                cat['lons'].append(node['lon'])
                cat['lats'].append(node['lat'])
                break

    for cat in categories:
        if cat['lons']:
            plt.scatter(cat['lons'], cat['lats'], c=cat['color'], s=15, zorder=6, label=cat['label'])
    
    # Zoom to unidroids bounding box
    all_lons = [n['lon'] for n in unidroids['nodes']]
    all_lats = [n['lat'] for n in unidroids['nodes']]
    margin = 0.0005
    plt.xlim(min(all_lons) - margin, max(all_lons) + margin)
    plt.ylim(min(all_lats) - margin, max(all_lats) + margin)

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Unidroids Waypoints: Distance to OSM Edges')
    plt.legend(title='Distance Thresholds')
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
