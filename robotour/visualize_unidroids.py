import json
import matplotlib.pyplot as plt
import os
import math
from osm_path import OSMPath

def visualize_combined(osm_file, unidroids_file, output_img):
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
    
    nodes = {n['id']: (n['lon'], n['lat']) for n in unidroids['nodes']}
    
    # Plot unidroids edges in blue
    for edge in unidroids.get('edges', []):
        u, v = edge['from'], edge['to']
        if u in nodes and v in nodes:
            lon1, lat1 = nodes[u]
            lon2, lat2 = nodes[v]
            plt.plot([lon1, lon2], [lat1, lat2], 'b', alpha=0.7, linewidth=2)
            
    # Plot unidroids nodes as small dots
    lons = [n['lon'] for n in unidroids['nodes']]
    lats = [n['lat'] for n in unidroids['nodes']]
    plt.scatter(lons, lats, c='red', s=10, zorder=5, label='Unidroids Waypoints')
    
    # Zoom to unidroids bounding box
    margin = 0.0005
    plt.xlim(min(lons) - margin, max(lons) + margin)
    plt.ylim(min(lats) - margin, max(lats) + margin)

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Unidroids Waypoints Overlay on OSM (Zoomed)')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save with higher DPI (default is usually 100, doubling to 200)
    plt.savefig(output_img, dpi=200)
    print(f"Visualization saved to {output_img}")

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    osm_file = os.path.join(current_dir, 'unidroids-stromovka.json')
    unidroids_file = os.path.join(current_dir, 'unidroids-map.json')
    output_img = os.path.join(current_dir, 'unidroids_visualization.png')
    
    visualize_combined(osm_file, unidroids_file, output_img)
