import matplotlib.pyplot as plt
import os
from osm_path import OSMPath

def visualize(osm_file, path_gps=None):
    osm_path = OSMPath(osm_file)
    
    # Plot all road segments
    for u, v in osm_path.graph.edges():
        lon1, lat1 = osm_path.nodes[u]
        lon2, lat2 = osm_path.nodes[v]
        plt.plot([lon1, lon2], [lat1, lat2], 'gray', alpha=0.5, linewidth=1)
    
    # Plot the calculated path
    if path_gps:
        lons, lats = zip(*path_gps)
        plt.plot(lons, lats, 'r', linewidth=2, label='Path')
        plt.scatter([lons[0], lons[-1]], [lats[0], lats[-1]], c='green', zorder=5)
    
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Stromovka Park Road Network')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Save the plot instead of showing it (since we are in a CLI environment)
    output_img = os.path.join(os.path.dirname(__file__), "stromovka_path.png")
    plt.savefig(output_img)
    print(f"Visualization saved to {output_img}")

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    osm_file = os.path.join(current_dir, "stromovka.json")
    
    # Example path
    osm_path = OSMPath(osm_file)
    start = (50.1055, 14.4285)
    end = (50.1085, 14.4150)
    path = osm_path.find_path(start, end)
    
    visualize(osm_file, path)
