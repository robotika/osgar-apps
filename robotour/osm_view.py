"""
  Robotour project - OSM Visualization
  Plot the road network and calculated paths using matplotlib.
"""
import argparse
import os

def visualize(osm_file, path_nodes=None, show=False):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from osm_path import OSMPath

    osm_path = OSMPath(osm_file)
    
    # Plot all road segments
    for u, v in osm_path.graph.edges():
        lon1, lat1 = osm_path.nodes[u]
        lon2, lat2 = osm_path.nodes[v]
        plt.plot([lon1, lon2], [lat1, lat2], 'gray', alpha=0.5, linewidth=1)
    
    # Plot the calculated path
    if path_nodes:
        path_gps = [osm_path.get_node_gps(node_id) for node_id in path_nodes]
        lons, lats = zip(*path_gps)
        plt.plot(lons, lats, 'r', linewidth=2, label='Path')
        plt.scatter([lons[0], lons[-1]], [lats[0], lats[-1]], c='green', zorder=5)
    
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title(f'Road Network: {os.path.basename(osm_file)}')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    
    if show:
        try:
            plt.show()
        except Exception as e:
            print(f"Warning: Could not show interactive plot: {e}")
            output_img = osm_file.replace('.json', '.png')
            plt.savefig(output_img)
            print(f"Fallback: Visualization saved to {output_img}")
    else:
        output_img = osm_file.replace('.json', '.png')
        plt.savefig(output_img)
        print(f"Visualization saved to {output_img}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize OSM data and path.')
    parser.add_argument('--input', default='stromovka.json', help='Input OSM JSON file')
    parser.add_argument('--start', type=float, nargs=2, help='Start GPS: lat lon')
    parser.add_argument('--end', type=float, nargs=2, help='End GPS: lat lon')
    parser.add_argument('--show', action='store_true', help='Show interactive plot')
    args = parser.parse_args()

    current_dir = os.path.dirname(__file__)
    osm_file = os.path.join(current_dir, args.input)
    
    path_nodes = None
    if args.start and args.end:
        from osm_path import OSMPath
        osm_path = OSMPath(osm_file)
        path_nodes = osm_path.find_path(tuple(args.start), tuple(args.end))
    
    visualize(osm_file, path_nodes, show=args.show)
