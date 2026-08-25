# Robotour Project Plan

The goal is to create a "virtual reference competitor" for the Robotour contest (autonomous outdoor delivery challenge).

## Step 1: OSM Data and Pathfinding

### Goals
- Load OpenStreetMap (OSM) data for Stromovka Park, Prague.
- Extract road segments suitable for the robot (e.g., `highway` attributes like `footway`, `path`, `cycleway`, `track`, `service`).
- Implement pathfinding between two GPS coordinates.

### Technical Approach
1.  **OSM Data Acquisition**:
    - Use Overpass API to fetch data for the bounding box of Stromovka.
    - Filter for relevant `highway` tags.
    - Store data locally (e.g., `stromovka.osm` or `stromovka.json`).

2.  **Data Processing**:
    - Parse OSM XML/JSON.
    - Extract nodes (lat, lon) and ways (ordered list of nodes).
    - Build a graph where nodes are intersections and edges are road segments.

3.  **Pathfinding**:
    - Map start/end GPS coordinates to the nearest nodes in the graph.
    - Implement Dijkstra or A* algorithm for shortest path calculation.
    - Output a list of waypoints (GPS coordinates).

4.  **Verification**:
    - Create a script/tool to visualize the graph and the calculated path (e.g., using `matplotlib` or saving to GeoJSON for external viewing).

## Future Steps
- Integration with OSGAR as a navigation node.
- Obstacle avoidance and local planning.
- Robotour-specific logic (e.g., handling delivery locations).

## Dependencies (to be confirmed)
- `requests` (for fetching OSM data)
- `shapely` (already in project, for geometric operations)
- `networkx` (optional, for graph operations) or custom implementation.
