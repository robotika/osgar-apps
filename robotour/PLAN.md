# Robotour Project Plan

The goal is to create a "virtual reference competitor" for the Robotour contest (autonomous outdoor delivery challenge).

## Environment
- Use `uv` for dependency management.
- Dependencies: `requests`, `shapely`, `networkx`, `osgar`.

## Step 1: OSM Data and Pathfinding

### Goals
- Load OpenStreetMap (OSM) data for Stromovka Park, Prague.
- Extract road segments suitable for the robot (e.g., `highway` attributes like `footway`, `path`, `cycleway`, `track`, `service`).
- Implement pathfinding between two GPS coordinates.

### Technical Approach
1.  **OSM Data Acquisition (`osm_fetch.py`)**:
    - Use Overpass API to fetch data for the bounding box of Stromovka.
    - Query: `way["highway"]` within the bounding box.
    - Store data locally as `robotour/stromovka.json`.

2.  **Data Processing & Pathfinding (`osm_path.py`)**:
    - Parse OSM JSON.
    - Extract nodes (lat, lon) and ways.
    - Build a `networkx.Graph`.
    - Function `find_path(start_gps, end_gps)`:
        - Find nearest graph nodes to start/end GPS.
        - Use `networkx.shortest_path` (Dijkstra).
        - Return waypoints.

3.  **Visualization (`osm_view.py`)**:
    - Plot the graph and the calculated path using `matplotlib`.

## Future Steps
- Integration with OSGAR as a navigation node.
- Obstacle avoidance and local planning.
- Robotour-specific logic (e.g., handling delivery locations).
