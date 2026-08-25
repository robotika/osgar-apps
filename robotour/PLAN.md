# Robotour Project Plan

The goal is to create a "virtual reference competitor" for the Robotour contest (autonomous outdoor delivery challenge).

## Environment
- Use `uv` for dependency management.
- Dependencies: `requests`, `shapely`, `networkx`, `osgar`.

## Step 1: OSM Data and Pathfinding [COMPLETED]

### Goals
- Load OpenStreetMap (OSM) data for Stromovka Park, Prague. [DONE]
- Extract road segments suitable for the robot. [DONE]
- Implement pathfinding between two GPS coordinates. [DONE]

### Accomplishments
- `osm_fetch.py`: Downloads `stromovka.json` via Overpass API.
- `osm_path.py`: Builds a `networkx` graph and finds the shortest path.
- `osm_view.py`: Generates `stromovka_path.png` for verification.

## Future Steps
- Integration with OSGAR as a navigation node.
- Obstacle avoidance and local planning.
- Robotour-specific logic (e.g., handling delivery locations).
