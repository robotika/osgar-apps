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
- Refactor to `argparse` for flexibility. [DONE]

### Accomplishments
- `osm_fetch.py`: Downloads OSM data via Overpass API. Supports custom `--bbox` and `--output`.
- `osm_path.py`: Builds a `networkx` graph and finds the shortest path. Supports custom `--input`, `--start`, and `--end`.
- `osm_view.py`: Generates PNG or opens interactive window (if GUI available) using `--show`.

### Usage Examples
```bash
# Fetch data for a custom area
uv run robotour/osm_fetch.py --bbox 50.101 14.406 50.111 14.435 --output stromovka.json

# Find path and print waypoints
uv run robotour/osm_path.py --input stromovka.json --start 50.1055 14.4285 --end 50.1085 14.4150

# Visualize (saves to stromovka.png)
uv run robotour/osm_view.py --input stromovka.json --start 50.1055 14.4285 --end 50.1085 14.4150
```

*Note: `--show` in `osm_view.py` requires a local GUI environment and will warn/fail in headless CLI environments.*

## Future Steps
- Integration with OSGAR as a navigation node.
- Obstacle avoidance and local planning.
- Robotour-specific logic (e.g., handling delivery locations).
