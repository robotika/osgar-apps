# LidarRoad

Extract road boundary from WanJee down pointing lidar (10 degrees).

## Usage

You can run the script in single scan analysis mode or in batch processing mode.

### Single Scan Analysis

Analyzes and visualizes the smoothness of a road from a single scan:
```bash
uv run python lidarroad.py <logfile> [--jump <seconds>] [--width <meters>] [--tolerance <millimeters>] [--fast]
```

- `<logfile>`: Path to the OSGAR logfile.
- `--jump`, `-j`: Jump forward to the specified time in seconds before starting analysis.
- `--width`, `-w`: Window size/width in meters (default is `3.0`).
- `--tolerance`, `-t`: Smoothness difference tolerance in millimeters between neighboring scan points (default is `10`).
- `--fast`: Bypass the legacy comparative calculation and dual assertions for maximum execution speed.

### Batch Processing Mode

Analyzes scans over a specified time interval and draws a unified plot of the best matching boundaries over time:
```bash
uv run python lidarroad.py <logfile> --batch <start_sec> <end_sec> [--width <meters>] [--tolerance <millimeters>] [--fast]
```

- `--batch`: Runs in batch mode, taking exact `<start_sec>` and `<end_sec>` limits.
