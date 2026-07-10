# Bird's Eye View (BEV) Road Mapping

This project provides tools for Bird's Eye View (BEV) road mapping, perspective calibration, and transformation of camera images into orthographic top-down views.

## view_bev.py

`view_bev.py` is an interactive calibration tool to find the optimal homography parameters for road perspective warping.

### Features
- Interactive GUI sliders (trackbars) to adjust the perspective trapezoid.
- Real-time display of the source calibration boundaries and the warped BEV result.
- Reference vertical and horizontal grid lines to align curved or straight lane boundaries.
- Ability to save and load calibration configurations directly to/from JSON.

### Interactive Controls
- **Trackbars**:
  - `Top Y`: Vertical coordinate of the horizon/far-field.
  - `Bottom Y`: Vertical coordinate of the near-field (bottom of image).
  - `Top Width`: Width of the trapezoid's top edge.
  - `Bottom Width`: Width of the trapezoid's bottom edge.
  - `Offset X`: Shifts the trapezoid horizontally (e.g., to adjust for camera mounting offset).
  - `Show Grid (0/1)`: Toggle display of blue parallel guidelines in the BEV window.
- **Keyboard Shortcuts**:
  - `s`: Save the current calibration parameters to JSON.
  - `r`: Reset all trackbars to their default values.
  - `q` or `ESC`: Quit the application.

### Usage

Run the script inside your `uv` environment, passing the path to a road image:

```bash
uv run python bev-road/view_bev.py bev-road/610633206-1fbf3fa1-f417-46ce-ba58-e497ecc988e8.png
```

By default, calibration parameters are loaded from and saved to a file matching the image name (e.g., `bev-road/610633206-1fbf3fa1-f417-46ce-ba58-e497ecc988e8_bev.json`).

You can also specify a custom config file and output dimensions:

```bash
uv run python bev-road/view_bev.py <image-path> --config my_custom_calib.json --width 400 --height 600
```

## log2imdir.py

`log2imdir.py` extracts camera frames from an OSGAR log file at specific spatial distance intervals (calculated from a Pose2D stream) and creates a CSV overview of their spatial coordinates.

### Features
- Decodes video streams dynamically using `av` (H.264 or H.265/HEVC auto-detection).
- Decodes JPEG/MJPEG image streams.
- Computes cumulative distance from the Pose2D stream to extract frames at precise distance milestones.
- Generates a CSV overview file matching each extracted image filename with its relative timestamp, spatial coordinates (X, Y in meters), and heading (in radians).

### Usage

Run the script to extract images from a log file:

```bash
uv run python bev-road/log2imdir.py path/to/logfile.log
```

### Options
- `-o`, `--out`: Custom output directory (defaults to logfile name without extension).
- `--start`: Distance along path to start extraction, in meters (default: `1.0`).
- `--step`: Interval distance between extractions, in meters (default: `0.5`).
- `--end`, `--finish`: Distance along path to end extraction, in meters (default: `10.0`).
- `--camera`: Name of the camera stream (default: `"oak.color"`).
- `--pose`: Name of the Pose2D stream (default: `"platform.pose2d"`).

### Output CSV Format

The output directory contains extracted images (`img-0000.jpg`, `img-0001.jpg`, etc.) and an `overview.csv` file without headers structured as follows:

```csv
img-0000.jpg,1.120,1.002,0.015,1.570796
img-0001.jpg,3.480,2.985,0.032,1.571210
```
- Column 1: Image filename
- Column 2: Timestamp in seconds from log start
- Column 3: X coordinate in meters
- Column 4: Y coordinate in meters
- Column 5: Heading in radians

