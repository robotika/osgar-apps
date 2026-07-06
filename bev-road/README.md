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
