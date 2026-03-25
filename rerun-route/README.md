# Rerun Route

This application allows a mobile robot to re-run a route recorded in a previous OSGAR log file.

## Usage

Update `config/matty-rerun-route.json` with the path to your log file and the desired stream name:

```json
"init": {
  "logfile": "path/to/your/previous-run.log",
  "pose2d_stream": "platform.pose2d",
  "max_speed": 0.5
}
```

Run the application with OSGAR:

```bash
python -m osgar.record config/matty-rerun-route.json
```

## How it works

The `RerunRoute` module:
1. Opens the specified OSGAR log file.
2. Extracts the `pose2d` path.
3. Initializes `osgar.followpath.FollowPath` with the extracted route.
4. Executes the path following.
