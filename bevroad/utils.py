import json
import math
import os
import sys


def load_calibration(config_path):
    """
    Loads calibration parameters from a JSON file. Exits on failure.
    """
    if not os.path.exists(config_path):
        print(f"Error: Calibration config file '{config_path}' does not exist.")
        sys.exit(1)
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f'Error loading calibration config {config_path}: {e}')
        sys.exit(1)


def load_config(config_path):
    """
    Loads config if it exists, otherwise returns empty dictionary.
    """
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f'Error loading config {config_path}: {e}')
    return {}


def save_config(config_path, data):
    """
    Saves configuration data to a JSON file.
    """
    try:
        with open(config_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f'Config successfully saved to: {config_path}')
    except Exception as e:
        print(f'Error saving config {config_path}: {e}')


def parse_csv(csv_path):
    """
    Parses metadata CSV file containing image filename, timestamp, robot_x, robot_y, heading.
    """
    if not os.path.exists(csv_path):
        print(f"Error: Metadata CSV file '{csv_path}' does not exist.")
        sys.exit(1)

    records = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) < 5:
                print(f'Warning: line {line_num} in CSV is invalid and will be skipped.')
                continue
            try:
                filename = parts[0]
                ts = float(parts[1])
                x = float(parts[2])
                y = float(parts[3])
                heading = float(parts[4])
                records.append({'filename': filename, 'ts': ts, 'x': x, 'y': y, 'heading': heading})
            except ValueError as e:
                print(f'Warning: line {line_num} parsing failed ({e}). Skipped.')
    return records


def transform_point(x_local, y_local, robot_x, robot_y, heading):
    """
    Transforms local robot-frame coordinates (X=forward, Y=left)
    to global coordinates based on robot's position and heading (radians).
    """
    c = math.cos(heading)
    s = math.sin(heading)
    x_global = robot_x + x_local * c - y_local * s
    y_global = robot_y + x_local * s + y_local * c
    return x_global, y_global


def global_to_local(gx, gy, rx, ry, heading):
    """
    Transforms global coordinates to local robot-frame coordinates (X=forward, Y=left)
    based on the robot's position and heading (radians).
    """
    dx = gx - rx
    dy = gy - ry
    c = math.cos(heading)
    s = math.sin(heading)
    x_local = dx * c + dy * s
    y_local = -dx * s + dy * c
    return x_local, y_local
