
import subprocess
import os
import glob
import re
import sys

def analyze_log(log_path, config_path):
    print(f"Analyzing {log_path}...")
    cmd = [
        "uv", "run", "python", "-m", "osgar.replay",
        log_path,
        "--module", "app",
        "--config", config_path
    ]
    
    # We run without -F first to catch the first divergence
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    stats = {
        "diverged": False,
        "new_stop": False,
        "new_slowdown": False,
        "other_change": False,
        "error": None
    }

    if result.returncode != 0:
        stats["diverged"] = True
        # Look for AssertionError: ([data], [ref_data], dt)
        # Example: AssertionError: ([250, 0], [500, 0], datetime.timedelta(seconds=7, microseconds=372147))
        match = re.search(r"AssertionError: \(\[(-?\d+), (-?\d+)\], \[(-?\d+), (-?\d+)\]", result.stderr)
        if match:
            new_speed = int(match.group(1))
            new_steer = int(match.group(2))
            old_speed = int(match.group(3))
            old_steer = int(match.group(4))
            
            if new_speed == 0 and old_speed > 0:
                stats["new_stop"] = True
            elif new_speed < old_speed:
                stats["new_slowdown"] = True
            elif new_steer != old_steer:
                stats["new_steering"] = True
            else:
                stats["other_change"] = True
        else:
            if "AssertionError" in result.stderr:
                stats["other_change"] = True
            else:
                stats["error"] = result.stderr.splitlines()[-1] if result.stderr.splitlines() else "Unknown error"

    return stats

def main():
    if len(sys.argv) < 2:
        print("Usage: python report_stats.py <log_dir> [config_path]")
        return

    log_dir = sys.argv[1]
    config_path = sys.argv[2] if len(sys.argv) > 2 else "robotem-rovne/config/matty-redroad.json"
    
    log_files = glob.glob(os.path.join(log_dir, "*.log"))
    if not log_files:
        print(f"No log files found in {log_dir}")
        return

    print(f"Found {len(log_files)} log files.")
    
    total = 0
    diverged_count = 0
    stops_count = 0
    slowdowns_count = 0
    steering_count = 0
    errors = []

    for log_file in log_files:
        total += 1
        stats = analyze_log(log_file, config_path)
        
        if stats.get("diverged"):
            diverged_count += 1
            if stats.get("new_stop"):
                stops_count += 1
            elif stats.get("new_slowdown"):
                slowdowns_count += 1
            elif stats.get("new_steering"):
                steering_count += 1
        
        if stats.get("error"):
            errors.append((log_file, stats["error"]))

    print("\n" + "="*30)
    print("REPLAY STATISTICS")
    print("="*30)
    print(f"Total logs processed:  {total}")
    print(f"Behavior changed:      {diverged_count}")
    print(f"  - New Stops:         {stops_count}")
    print(f"  - New Slowdowns:     {slowdowns_count}")
    print(f"  - New Steering:      {steering_count}")
    print(f"  - Other changes:     {diverged_count - stops_count - slowdowns_count - steering_count}")
    
    if errors:
        print("\nErrors encountered:")
        for log, err in errors:
            print(f"  {log}: {err}")

if __name__ == "__main__":
    main()
