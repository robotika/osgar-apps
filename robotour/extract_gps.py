import json
import csv
import sys
import os

def extract(input_file, output_csv):
    base_time_sec = 0
    with open(input_file, 'r') as f:
        line = f.readline()
        if line.startswith('BASE_TIME:'):
            parts = line.split()
            # We don't necessarily need the absolute date, but we have the relative timestamps
            pass
        
        f.seek(0)
        
        results = []
        for line in f:
            if line.startswith('BASE_TIME:'):
                continue
            
            parts = line.split(maxsplit=3)
            if len(parts) < 4:
                continue
            
            ts = float(parts[0])
            endpoint = parts[1]
            msg_name = parts[2]
            try:
                payload = json.loads(parts[3])
            except json.JSONDecodeError:
                continue
            
            source = None
            if endpoint == 'ipc:///tmp/robot-gnss-dual':
                if msg_name == 'BESTNAV':
                    source = 'gnss_dual_primary'
                elif msg_name == 'BESTNAVH':
                    source = 'gnss_dual_secondary'
            elif endpoint == 'ipc:///tmp/robot-gnss-gps' and msg_name == 'BESTNAV':
                source = 'gnss_gps'
            elif endpoint == 'ipc:///tmp/robot-fusion' and msg_name == 'SOLUTION':
                source = 'fusion'
            
            if source and 'lat' in payload and 'lon' in payload:
                results.append({
                    'timestamp': ts,
                    'source': source,
                    'lat': payload['lat'],
                    'lon': payload['lon']
                })
    
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['timestamp', 'source', 'lat', 'lon'])
        writer.writeheader()
        writer.writerows(results)
    
    print(f"Extracted {len(results)} points to {output_csv}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        input_file = 'robotour/logger-10-25-48.dat'
        output_csv = 'robotour/gps_data.csv'
    else:
        input_file = sys.argv[1]
        output_csv = sys.argv[2]
    
    extract(input_file, output_csv)
