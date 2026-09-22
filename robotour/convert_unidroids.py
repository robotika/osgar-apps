import json
import sys

def convert(input_file, output_file):
    with open(input_file, 'r') as f:
        data = json.load(f)
    
    elements = []
    
    # Convert nodes
    for node in data['nodes']:
        elements.append({
            'type': 'node',
            'id': node['id'],
            'lat': node['lat'],
            'lon': node['lon']
        })
    
    # Convert edges to ways
    for edge in data.get('edges', []):
        elements.append({
            'type': 'way',
            'id': edge['id'],
            'nodes': [edge['from'], edge['to']],
            'tags': {'highway': 'path'}  # OSMPath looks for 'highway' tag
        })
    
    with open(output_file, 'w') as f:
        json.dump({'elements': elements}, f, indent=2)
    print(f"Converted {input_file} to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 convert_unidroids.py <input> <output>")
    else:
        convert(sys.argv[1], sys.argv[2])
