import json
import sys

def merge_osm_files(file1, file2, output_file):
    with open(file1, 'r') as f:
        data1 = json.load(f)
    with open(file2, 'r') as f:
        data2 = json.load(f)
    
    # Standard Overpass JSON has 'elements'
    # My converted unidroids-osm.json also has 'elements'
    
    elements1 = data1.get('elements', [])
    elements2 = data2.get('elements', [])
    
    merged_data = {
        'elements': elements1 + elements2
    }
    
    with open(output_file, 'w') as f:
        json.dump(merged_data, f, indent=2)
    print(f"Merged {file1} and {file2} into {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python3 merge_osm.py <file1> <file2> <output>")
    else:
        merge_osm_files(sys.argv[1], sys.argv[2], sys.argv[3])
