"""
  Robotour project - OSM Data Acquisition
  Fetch OpenStreetMap (OSM) data for a given bounding box using Overpass API.
"""
import argparse
import requests
import json
import os

def fetch_osm_data(bbox, output_file):
    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = f"""
    [out:json];
    (
      way["highway"]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]});
    );
    out body;
    >;
    out skel qt;
    """
    headers = {
        'User-Agent': 'RobotourReferenceCompetitor/0.1 (https://github.com/robotika/osgar-apps)'
    }
    response = requests.get(overpass_url, params={'data': overpass_query}, headers=headers)
    if response.status_code == 200:
        data = response.json()
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Data saved to {output_file}")
    else:
        print(f"Error fetching data: {response.status_code}")
        print(response.text)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fetch OSM data for a given bounding box.')
    parser.add_argument('--bbox', type=float, nargs=4, 
                        default=[50.101, 14.406, 50.111, 14.435],
                        help='Bounding box: south west north east or '
                             'minlon minlat maxlon maxlat (default: Stromovka)')
    parser.add_argument('--output', default='stromovka.json', help='Output JSON file name')
    args = parser.parse_args()

    output_path = os.path.join(os.path.dirname(__file__), args.output)
    fetch_osm_data(args.bbox, output_path)
