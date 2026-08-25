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
    # Stromovka Park, Prague
    # bbox = (south, west, north, east)
    stromovka_bbox = (50.101, 14.406, 50.111, 14.435)
    output_path = os.path.join(os.path.dirname(__file__), "stromovka.json")
    fetch_osm_data(stromovka_bbox, output_path)
