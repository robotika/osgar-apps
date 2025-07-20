"""
   Report artifact for STIX in Colorado April 2019
         reused for Pittsburgh August 2019
     and reused again in Elma/Olympia February 2020
"""
import requests
import time
import math
import json

from osgar.node import Node


URL_BASE = "http://localhost"  # local Robotika test/demo
#URL_BASE = "http://localhost:8888"  # local Robotika test/demo
#URL_BASE = "http://10.100.1.200:8000"  # Alpha (was Army and Safety Research) Tunnel
#URL_BASE = "http://10.100.2.200:8000"  # Beta (was Miami and Experimental) Tunnel

#URL_BASE = "http://10.100.1.200:8000"  # Finals

ARTF_TYPES = ['Survivor', 'Backpack', 'Cell Phone',  # common
              'Drill', 'Fire Extinguisher',          # tunnel extra
              'Gas', 'Vent']                         # urban extra
ARTF_TYPES_SHORT = [x[0] for x in ARTF_TYPES]

json_headers = {
#    "Authorization" : "Bearer subttesttoken123",  # demo
#    "Authorization" : "Bearer NfEwAHEYsKqQkxSf",  # STIX
    "Authorization" : "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI4M2Q3OGM4ZS04MzhhLTQ0NzctOWM3Yi02N2VmMTZlNWY3MTYiLCJpIjowfQ.i4KuwEtc5_6oIYz5TDWcdzl5bMkvCpLZTSZG2Avy84w",  # test
    "Content-Type" : "application/json",
}


def get_status():
    print('Get Status')
    url = URL_BASE + "/api/status/"

    # Correct GET /api/status/ request
    response = requests.get(url, headers=json_headers)
    assert response.status_code == 200, response.status_code
    print(response.content)
    print("-------------------")
    return response.content


def report_artf(artf_type, x, y, z):
    report_data = {
"casualty_id": 1,
"team": "Robotika",
"system": "Matty M01",
        "location":
            {
                "latitude": 10,
                "longitude": 20,
                "time_ago": 1,
            }
}

    print('Report', report_data)
    url = URL_BASE + "/api/initial_report"

    # Correct POST /api/artifact_reports/ request
    response = requests.post(url, json=report_data, headers=json_headers)
    print(response.content)
    assert response.status_code == 201, response.status_code
    print("-------------------")
    return response.content


def report_artf0(artf_type, x, y, z):
    artifact_report_data = {
        "x": x,
        "y": y,
        "z": z,
        "type": artf_type,
    }
    print('Report', artifact_report_data)
    url = URL_BASE + "/api/artifact_reports/"

    # Correct POST /api/artifact_reports/ request
    response = requests.post(url, json=artifact_report_data, headers=json_headers)
    print(response.content)
    assert response.status_code == 201, response.status_code
    print("-------------------")
    return response.content


def triple(x, y, z):
    """
    Generate 3 coordinates with the same Z and XY in triangle of 4m
    """
    dist = 4.0
    arr = []
    for angle_deg in [0, 120, 240]:
        angle = math.radians(angle_deg)
        arr.append((x + math.cos(angle)*dist, y + math.sin(angle)*dist, z))
    return arr


def score(artf_type, x, y, z):
    """
    return True for scored artifact report
    """
    before = json.loads(bytes.decode(get_status()))
    time.sleep(2)
    report_artf(artf_type, x, y, z)
    time.sleep(2)
    after = json.loads(bytes.decode(get_status()))
    return before['score'] < after['score']


class Reporter(Node):
    def on_report(self, data):
        pass

    def on_image(self, data):
        pass


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Report artifact to server')
    parser.add_argument('artf_type', help='Type of artifact', choices=ARTF_TYPES + ARTF_TYPES_SHORT)
    parser.add_argument('x', help='X coordinate in meters', type=float)
    parser.add_argument('y', help='Y coordinate in meters', type=float)
    parser.add_argument('z', help='Z coordinate in meters', type=float)
#    parser.add_argument('--only-one', '-1', help='only one exact shot',
#                        action='store_true')
    args = parser.parse_args()

    artf_type = args.artf_type
    if artf_type in ARTF_TYPES_SHORT:
        artf_type = ARTF_TYPES[ARTF_TYPES_SHORT.index(artf_type)]

    print('Reporting:', artf_type)
    if True:  # args.only_one:
        print(score(artf_type, args.x, args.y, args.z))
    else:
        for x, y, z in triple(args.x, args.y, args.z):
            if score(artf_type, x, y, z):
                break

# vim: expandtab sw=4 ts=4

