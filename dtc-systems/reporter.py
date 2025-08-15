"""
   DTC reporter to be used on Challenge Event 2 in September 2025
"""
import time
import json
from pathlib import Path

import requests
import cv2

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
    # local testing server
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


def initial_report(report_data):

    print('Report', report_data)
    url = URL_BASE + "/api/initial_report"

    # Correct POST /api/artifact_reports/ request
    response = requests.post(url, json=report_data, headers=json_headers)
    print(response.content)
    assert response.status_code in [200, 201], response.status_code
    print("-------------------")
    return response.content


def submit_dtc_image(casualty_id, img_path):
    report_data = {
        'casualty_id': casualty_id,
        "team": "Robotika",
        "system": "Matty M01",
        'time_ago': 0
    }
    print('Report', report_data)
    url = URL_BASE + "/api/casualty_image"

    with open(img_path, 'rb') as f:
        files = {
            'file': ("image.jpg", f, "image/jpeg")
        }
        response = requests.post(url, files=files, json=report_data, headers=json_headers)
    #    response = requests.post(url, json=report_data, headers=json_headers)
    print(response.content)
    assert response.status_code in [200, 201], response.status_code
    print("-------------------")
    return response.content


def submit_dtc_report(report_data):
    before = json.loads(bytes.decode(get_status()))
    time.sleep(2)
    report_status = json.loads(bytes.decode(initial_report(report_data)))
    time.sleep(2)
    after = json.loads(bytes.decode(get_status()))
    # DTC does not provide online reporting
    return report_status['report_status'] == "accepted"


def get_keyframe_image(data):
    # try H264 via OpenCV
    is_h264 = data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 0930'))
    is_h265 = data.startswith(bytes.fromhex('00000001 460150')) or data.startswith(bytes.fromhex('00000001 460130'))
    assert is_h264 or is_h265, data[:20].hex()
    tmp_filename = 'tmp-reporter.h26x'
    if data.startswith(bytes.fromhex('00000001 0950')) or data.startswith(bytes.fromhex('00000001 460150')):
        # I - key frame
        with open(tmp_filename, 'wb') as f:
            f.write(data)
    else:
        return None
    cap = cv2.VideoCapture(tmp_filename)
    ret, frame = cap.read()
#        if ret:
#            image = pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "BGR")
    cap.release()
    return frame


class Reporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.is_team_reporter = config.get('is_team_reporter', False)
        self.grab_image = False
        self.report_index = 0
        Path('dtc_report/reports').mkdir(parents=True, exist_ok=True)
        Path('dtc_report/images').mkdir(parents=True, exist_ok=True)

    def on_report(self, data):
        self.report_index += 1

        report_cmd = {
"casualty_id": self.report_index,
"team": "Robotika",
"system": "Matty M01",
"location":
{
"latitude": data['lat'],
"longitude": data['lon'],
"time_ago": 0
},
"severe_hemorrhage": {
"value": 1,
"time_ago": 0
},
"respiratory_distress": {
"value": 0,
"time_ago": 0
},
"hr": {
"value": 120,
"time_ago": 0
},
"rr": {
"value": 30,
"time_ago": 0
},
"temp": {
"value": 100,
"time_ago": 0
}
,
"trauma_head": 0,
"trauma_torso": 0,
"trauma_lower_ext": 1,
"trauma_upper_ext": 0,
"alertness_ocular": {
"value": 1,
"time_ago": 0
},
"alertness_verbal": {
"value": 2,
"time_ago": 0
},
"alertness_motor": {
"value": 2,
"time_ago": 0
}
}

        if self.is_team_reporter:
            submit_dtc_report(report_cmd)

        print(self.time, f'REPORT {self.report_index}:', data)
        filename = f'report{self.report_index}.json'
        with open(Path('dtc_report/reports') / filename, 'w') as fd:
            json.dump(report_cmd, fd)
        self.grab_image = True

    def on_image(self, data):
        if self.grab_image:
            # search for I-frame
            image = get_keyframe_image(data)
            if image is not None:
                filename = f'image{self.report_index}.jpg'
                print(self.time, f'Saving {filename} ...')
                img_path = str(Path('dtc_report/images') / filename)
                cv2.imwrite(img_path, image)
                self.grab_image = False

                if self.is_team_reporter:
                    submit_dtc_image(self.report_index, img_path)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Manual DTC Report/test to server')
    args = parser.parse_args()

    _report_data = {
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
    print(submit_dtc_report(_report_data))

# vim: expandtab sw=4 ts=4
