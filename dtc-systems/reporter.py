"""
   DTC reporter to be used on Challenge Event 2 in September 2025
"""
import time
import json
from pathlib import Path

import requests
import cv2

from osgar.node import Node
from report import DTCReport, unpack_data
from osgar.drivers.lora import parse_lora_packet


#URL_BASE = "http://localhost"  # local Robotika test/demo
URL_BASE = "http://192.168.1.7"  # local Robotika test/demo


json_authorization = {
    # local testing server
    "Authorization": "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI4M2Q3OGM4ZS04MzhhLTQ0NzctOWM3Yi02N2VmMTZlNWY3MTYiLCJpIjowfQ.i4KuwEtc5_6oIYz5TDWcdzl5bMkvCpLZTSZG2Avy84w",
}

json_headers = {
    **json_authorization,
    "Content-Type" : "application/json",
}


def get_status():
    print('Get Status')
    url = URL_BASE + "/api/status"

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
            'file': f
        }
        response = requests.post(url, files=files, data=report_data, headers=json_authorization)
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
    return report_status['report_status'] == "accepted", report_status


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
        bus.register('server_response', 'lora_ack')
        self.is_team_reporter = config.get('is_team_reporter', False)
        self.grab_image = False
        self.report_index = 0
        Path('dtc_report/reports').mkdir(parents=True, exist_ok=True)
        Path('dtc_report/images').mkdir(parents=True, exist_ok=True)

    def on_report(self, data):
        report_cmd = data.copy()
        self.report_index += 1
        report_cmd["casualty_id"] = self.report_index

        if self.is_team_reporter:
            ok, server_response = submit_dtc_report(report_cmd)
            self.publish('server_response', server_response)

        print(self.time, f'REPORT {self.report_index}')
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

    def on_lora_report(self, data):
        addr, payload = data
        if 1 in addr:
            return  # note, hard link to base-station!
        r = unpack_data(payload)
        if r.casualty_id is None or r.casualty_id == 0:
            # just report of robot positions
            print(self.time, f'Pose {addr}: ({r.location_lat:.6f}, {r.location_lon:.6f})')
        else:
            print(self.time, f'Report {r.tojson()}')
            self.on_report(r.tojson())  # TODO refactor not to use on_* callback
            if self.is_team_reporter:
                # confirm receiving and successful processing
#                self.publish('lora_ack', data)
                pass


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
