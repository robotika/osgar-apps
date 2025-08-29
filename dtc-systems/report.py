"""
  DTC report structure with pack/unpack for LoRa transmission
"""

import bitstring


def normalize_matty_name(nickname):
    if 'Matty M' in nickname:
        return nickname
    if nickname[-1] == '-':
        nickname = nickname[:-1]  # cut - for OSGAR prefix name like `m03-`
    return 'Matty ' + nickname.upper()


class DTCReport:
    def __init__(self, system, lat, lon):
        # common header
        self.casualty_id = None  # to be filled on basestation
        self.system = normalize_matty_name(system)  # e.g. "Matty M01"

        # required fields
        self.location_lat = lat  # range from -90 and 90 in degrees
        self.location_lon = lon  # range from -180 and 180 in degrees
        # optional fields
        self.severe_hemorrhage = None  # <value_at_time>,    // 0, 1
        self.respiratory_distress = None  # <value_at_time>, // 0, 1
        self.hr = None  # <value_at_time>,  // beats per minute
        self.rr = None  # <value_at_time>,  // breaths per minute
        self.trauma_head = None  # <int>,  // 0, 1, 2
        self.trauma_torso = None  # <int>, // 0, 1, 2
        self.trauma_lower_ext = None  # <int>,  // 0, 1, 2, 3
        self.trauma_upper_ext = None  # <int>,  // 0, 1, 2, 3
        self.alertness_ocular = None  # <value_at_time>, // 0, 1, 2
        self.alertness_verbal = None  # <value_at_time>, // 0, 1, 2, 3
        self.alertness_motor = None  # <value_at_time>   // 0, 1, 2, 3

    def tojson(self):
        report_json = {
            "casualty_id": self.casualty_id,
            "team": "Robotika",
            "system": self.system,
            "location":
                {
                    "latitude": self.location_lat,
                    "longitude": self.location_lon,
                    "time_ago": 0
                }
        }
        if self.severe_hemorrhage is not None:
            report_json["severe_hemorrhage"] = {
                "value": self.severe_hemorrhage,
                "time_ago": 0
            }
        if self.respiratory_distress is not None:
            report_json["respiratory_distress"] = {
                "value": self.respiratory_distress,
                "time_ago": 0
            }
        if self.hr is not None:
            report_json['hr'] = {
                "value": self.hr,
                "time_ago": 0
            }
        if self.rr is not None:
            report_json["rr"] = {
                "value": self.rr,
                "time_ago": 0
            }
        if self.trauma_head is not None:
            report_json["trauma_head"] = self.trauma_head
        if self.trauma_torso is not None:
            report_json["trauma_torso"] = self.trauma_torso
        if self.trauma_lower_ext is not None:
            report_json["trauma_lower_ext"] = self.trauma_lower_ext
        if self.trauma_upper_ext is not None:
            report_json["trauma_upper_ext"] = self.trauma_upper_ext
        if self.alertness_ocular is not None:
            report_json["alertness_ocular"] = {
                "value": self.alertness_ocular,
                "time_ago": 0
            }
        if self.alertness_verbal is not None:
            report_json["alertness_verbal"] = {
                "value": self.alertness_verbal,
                "time_ago": 0
            }
        if self.alertness_motor is not None:
            report_json["alertness_motor"] = {
                "value": self.alertness_motor,
                "time_ago": 0
            }
        return report_json


def pack_data(report):
    """Packs the data, handling the optional fields"""
    s = bitstring.BitStream()
    assert report.system is not None
    assert len(report.system) >= 3, report.system
    assert report.system[-3:-1] == 'M0', report.system
    assert report.system[-1] in ['1', '2', '3', '4', '5'], report.system
    s.append(bitstring.pack('uint:5, uint:3', ord('M')-ord('A'), int(report.system[-1])))
    s.append(bitstring.pack('int:32, int:32',
                            int(round(report.location_lat * 3_600_000)),
                            int(round(report.location_lon * 3_600_000))))

    if report.severe_hemorrhage is not None:
        # If exists, set the flag to 1 and append the data
        s.append(bitstring.pack('bool:1, uint:1', True, report.severe_hemorrhage))
    else:
        # If is omitted, just set the flag to 0
        s.append(bitstring.pack('bool:1', False))

    if report.respiratory_distress is not None:
        s.append(bitstring.pack('bool:1, uint:1', True, report.respiratory_distress))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.hr is not None:
        s.append(bitstring.pack('bool:1, uint:8', True, report.hr))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.rr is not None:
        # Newborns: 30 to 60 breaths per minute
        s.append(bitstring.pack('bool:1, uint:6', True, report.rr))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.trauma_head is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.trauma_head))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.trauma_torso is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.trauma_torso))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.trauma_lower_ext is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.trauma_lower_ext))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.trauma_upper_ext is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.trauma_upper_ext))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.alertness_ocular is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.alertness_ocular))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.alertness_verbal is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.alertness_verbal))
    else:
        s.append(bitstring.pack('bool:1', False))

    if report.alertness_motor is not None:
        s.append(bitstring.pack('bool:1, uint:2', True, report.alertness_motor))
    else:
        s.append(bitstring.pack('bool:1', False))

    return s.tobytes()


def unpack_data(packed_bytes):
    """Safely unpacks the data, checking the presence flag."""
    unpacker = bitstring.BitStream(packed_bytes)

    # Read the mandatory fields
    letter, serial_num = unpacker.readlist('uint:5, uint:3')
    assert letter == ord('M') - ord('A'), letter
    sys_name = f'Matty M{serial_num:02}'
    lat_ms, lon_ms = unpacker.readlist('int:32, int:32')
    report = DTCReport(sys_name, lat_ms/3_600_000, lon_ms/3_600_000)

    # Read the presence flag
    available = unpacker.read('bool:1')
    if available:
        # If the flag is true, read the optional field
        report.severe_hemorrhage = unpacker.read('uint:1')

    if unpacker.read('bool:1'):
        report.respiratory_distress = unpacker.read('uint:1')

    if unpacker.read('bool:1'):
        report.hr = unpacker.read('uint:8')

    if unpacker.read('bool:1'):
        report.rr = unpacker.read('uint:6')

    if unpacker.read('bool:1'):
        report.trauma_head = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.trauma_torso = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.trauma_lower_ext = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.trauma_upper_ext = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.alertness_ocular = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.alertness_verbal = unpacker.read('uint:2')

    if unpacker.read('bool:1'):
        report.alertness_motor = unpacker.read('uint:2')

    return report
