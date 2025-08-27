"""
  DTC report structure with pack/unpack for LoRa transmission
"""

import bitstring


class DTCReport:
    def __init__(self, lat, lon):
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


def pack_data(report):
    """Packs the data, handling the optional fields"""
    s = bitstring.BitStream()
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

    return s.tobytes()


def unpack_data(packed_bytes):
    """Safely unpacks the data, checking the presence flag."""
    unpacker = bitstring.BitStream(packed_bytes)

    # Read the mandatory fields
    lat_ms, lon_ms = unpacker.readlist('int:32, int:32')
    report = DTCReport(lat_ms/3_600_000, lon_ms/3_600_000)

    # Read the presence flag
    available = unpacker.read('bool:1')
    if available:
        # If the flag is true, read the optional field
        report.severe_hemorrhage = unpacker.read('uint:1')

    if unpacker.read('bool:1'):
        report.respiratory_distress = unpacker.read('uint:1')

    return report
