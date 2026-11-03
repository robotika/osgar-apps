import datetime
import struct
import tempfile
import unittest
from pathlib import Path

from osgar.lib.serialize import serialize
from osgar.logger import LogWriter
from osgar.platforms.matty import ESC, SYNC, RobotStatus, add_esc_chars

from dh2.encoder_audit import PacketDecoder, Status, analyze_intervals, incoming_stream, read_recording, signed_delta


def wire(payload):
    body = bytes([len(payload)]) + payload
    return bytes([SYNC]) + add_esc_chars(body + bytes([(-sum(body)) & 255]))


def status_packet(counter=1, encoders=(0, 0, 0, 0), flags=128, attitude=True):
    data = struct.pack('<BBBBHHhh4H', counter, ord('I'), flags, 2, 12000, 123, -100, -1234, *encoders)
    return data + (struct.pack('<hhh', -123, 456, -789) if attitude else b'')


class EncoderAuditTest(unittest.TestCase):
    def sample(self, t, values, counter=1, flags=128, speed=100):
        return Status(t, counter, flags, speed, 0, list(values), None)

    def test_fragmentation_batching_escaping_and_final_packet(self):
        payload1 = status_packet(encoders=(SYNC, ESC, 32768, 65535))
        payload2 = status_packet(counter=2, encoders=(1, 2, 3, 4), attitude=False)
        data = wire(payload1) + wire(b'\x05A') + wire(payload2)
        for split in range(len(data) + 1):
            decoder = PacketDecoder()
            decoder.feed(data[:split], 1)
            decoder.feed(data[split:], 2)
            self.assertEqual(len(decoder.statuses), 2)
            self.assertEqual(decoder.statuses[0].encoders, [SYNC, ESC, 32768, 65535])
            self.assertEqual(decoder.statuses[0].rpy, [-123, 456, -789])
            self.assertIsNone(decoder.statuses[1].rpy)
            self.assertEqual(decoder.statuses[-1].time, 2 if split < len(data) else 1)
            self.assertEqual(decoder.counts['type_A'], 1)
            self.assertFalse(decoder.frame)

    def test_corrupt_crc_and_truncated_escape_resynchronize(self):
        broken = bytearray(wire(status_packet()))
        broken[-1] ^= 1
        decoder = PacketDecoder()
        with self.assertLogs(level='WARNING'):
            decoder.feed(bytes(broken), 1)
        self.assertEqual(decoder.counts['checksum_rejections'], 1)
        decoder.feed(bytes([SYNC, 26, ESC]), 2)
        decoder.feed(wire(status_packet(counter=2)), 3)
        self.assertEqual(len(decoder.statuses), 1)
        self.assertEqual(decoder.statuses[0].counter, 2)
        self.assertEqual(decoder.counts['resynchronizations'], 1)
        decoder.feed(bytes([SYNC, 20, ESC, 0]), 4)
        decoder.feed(wire(bytes([3, ord('V'), 4, 8])), 5)
        self.assertEqual(decoder.counts['invalid_escapes'], 1)
        self.assertEqual(decoder.versions, [dict(time=5, robot=4, version=8)])

    def test_modulo_values_are_not_servo_4096_tick_counts(self):
        self.assertEqual(signed_delta([0, 65535, 32768, 12], [65535, 0, 32767, 15]), [1, -1, 1, -3])
        rows = analyze_intervals([self.sample(1, [65530] * 4, counter=255),
                                  self.sample(1.1, [4] * 4, counter=0)])
        self.assertTrue(rows[1]['accepted'])
        self.assertEqual(rows[1]['wheel_mean_delta_mm'], 10)
        self.assertAlmostEqual(rows[1]['speed_dt_mm'], 10)

    def test_error_and_first_recovery_interval_do_not_become_travel(self):
        samples = [self.sample(1, [0] * 4),
                   self.sample(1.1, [100] * 4, 2, RobotStatus.ERROR_ENCODER.value),
                   self.sample(1.2, [110] * 4, 3), self.sample(1.3, [120] * 4, 4)]
        rows = analyze_intervals(samples)
        self.assertIn('encoder_error', rows[1]['reason'])
        self.assertIn('after_encoder_error', rows[2]['reason'])
        self.assertTrue(rows[3]['accepted'])
        self.assertEqual(rows[-1]['cumulative_wheel_mean_mm'], 10)

    def test_missing_status_can_recover_cumulative_distance(self):
        rows = analyze_intervals([self.sample(1, [0] * 4, 10), self.sample(1.3, [30] * 4, 14)])
        self.assertTrue(rows[1]['accepted'])
        self.assertEqual(rows[1]['wheel_mean_delta_mm'], 30)
        self.assertAlmostEqual(rows[1]['speed_dt_mm'], 30)

    def test_reset_jump_repeated_time_and_gap_are_rejected(self):
        for second, resets, reason in [
            (self.sample(1.1, [10] * 4, 2), [1.05], 'reset_command'),
            (self.sample(1.1, [3000] * 4, 2), [], 'encoder_jump_or_reset'),
            (self.sample(1, [10] * 4, 2), [], 'nonpositive_interval'),
            (self.sample(2, [10] * 4, 2), [], 'time_gap'),
            (self.sample(1.1, [10] * 4, 1), [], 'counter_repeat_or_restart'),
        ]:
            with self.subTest(reason=reason):
                rows = analyze_intervals([self.sample(1, [0] * 4), second], resets)
                self.assertFalse(rows[1]['accepted'])
                self.assertIn(reason, rows[1]['reason'])
                self.assertEqual(rows[1]['cumulative_wheel_mean_mm'], 0)

    def test_signed_means_and_irregular_interval(self):
        rows = analyze_intervals([self.sample(1, [100] * 4),
                                  self.sample(1.2, [80] * 4, 2, speed=-100)])
        self.assertAlmostEqual(rows[-1]['cumulative_speed_dt_mm'], -20)
        self.assertAlmostEqual(rows[-1]['cumulative_fixed_100ms_mm'], -10)
        self.assertEqual(rows[-1]['cumulative_wheel_mean_mm'], -20)
        turning = analyze_intervals([self.sample(1, [100] * 4),
                                     self.sample(1.1, [90, 110, 110, 90], 2, speed=0)])
        self.assertEqual(turning[-1]['wheel_mean_delta_mm'], 0)
        self.assertTrue(turning[-1]['accepted'])

    def test_bounds_must_make_modular_difference_unambiguous(self):
        with self.assertRaisesRegex(ValueError, 'modulus'):
            analyze_intervals([], max_gap=30, max_wheel_speed=2)

    def test_reset_at_shared_timestamp_rejects_both_adjacent_intervals(self):
        samples = [self.sample(1, [0] * 4), self.sample(1.1, [10] * 4, 2),
                   self.sample(1.2, [20] * 4, 3)]
        rows = analyze_intervals(samples, [1.1])
        self.assertIn('reset_command', rows[1]['reason'])
        self.assertIn('reset_command', rows[2]['reason'])

    def test_source_is_input_link_not_outgoing_command_stream(self):
        config = {'robot': {'links': [['serial.raw', 'platform.esp_data'],
                                      ['platform.esp_data', 'serial.raw']]}}
        self.assertEqual(incoming_stream(config, ['platform.esp_data', 'serial.raw']), 'serial.raw')
        with self.assertRaises(ValueError):
            incoming_stream({}, ['platform.esp_data'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.log'
            with LogWriter(filename=str(path)) as log:
                log.write(0, repr(config).encode('ascii'), dt=datetime.timedelta(0))
                incoming = log.register('serial.raw')
                outgoing = log.register('platform.esp_data')
                messages = [(incoming, wire(status_packet(counter=255))),
                            (outgoing, wire(b'\x01R')),
                            (incoming, wire(b'\x00Psentence') + wire(status_packet(counter=1)))]
                for i, (stream, value) in enumerate(messages):
                    log.write(stream, serialize(value), dt=datetime.timedelta(seconds=i + 1))
            source, decoder, outgoing_decoder, data = read_recording(path)
            self.assertEqual(source, 'serial.raw')
            self.assertEqual(len(decoder.statuses), 2)
            self.assertEqual(decoder.info_counters, [255, 0, 1])
            self.assertEqual(outgoing_decoder.resets, [2])
            self.assertFalse(data['platform.pose2d'])


if __name__ == '__main__':
    unittest.main()
