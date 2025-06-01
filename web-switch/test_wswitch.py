import unittest
from unittest.mock import MagicMock, call, patch

# we do not want to start the webserver during test
with patch.dict('sys.modules', {'http.server': MagicMock()}):
    from wswitch import WebPageSwitch, data_queue


class WebPageSwitchTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        switch = WebPageSwitch(config={}, bus=bus)

        data_queue.put(False)  # fake switch to False
        bus.is_alive = MagicMock(side_effect=[True, False])  # process 1 message from the queue
        switch.run()

        self.assertEqual(len(bus.method_calls), 4)
        self.assertEqual(bus.method_calls[-1], call.is_alive())
        self.assertEqual(bus.method_calls[-2], call.publish('status', False))

    def test_on_command(self):
        bus = MagicMock()
        config = {
            'on_cmd': [200, 0],
            'off_cmd': [0, 0]
        }
        switch = WebPageSwitch(config=config, bus=bus)

        data_queue.put(True)
        bus.is_alive = MagicMock(side_effect=[True, False])  # process 1 message from the queue
        switch.run()

        self.assertEqual(len(bus.method_calls), 5)
        self.assertEqual(bus.method_calls[-1], call.is_alive())
        self.assertEqual(bus.method_calls[-2], call.publish('cmd', [200, 0]))
        self.assertEqual(bus.method_calls[-3], call.publish('status', True))

    def test_off_command(self):
        bus = MagicMock()
        config = {
            'on_cmd': [200, 0],
            'off_cmd': [0, 0]
        }
        switch = WebPageSwitch(config=config, bus=bus)

        data_queue.put(False)
        bus.is_alive = MagicMock(side_effect=[True, False])  # process 1 message from the queue
        switch.run()

        self.assertEqual(len(bus.method_calls), 5)
        self.assertEqual(bus.method_calls[-1], call.is_alive())
        self.assertEqual(bus.method_calls[-2], call.publish('cmd', [0, 0]))
        self.assertEqual(bus.method_calls[-3], call.publish('status', False))

# vim: expandtab sw=4 ts=4
