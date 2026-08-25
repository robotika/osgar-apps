"""
  Test for Navigator node
"""
import unittest
from unittest.mock import MagicMock
import math

from robotour.navigator import Navigator

class NavigatorTest(unittest.TestCase):
    def test_navigation_logic(self):
        # Mock bus and config
        bus = MagicMock()
        # Stromovka coordinates near Planetarium
        config = {
            'osm_file': 'stromovka.json',
            'destination': [50.1085, 14.4150]
        }
        
        navigator = Navigator(config, bus)
        
        # 1. Initial GPS update to trigger pathfinding
        # Coordinates in 1/10^7 degrees: [lat, lon]
        # 50.1055, 14.4285
        start_gps = [501055000, 144285000]
        navigator.on_gps(start_gps)
        
        # Check if path was found and published
        self.assertTrue(bus.publish.called)
        # Find 'debug_path' call
        debug_path_call = [c for c in bus.publish.call_args_list if c[0][0] == 'debug_path']
        self.assertEqual(len(debug_path_call), 1)
        path = debug_path_call[0][0][1] # List of [lat, lon]
        self.assertGreater(len(path), 0)
        
        # 2. Update GPS to a point slightly ahead
        # Move slightly towards the first waypoint
        next_node_gps = path[1] # [lat, lon]
        # Coordinates in 1/10^7 degrees: [lat, lon]
        # Let's simulate being 10 meters away in latitude.
        # 1 degree of latitude is ~111km, so 1/10^7 deg is ~1.1cm.
        # 900 units is ~10 meters.
        curr_gps = [int(next_node_gps[0] * 10**7) - 900, int(next_node_gps[1] * 10**7)]
        
        # Reset mock to clear initial path publish
        bus.publish.reset_mock()
        navigator.on_gps(curr_gps)
        
        # Check navigator_info
        info_calls = [c for c in bus.publish.call_args_list if c[0][0] == 'navigator_info']
        self.assertGreater(len(info_calls), 0)
        last_info = info_calls[-1][0][1]
        self.assertIn('dist', last_info)
        self.assertIn('azimuth', last_info)
        # 900 * 10^-7 degrees * (40000000 / 360) approx 10.0 meters
        self.assertLess(last_info['dist'], 15.0) 
        self.assertGreater(last_info['dist'], 5.0)
        
        # 3. Simulate reaching a node (dist < 2m)
        reached_gps = [int(next_node_gps[0] * 10**7), int(next_node_gps[1] * 10**7)]
        bus.publish.reset_mock()
        navigator.on_gps(reached_gps)
        
        # Check if next_node_index incremented
        self.assertEqual(navigator.next_node_index, 2)

    def test_geo_logic(self):
        # Test internal geo functions
        from robotour.osm_path import geo_length, geo_angle
        # Two points 1/3600 degree apart on latitude (approx 30.86m)
        # (lon, lat)
        p1 = (14.0, 50.0)
        p2 = (14.0, 50.0 + 1.0/3600.0)
        
        dist = geo_length(p1, p2)
        self.assertAlmostEqual(dist, 30.86, places=1)
        
        angle = geo_angle(p1, p2)
        self.assertAlmostEqual(math.degrees(angle), 90.0)

if __name__ == '__main__':
    unittest.main()
