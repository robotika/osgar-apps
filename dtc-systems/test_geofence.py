import unittest

from geofence import Geofence


class GeofenceTest(unittest.TestCase):
    """Test suite for the Geofence class."""

    def setUp(self):
        """Set up a common Geofence object for all tests."""
        prague_geofence_coords = [
            [50.11, 14.36],  # Northwest corner
            [50.11, 14.52],  # Northeast corner
            [50.04, 14.52],  # Southeast corner
            [50.04, 14.36],  # Southwest corner
        ]
        self.prague_geofence = Geofence(prague_geofence_coords)

    def test_point_inside(self):
        """Test a point that is clearly inside the geofence."""
        point_inside = [50.08, 14.42]  # Prague Castle
        distance = self.prague_geofence.border_dist(point_inside)
        self.assertGreater(distance, 0, "Distance for an inside point must be positive.")
        # We can also check if the value is in a reasonable range.
        self.assertAlmostEqual(distance, 3335.8, delta=1)

    def test_point_outside(self):
        """Test a point that is clearly outside the geofence."""
        point_outside = [50.13, 14.40]  # North of the geofence
        distance = self.prague_geofence.border_dist(point_outside)
        self.assertLess(distance, 0, "Distance for an outside point must be negative.")
        self.assertAlmostEqual(distance, -2223.8, delta=1)

    def test_point_on_border(self):
        """Test a point that lies exactly on the geofence border."""
        point_on_border = [50.11, 14.44]  # On the northern border
        distance = self.prague_geofence.border_dist(point_on_border)
        self.assertEqual(distance, 0.0, "Distance for a point on the border must be zero.")

    def test_invalid_polygon_initialization(self):
        """Test that initializing a geofence with too few points raises an error."""
        with self.assertRaises(ValueError, msg="Initializing with < 3 points should raise ValueError."):
            Geofence([[50.1, 14.4], [50.2, 14.5]])

    def test_random_point(self):
        waypoint = self.prague_geofence.get_random_inner_waypoint()
        self.assertGreater(self.prague_geofence.border_dist(waypoint), 0.0)
        waypoint2 = self.prague_geofence.get_random_inner_waypoint()
        self.assertNotEqual(waypoint, waypoint2)

if __name__ == '__main__':
    unittest.main()
