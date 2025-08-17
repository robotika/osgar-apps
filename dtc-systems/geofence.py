import math
from random import Random

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

class Geofence:
    """
    A class to represent a geofence polygon and calculate distances to its border.
    
    This version uses a self-contained Haversine formula for distance calculation,
    removing the dependency on the geopy library.
    """

    # Earth's mean radius in meters
    EARTH_RADIUS_METERS = 6371 * 1000

    def __init__(self, coordinates):  # : list[list[float]]
        """
        Initializes the Geofence object.

        Args:
            coordinates (list[list[float]]): A list of [latitude, longitude] pairs
                                             in degrees that define the polygon's
                                             vertices in order.
        """
        if not coordinates or len(coordinates) < 3:
            raise ValueError("A polygon must have at least 3 points.")

        self.polygon_coords_lon_lat = [(lon, lat) for lat, lon in coordinates]
        self.geofence_poly = Polygon(self.polygon_coords_lon_lat)
        self.random = Random(0).random  # internal random generator with seed

    @staticmethod
    def _haversine_distance(pos1, pos2):
        # (pos1: tuple[float, float], pos2: tuple[float, float]) -> float:
        """
        Calculates the Haversine distance between two points on Earth.

        Args:
            pos1 (tuple): (latitude, longitude) for the first point in degrees.
            pos2 (tuple): (latitude, longitude) for the second point in degrees.

        Returns:
            float: The distance between the two points in meters.
        """
        lat1, lon1 = pos1
        lat2, lon2 = pos2

        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Difference in coordinates
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = Geofence.EARTH_RADIUS_METERS * c
        return distance

    def border_dist(self, position):  # : list[float] -> float:
        """
        Calculates the shortest distance from a position to the geofence border.

        Args:
            position (list[float]): A [latitude, longitude] pair in degrees.

        Returns:
            float: The shortest distance to the geofence border in meters.
                   The value is positive if inside, negative if outside, and 0 on the border.
        """
        point_lat, point_lon = position
        point_geom = Point(point_lon, point_lat)

        if self.geofence_poly.touches(point_geom):
            return 0.0

        _, nearest_point_on_boundary = nearest_points(point_geom, self.geofence_poly.exterior)
        nearest_lon, nearest_lat = nearest_point_on_boundary.x, nearest_point_on_boundary.y

        # Use the internal Haversine distance calculation
        distance_meters = self._haversine_distance((point_lat, point_lon), (nearest_lat, nearest_lon))
        
        is_inside = self.geofence_poly.contains(point_geom)

        return distance_meters if is_inside else -distance_meters

    def get_random_inner_waypoint(self, min_dist_from_border=2.0):
        """
        Get random point inside geofence
        :return:
        """
        bbox = self.geofence_poly.bounds
        assert len(bbox) == 4, bbox
        # 14.36, 50.04, 14.52, 50.11
        for i in range(10):
            t, u = self.random(), self.random()
            pt = t * bbox[1] + (1 - t) * bbox[3], u * bbox[0] + (1 - u) * bbox[2]
            if self.border_dist(pt) > min_dist_from_border:
                return pt
        return (bbox[1] + bbox[3]) / 2, (bbox[0] + bbox[2]) / 2  # fallback center (lat, lon)
