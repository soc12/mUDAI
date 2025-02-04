from handleGeo.coordinates import WGS84
from handleGeo.coordinates.WGS84 import WGS84_class
from handleGeo.coordinates.NED import NED

import math
import numpy as np


class ConvCoords:
    """Converts geographic coordinates and obstacles to and from WGS84 and NED."""

    def __init__(self, geo_coords, geo_obstacles, init_pos):
        self.geo_coords = geo_coords
        self.geo_obstacles = geo_obstacles
        self.obstacles_wgs84 = geo_obstacles
        self.obstacles_ned = [[] for _ in range(len(self.geo_obstacles))]
        self.reference = WGS84_class(math.radians(init_pos['lat']),
                                    math.radians(init_pos['long']), 0)

    def conv_wgs84_to_ned(self, geo_coords):
        """Convert geographic coordinates from WGS84 to NED."""
        cart_coords = self.wgs84_to_ned(geo_coords)
        return cart_coords

    def obstacles_to_ned(self):
        """Convert obstacle coordinates from WGS84 to NED."""
        for i in range(len(self.geo_obstacles)):
            self.obstacles_ned[i] = self.wgs84_to_ned(self.geo_obstacles[i])
        return self.obstacles_ned

    def wgs84_to_ned(self, geo_coords):
        """Convert a set of geographic coordinates from WGS84 to NED."""
        cart_coords = np.zeros((len(geo_coords), len(geo_coords[0])))
        for i, coord in enumerate(geo_coords):
            wgs84 = WGS84_class(math.radians(coord[0]), math.radians(coord[1]), 0)
            ned = WGS84.displacement(self.reference, wgs84)

            cart_coords[i][0] = ned.north
            cart_coords[i][1] = ned.east

        return cart_coords

    def ned_to_wgs84(self, cart_coords, polygon=False):
        """Convert NED coordinates back to WGS84."""
        local = []
        self.waypoints_ned = cart_coords
        j = 0
        while j < len(cart_coords):
            geo_coords = []
            i = 0
            while i < len(cart_coords[j]):
                ned = NED(cart_coords[j][i][0], cart_coords[j][i][1], 0)
                wgs84 = WGS84.displace(self.reference, ned)
                if polygon:
                    geo_coords.append([math.degrees(wgs84.latitude),
                                       math.degrees(wgs84.longitude)])
                else:
                    geo_coords.append([math.degrees(wgs84.latitude),
                                       math.degrees(wgs84.longitude),
                                       cart_coords[j][i][2],
                                       cart_coords[j][i][3]])
                i += 1
            local.append(geo_coords)
            j += 1
        self.waypoints_wgs84 = local
        return self.waypoints_wgs84

    def get_obstacles_wgs84(self):
        """Return the original WGS84 obstacle coordinates."""
        return self.obstacles_wgs84

    def get_obstacles_ned(self):
        """Return the obstacle coordinates in NED format."""
        return self.obstacles_ned

    def get_polygon_wgs84(self):
        """Return the original WGS84 polygon coordinates."""
        return self.polygon_wgs84

    def get_polygon_ned(self):
        """Return the polygon coordinates in NED format."""
        return self.polygon_ned

    def get_waypoints_wgs84(self):
        """Return waypoints in WGS84 format."""
        return self.waypoints_wgs84

    def get_waypoints_ned(self):
        """Return waypoints in NED format."""
        return self.waypoints_ned
