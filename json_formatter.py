import datetime
import json
from utils import get_elevation, calculate_flight_duration

all_cases = ['mission', 'visualization_local', 'visualization_online']

class UAVPathPlanner:
    def __init__(self, data, WGS84_paths, WGS84_fov, geoCoords, Viewpoints, solver, sender="Alert-driven_UAV_path_planning",
                 mode='all'):
        self.sender = sender
        self.data = data
        self.ViewPoints = Viewpoints
        self.WGS84_paths = WGS84_paths
        self.WGS84_fov = WGS84_fov
        self.geoCoords = geoCoords
        self.solver = solver
        try:
            with open('google_api_key.txt', 'r') as f:
                self.api_key = f.read()
            print("✅ Using Google Elevation API (API key found).")
        except FileNotFoundError:
            self.api_key = None
            print("⚠️ 'Google API key not found. Falling back to Open-Elevation API (no API key required).")
        self.init_elevation = get_elevation(data['initialPos']['lat'], data['initialPos']['long'], key=self.api_key)
        self.mode = all_cases if mode == 'all' else [mode]

    def format_waypoint(self, waypoint):
        if self.visualize:
            latitude, longitude, *whatever = waypoint

            waypoint_out = {
                'lat': latitude,
                'lng': longitude,
            }
            return waypoint_out

        else:
            latitude, longitude, altitude, idx = waypoint
            elev_diff, elev = get_elevation(latitude, longitude, self.init_elevation, key=self.api_key)
            waypoint_out = {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude + elev_diff,
                'elevation': elev
            }
            if type(idx) == int:
                waypoint_out['heading'] = self.ViewPoints[idx, 3] + 90 if self.ViewPoints[idx, 3] < 90 else \
                    self.ViewPoints[idx, 3] - 90
                waypoint_out['photo'] = True
            else:
                waypoint_out['heading'] = None
                waypoint_out['photo'] = False

        return waypoint_out

    def format_path(self, path, count):
        waypoints = [self.format_waypoint(waypoint) for waypoint in path]
        if self.visualize:
            return {'points': waypoints}
        else:
            flight_duration = calculate_flight_duration(self.data, self.solver.flight_distance[count])
            return {
                'pathID': count,
                'photomode': 'viewpoint',
                'flightDuration': f'{flight_duration:.2f} minutes',
                'waypoints':waypoints
            }

    def format_polygon(self, path):
        lst = [self.format_waypoint(waypoint) for waypoint in path]
        return {
            'points': lst
        }

    def generate_output_json(self, case):
        out_json = {
            'sender': self.sender,
            'dateTime': str(datetime.datetime.now()),
            'horizontal_speed': self.data['horizontal_speed'],
            'vertical_speed': self.data['vertical_speed'],
        }

        if case == 'mission':
            self.visualize = False
            out_json['paths'] = [self.format_path(path, count) for count, path in enumerate(self.WGS84_paths)]
        elif case in ['visualization_local', 'visualization_online']:
            self.visualize = True
            out_json['paths'] = [self.format_path(path, count) for count, path in enumerate(self.WGS84_paths)]
            out_json['polygons'] = [self.format_polygon(path) for count, path in enumerate(self.geoCoords)]
            temp_fov = [self.format_polygon(path) for count, path in enumerate(self.WGS84_fov)]
            if case == 'visualization_online':
                out_json['obstacles'] = temp_fov
            else:
                out_json['fov'] = temp_fov
        return out_json

    def generate_output(self):
        for mode in self.mode:
            out_json = self.generate_output_json(mode)
            with open(f'{mode}.json', 'w') as outfile:
                json.dump(out_json, outfile, separators=(',', ':'), indent=4)
