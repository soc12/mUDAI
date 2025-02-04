import gmplot
import json
import matplotlib.pyplot as plt
from itertools import cycle
import os



def generate_fov_map(viz_file='visualization_local.json', mission_file='mission.json'):
    """
    Generates an interactive HTML map visualization from mission and FOV data.
    Enables clicking on the map to display coordinates and prints a clickable link.

    Args:
        viz_file (str): Path to the visualization JSON file.
        mission_file (str): Path to the mission JSON file.
    """
    try:
        with open(mission_file) as mission_data_file, open(viz_file) as viz_data_file:
            mission_data = json.load(mission_data_file)
            viz_data = json.load(viz_data_file)
    except IOError as e:
        print(f"Error opening files: {e}")
        return

    try:
        with open('google_api_key.txt', 'r') as f:
            api_key = f.read()
    except FileNotFoundError:
        api_key = None

    first_point = viz_data['paths'][0]['points'][0]
    initial_latitude, initial_longitude = first_point['lat'], first_point['lng']
    map_plotter = gmplot.GoogleMapPlotter(initial_latitude, initial_longitude, 18, map_type='satellite', apikey=api_key)

    # Add Clickable Polygons
    for feature_type, color, width in [('polygons', 'red', 2), ('fov', 'blue', 5)]:
        for feature in viz_data[feature_type]:
            latitudes = [point['lat'] for point in feature['points']]
            longitudes = [point['lng'] for point in feature['points']]
            map_plotter.polygon(latitudes, longitudes, color=color, edge_width=width)

    # Add Paths with dynamic colors
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = cycle(prop_cycle.by_key()['color'])
    for path in mission_data['paths']:
        latitudes = [waypoint['latitude'] for waypoint in path['waypoints']]
        longitudes = [waypoint['longitude'] for waypoint in path['waypoints']]
        map_plotter.plot(latitudes, longitudes, color=next(colors), edge_width=5)

        # Add Clickable Markers for waypoints
        for lat, lng in zip(latitudes, longitudes):
            map_plotter.marker(lat, lng)
        map_plotter.marker(latitudes[0], longitudes[0],label="T", color='blue')

    # Save the map
    output_html_file = "fov_map.html"
    map_plotter.draw(output_html_file)

    # Print a Clickable Link in Terminal
    abs_path = os.path.abspath(output_html_file)
    file_url = f"file://{abs_path}"  # For clickable links in some terminals
    print(f"✅ Interactive map saved to \033[94m{output_html_file}\033[0m (click to open in supported terminals)")
    print(f"🔗 Open manually by pasting in browser: {file_url}")

