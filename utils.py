import json
import requests


def process_data(filename):
    geoCoords = []
    file = open(filename)
    data = json.load(file)
    for i in data['polygons']:
        temp = []
        for j in i:
            temp.append([j.get("lat"), j.get("long")])
        geoCoords.append(temp)
    return geoCoords, data


def generate_camera_fov(solver, camera_dict, ax=None):
    camera_fov = []
    for UAV_view_points in solver.final_path_parcel_idx:
        for vp in UAV_view_points:
            if vp[0] - 1 < 0:
                continue
            temp = []
            camera_corners = camera_dict[vp[0] - 1]
            xx, yy = camera_corners.exterior.coords.xy
            for i in range(4):
                x = [xx[i], vp[1][0]]
                y = [yy[i], vp[1][1]]
                z = [0.0, vp[1][2]]
                temp.append([xx[i], yy[i]])
                if ax is not None:
                    ax.plot(x, y, z, color='black')
            camera_fov.append(temp)
    return camera_fov


def calculate_max_distance(data):
    battery_duration = data['batteryDuration']
    h_speed = data['horizontal_speed']
    v_speed = data['vertical_speed']
    a = data['hCoeff']
    b = data['vCoeff']
    offset = data['safety_factor']
    return int(offset * battery_duration * 60 * (a * h_speed + b * v_speed))


def calculate_flight_duration(data, distance):
    battery_duration = data['batteryDuration']
    h_speed = data['horizontal_speed']
    v_speed = data['vertical_speed']
    a = data['hCoeff']
    b = data['vCoeff']
    return distance / (60 * (a * h_speed + b * v_speed))


def get_elevation(latitude, longitude, init_elevation=None, key=None):
    if key is not None:
        url = f'https://maps.googleapis.com/maps/api/elevation/json?locations={latitude},{longitude}&key={key}'
    else:
        url = f'https://api.open-elevation.com/api/v1/lookup?locations={latitude},{longitude}'
    response = requests.get(url)
    data = response.json()
    elevation = data['results'][0]['elevation']
    if init_elevation is not None:
        return elevation - init_elevation, elevation
    else:
        return elevation


def eval_func(x, obj, a):
    xx, y, z, theta = x[0], x[1], x[2], x[3]
    score, _ = obj.evaluate(xx, y, z, theta)
    return -score
