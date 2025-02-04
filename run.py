from EvaluateViewPoint import EvaluateViewPoint
from handleGeo.ConvCoords import ConvCoords
from scipy.optimize import dual_annealing
from CVRP import CVRP
from json_formatter import *
from utils import *
from plot import *
from visualize_fov import generate_fov_map
import argparse

def main(filename):
    geoCoords, data = process_data(filename)
    MAX_SUPPORTED_DIST = calculate_max_distance(data)
    IOU_METRIC = data['iou_metric']
    num_vehicles = data['num_vehicles']

    convertor = ConvCoords([], [], data['initialPos'])
    ned_coords = [convertor.conv_wgs84_to_ned(i) for i in geoCoords]

    camera_dict = {}
    ViewPoints = np.zeros((len(ned_coords), 4))
    total_obj_score = 0
    for count, parcel in enumerate(ned_coords):
        print(f"\nOptimizing for parcel {count}")
        evaluator = EvaluateViewPoint(parcel, data, eval_with_IOU=IOU_METRIC)
        ret = dual_annealing(eval_func, args=(evaluator, None), bounds=evaluator.bounds, maxiter=100)
        ViewPoints[count] = ret['x']
        score, camera_view = evaluator.evaluate(*ViewPoints[count])
        print(f'Optimal configuration: {[*ViewPoints[count]]} with score: {score}')
        total_obj_score += score
        camera_dict[count] = camera_view

    print(f"Average score per parcel {total_obj_score/len(ned_coords)}")
    print(f'\nCalculating paths..')
    success = 0
    while not success:
        solver = CVRP(ViewPoints[:, :-1], num_vehicles=num_vehicles, distance_capacity=MAX_SUPPORTED_DIST,
                      json_data=data, time_limit=100)  # Time_limit for solver
        success = solver.mtsp()
        num_vehicles += num_vehicles

    if data['plot']:
        axes = plot_2d_3d(solver, ned_coords, camera_dict)
        axes[0].set_title('2D View')
        axes[1].set_title('3D View')
        camera_fov = generate_camera_fov(solver, camera_dict, axes[1])

        plt.tight_layout()
        plt.show()

    else:
        ax = None
        camera_fov = generate_camera_fov(solver, camera_dict, ax)

    WGS84_paths = convertor.ned_to_wgs84(solver.final_path_ned)
    WGS84_fov = convertor.ned_to_wgs84(camera_fov, True)

    mission = UAVPathPlanner(data, WGS84_paths, WGS84_fov, geoCoords, ViewPoints, solver)
    mission.generate_output()
    generate_fov_map()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process and evaluate GeoJSON data for viewpoints.")
    parser.add_argument('-filename', type=str, default='geojson/example.json', help='Path to the input GeoJSON file.')
    parser.add_argument('-outputname', type=str, help='Path to save the output data, optional.')
    args = parser.parse_args()
    main(args.filename)