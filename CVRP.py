import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

OFFSET = 5.0  # TODO check with num_vehicles if there is enough space


class CVRP:
    def __init__(self, ViewPoints, num_vehicles, distance_capacity, json_data, time_limit):
        if json_data['multi_uav']:
            self.nom_alt = [json_data['flightAltitude'] + i * OFFSET for i in range(num_vehicles)]
        else:
            self.nom_alt = [json_data['flightAltitude'] for _ in range(num_vehicles)]
        if 'initialPos' in json_data:
            init_pos = np.array([0,0,self.nom_alt[0]])
        else:
            init_pos = ViewPoints.mean(axis=0)
        self.v_speed = json_data['vertical_speed']
        self.h_speed = json_data['horizontal_speed']
        speed_ratio = self.h_speed / self.v_speed
        allDiff = [2 * max(abs(max(ViewPoints[:, 2]) - alt),
                           abs(min(ViewPoints[:, 2]) - alt)) * speed_ratio for alt in self.nom_alt]
        self.heightDiff = max(allDiff)
        ViewPoints = np.concatenate(([init_pos], ViewPoints), axis=0)
        self.distance_capacity = distance_capacity
        self.ViewPoints = ViewPoints
        self.num_vehicles = num_vehicles
        self.data = self.create_data_model()
        if self.data['distance_matrix'][0].max()*2 > self.distance_capacity:
            raise Exception("Not enough battery! The distance from take-off point to most distant parcel exceeds battery capacity.")
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['distance_matrix']),
                                                    self.data['num_vehicles'], self.data['depot'])

        self.routing = pywrapcp.RoutingModel(self.manager)
        self.time_limit = time_limit
        self.final_path_ned = []
        self.final_path_parcel_idx = []
        self.plot_lines = []
        self.solution = None
        self.flight_distance = []

    def create_data_model(self):
        data = {}
        data['depot'] = 0
        data['distance_matrix'] = np.linalg.norm(self.ViewPoints[:, None] - self.ViewPoints, axis=2) \
                                  + self.heightDiff \
                                  + self.speed_penalty(self.h_speed) \
                                  + 2 * self.speed_penalty(self.v_speed)
        data['num_vehicles'] = self.num_vehicles
        return data

    def speed_penalty(self, speed):
        return speed ** 2 / 20 * (1 + abs(speed))

    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return int(self.data['distance_matrix'][from_node][to_node])

    def generate_final_paths(self, plot):
        """Prints solution on console."""
        vehicles_used = []
        for vehicle_id in range(self.data['num_vehicles']):
            if not self.routing.IsEnd(self.solution.Value(self.routing.NextVar(self.routing.Start(vehicle_id)))):
                vehicles_used.append(vehicle_id)
        # fig, ax = plt.subplots(1, 2)
        max_route_distance = 0
        for vehicle_id in vehicles_used:
            visual = []
            actual = []
            point_parcel = []
            index = self.routing.Start(vehicle_id)
            actual.append([self.ViewPoints[0, 0], self.ViewPoints[0, 1], self.nom_alt[vehicle_id], False])
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            while not self.routing.IsEnd(index):
                plan_output += ' {} -> '.format(self.manager.IndexToNode(index))
                idx = self.manager.IndexToNode(index)
                if idx:
                    visual.append(idx)
                    actual.append([self.ViewPoints[idx, 0], self.ViewPoints[idx, 1], self.nom_alt[vehicle_id], False])
                    actual.append([self.ViewPoints[idx, 0], self.ViewPoints[idx, 1], self.ViewPoints[idx, 2], idx - 1])
                    actual.append([self.ViewPoints[idx, 0], self.ViewPoints[idx, 1], self.nom_alt[vehicle_id], False])
                    point_parcel.append(
                        (idx, [self.ViewPoints[idx, 0], self.ViewPoints[idx, 1], self.ViewPoints[idx, 2]]))
                previous_index = index
                index = self.solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            actual.append([self.ViewPoints[0, 0], self.ViewPoints[0, 1], self.nom_alt[vehicle_id], False])
            visual.append(self.manager.IndexToNode(index))
            self.final_path_ned.append(actual)
            self.final_path_parcel_idx.append(point_parcel)
            if plot:
                xs = np.concatenate(([self.ViewPoints[0, 0]], self.ViewPoints[visual, 0], [self.ViewPoints[0, 0]]))
                ys = np.concatenate(([self.ViewPoints[0, 1]], self.ViewPoints[visual, 1], [self.ViewPoints[0, 1]]))
                self.plot_lines.append([xs, ys])

            plan_output += '{}\n'.format(self.manager.IndexToNode(index))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            print(plan_output)
            self.flight_distance.append(route_distance)
            max_route_distance = max(route_distance, max_route_distance)
        print('Maximum of the route distances: {}m'.format(max_route_distance))
        # ax[1].scatter(ViewPoints[:, 0], ViewPoints[:, 1])

    def mtsp(self, plot=True):
        transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)

        # Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        # Add Distance constraint.
        dimension_name = 'Distance'
        self.routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            self.distance_capacity,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name)
        distance_dimension = self.routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

        # Solve the problem.
        search_parameters.solution_limit = 10000
        #search_parameters.time_limit.seconds = self.time_limit
        self.solution = self.routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if self.solution:
            self.generate_final_paths(plot)
            return True
        else:
            print(f'No solution found for {self.num_vehicles} vehicles')
            return False
