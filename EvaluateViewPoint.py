import numpy as np
from shapely import affinity
from shapely.geometry import Polygon
np.set_printoptions(legacy='1.25')

class EvaluateViewPoint:

    def __init__(self, parcel, data, eval_with_IOU=True):
        self.parcel = Polygon(parcel)
        self.eval_with_IOU = eval_with_IOU
        theta_bounds = (0, 180)
        x_bounds = (np.min(parcel[:, 0]), np.max(parcel[:, 0]))
        y_bounds = (np.min(parcel[:, 1]), np.max(parcel[:, 1]))
        z_bounds = (data['min_altitude'], data['max_altitude'])
        self.bounds = [x_bounds, y_bounds, z_bounds, theta_bounds]
        self.hFOV = data['cameraSpecs']['hFOV']
        self.vFOV = data['cameraSpecs']['vFOV']

    def evaluate(self, x, y, z, theta):
        self.camera_polygon = self.camera_coverage(x, y, z, theta)
        if self.eval_with_IOU:
            metric = self.intersection_over_union(self.parcel, self.camera_polygon)
        else:
            metric = self.custom_objective_function(self.parcel, self.camera_polygon)
        return metric, self.camera_polygon

    def covered(self, altitude, FOV):
        return 2 * altitude * self.getTanFromDegrees(FOV / 2)

    def getTanFromDegrees(self, degrees):
        return np.tan(degrees * np.pi / 180)

    def intersection_over_union(self, p, c):
        if p.intersects(c):
            inter = p.intersection(c)
            return inter.area / (p.area + c.area - inter.area)
        else:
            return 0.0

    def custom_objective_function(self, p, c):
        if p.intersects(c):
            inter = p.intersection(c)
            return 2 * inter.area ** 2 - c.area
        else:
            return 0.0

    def camera_coverage(self, x, y, z, theta):
        H_d = self.covered(z, self.hFOV)
        V_d = self.covered(z, self.vFOV)

        x1 = x - V_d / 2
        x2 = x1
        x3 = x + V_d / 2
        x4 = x3

        y1 = y + H_d / 2
        y4 = y1
        y3 = y - H_d / 2
        y2 = y3

        p = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
        p_final = affinity.rotate(p, theta, 'center')

        return p_final
