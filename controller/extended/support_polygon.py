import numpy as np
from scipy.io import loadmat
from scipy.spatial import ConvexHull

class supportPolygon:
    def __init__(self, points, margin=0.):
        if margin <= 0.:
            self.points = points
        else:
            self.points = marginalize(points, margin)

        self.centroid = self.points.sum(0) / self.points.shape[0]
        self.cv_hull = ConvexHull(self.points, qhull_options="QJ")
        self.A = self.cv_hull.equations[:, :-1]
        self.b = self.cv_hull.equations[:, -1]
        self.b = self.b.reshape(self.b.shape[0], 1)
        self.tol = 1e-10


    def contains(self, point):
        point = point.reshape(2,1)
        return np.all(self.A @ point + self.b <= self.tol)


def get_circle_intersections(x0, y0, r0, x1, y1, r1):
    # check the algebra here: https://paulbourke.net/geometry/circlesphere/
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)


    x2 = x0 + a * (x1 - x0) / d
    y2 = y0 + a * (y1 - y0) / d

    return np.array([x2, y2])


def marginalize(points, margin):
    # new_points is such that
    # np.linalg.norm(points-marginalize(points, 0.01), axis=1) == margin * np.ones(points.shape[0])
    centroid = points.sum(0)/points.shape[0]
    new_points = np.zeros_like(points)
    for i, p in enumerate(points):
        d = np.sqrt((p[0] - centroid[0]) ** 2 + (p[1] - centroid[1]) ** 2)

        new_p = get_circle_intersections(centroid[0], centroid[1], d-margin,
                                 p[0], p[1], margin)
        new_points[i, 0] = new_p[0]
        new_points[i, 1] = new_p[1]
    return new_points




LF_footWF = np.array([  0.2,  0.15])
RF_footWF = np.array([  0.2, -0.15])
LH_footWF = np.array([ -0.15,  0.15])
RH_footWF = np.array([ -0.15, -0.15])
feet = np.vstack([LF_footWF, RF_footWF, LH_footWF, RH_footWF])

zmp = np.array([0.1,  0.0])

SP = supportPolygon(feet, margin=0.02)
print(SP.contains(zmp))