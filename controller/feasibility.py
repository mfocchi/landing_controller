import numpy as np
from scipy.io import loadmat

from scipy.spatial import ConvexHull

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Polygon

class Feasibility:
    def __init__(self, filename, type):
        data = loadmat(filename)
        if type == 'ONLY_KINEMATICS':
            self.region = data['kin_regions']
        elif type == 'KINEMATICS_AND_FRICTION':
            self.region = data['kin_and_fric_reg'][0]
        self.points = np.vstack(self.region)
        self.cv_hull = ConvexHull(self.points, qhull_options="QJ")
        self.A = self.cv_hull.equations[:, :-1]
        self.b = self.cv_hull.equations[:, -1]
        self.tol = 1e-12

    def checkPointFeasibility(self, point):
        return all(self.A @ point + self.b <= self.tol)

    def checkTrajFeasibility(self, traj):
        for point in traj:
            if not self.checkPointFeasibility(point):
                return False
        return True



if __name__ == '__main__':
    filename = "go1_kin_region.mat"
    feasibility_kin = Feasibility(filename, "ONLY_KINEMATICS")
    feasibility_oth = Feasibility(filename, "KINEMATICS_AND_FRICTION")

    traj = np.zeros([50, 3])
    traj[:, 0] = np.linspace(-0.2, 0.2, 50)
    traj[:, 1] = 0
    traj[:, 2] = 0.25

    for f in [feasibility_kin, feasibility_oth]:
        scale = np.linspace(50, 150, len(f.region))
        jet = cm = plt.get_cmap('RdBu')
        cNorm = colors.Normalize(vmin=55, vmax=145)
        scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        idx = 5
        for kin_reg in f.region:
            colorVal = scalarMap.to_rgba(scale[idx])
            idx -= 1
            p = Polygon(kin_reg[:, :2], label=str(kin_reg[0, 2]), edgecolor='k', facecolor=colorVal)
            ax.add_patch(p)
            art3d.pathpatch_2d_to_3d(p, z=kin_reg[0, 2], zdir="z")
        ax.set_xlim(-0.4, 0.4)
        ax.set_ylim(-0.4, 0.4)
        ax.set_zlim(-0.0, 0.3)

        ax.legend(title='height [m]')

        # check points in convex hull

        elapsed = []
        for p_test in traj:
            res = f.checkPointFeasibility(p_test)
            if res:
                color = 'b'
            else:
                color = 'r'
            ax.scatter(p_test[0], p_test[1], p_test[2], color=color, marker='o', s=10)
        plt.show()

        print("fully feasible trajectory?", f.checkTrajFeasibility(traj))