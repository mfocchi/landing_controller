import numpy as np

from landingController import *
from feasibility import *
from bezier import *
from scipy.io import loadmat
import os


class ExtendedLandingController(LandingController):
    def __init__(self, robot, dt, q0, g_mag=9.81, smoothing_param = 0.005, naive=False):

        kinematic_region_filename = os.environ['LOCOSIM_DIR']+'/landing_controller/controller/go1_kin_region_l0.mat'
        lookup_table_forces_filename = os.environ['LOCOSIM_DIR'] + '/landing_controller/controller/lookup_table_forces.mat'
        super(ExtendedLandingController, self).__init__(robot, dt, q0, g_mag, smoothing_param, naive)
        # the limit (x, y) coordinates of virtual foot are -self.feasibility.points[:, :2]
        self.feasibility = Feasibility(kinematic_region_filename, type='KINEMATICS_AND_FRICTION')
        self.bezier = BezierLanding(dt)
        data = loadmat(lookup_table_forces_filename)
        self.lookup_table_forces = data['lookup_table_forces']


        self.ctrl_horz = self.bezier.max_N
        self.ctrl_range = range(1, self.ctrl_horz + 1)
        self.state_x = np.empty([2 * (self.ctrl_horz+1), 1]) * np.nan
        self.state_y = np.empty([2 * (self.ctrl_horz+1), 1]) * np.nan

        self.time = np.array([np.nan])

        self.T_p_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan
        self.T_v_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan
        self.T_a_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan

        self.Axy = np.array([[0., 0.],
                             [1., 0.]])

        self.Bxy = np.array([[0.],
                             [0.]])

        # self.Axy_d = np.empty((2,2,self.ctrl_horz + 1)) * np.nan
        # self.Bxy_d = np.empty((2,1,self.ctrl_horz + 1)) * np.nan
        self.Axy_d = np.empty((2, 2, self.ctrl_horz + 1)) * np.nan
        self.Bxy_d = np.empty((2, 1, self.ctrl_horz + 1)) * np.nan
        self.PHI_xy = np.empty((2 * self.ctrl_horz, 2)) * np.nan
        self.GAMMA_xy = np.empty((2 * self.ctrl_horz, 1)) * np.nan

        self.omega_sq_ref = np.empty([self.ctrl_horz + 1]) * np.nan


    def suboptimal_force(self, theta):
        idx = (np.abs(self.lookup_table_forces[:, 0] - theta)).argmin()
        return self.lookup_table_forces[idx, 1]


    def _propagation_matrices(self):
        # matrices that describe xy states starting from 0 to ctrl_horz

        PHI_xy = np.eye(2)
        GAMMA_xy = np.zeros((2, 1))

        for k in self.ctrl_range:
            self.Axy[0, 1] = self.omega_sq_ref[k]
            self.Bxy[0, 0] = -self.omega_sq_ref[k]

            # self.Axy_d[:,:,k] = np.eye(2) + self.dt * self.Axy
            # self.Bxy_d[:,:,k] = self.dt * self.Bxy
            #
            # PHI_xy   = self.Axy_d[:,:,k] @ PHI_xy
            # GAMMA_xy = self.Axy_d[:,:,k] @ GAMMA_xy + self.Bxy_d[:,:,k]

            self.Axy_d = self.dt * self.Axy
            self.Axy_d[0, 0] += 1.
            self.Axy_d[1, 1] += 1.

            self.Bxy_d = self.dt * self.Bxy

            PHI_xy = self.Axy_d @ PHI_xy
            GAMMA_xy = self.Axy_d @ GAMMA_xy + self.Bxy_d

            self.PHI_xy[2 * (k - 1):2 * k, :] = PHI_xy
            self.GAMMA_xy[2 * (k - 1):2 * k, :] = GAMMA_xy

    def _xy_dynamics(self, state_x, zmp_x, state_y, zmp_y):
        # state = [velocity]
        #         [position]
        self.state_x[0:2] = state_x
        self.state_y[0:2] = state_y
        self.state_x[2:] = self.PHI_xy @ state_x + self.GAMMA_xy @ zmp_x
        self.state_y[2:] = self.PHI_xy @ state_y + self.GAMMA_xy @ zmp_y

        self.T_p_com_ref[0, :] = self.state_x[1::2].flatten()
        self.T_v_com_ref[0, :] = self.state_x[0::2].flatten()

        self.T_p_com_ref[1, :] = self.state_y[1::2].flatten()
        self.T_v_com_ref[1, :] = self.state_y[0::2].flatten()

    def _z_dynamics(self, p0, pf, v0, a_max, af):
        self.time, pos, posd, posdd , T_thrust = self.bezier.computeBezier(p0, pf, v0, a_max, af)

        self.T_p_com_ref[2,:] = np.nan
        self.T_v_com_ref[2,:] = np.nan
        self.T_a_com_ref[2,:] = np.nan
        for i in range(self.time.shape[0]):
            self.T_p_com_ref[2, i] = pos[i]
            self.T_v_com_ref[2, i] = posd[i]
            self.T_a_com_ref[2, i] = posdd[i]
            self.omega_sq_ref[i] = (self.T_a_com_ref[2, i] - self.g_mag)/self.T_p_com_ref[2, i]


if __name__ == '__main__':
    from base_controllers.utils.common_functions import getRobotModel
    import base_controllers.params as conf

    robot_name = 'go1'
    robot = getRobotModel(robot_name, generate_urdf=False)
    dt = conf.robot_params[robot_name]['dt']
    q0 = pin.neutral(robot.model)
    q0[2] = 0.249
    q0[7:] = conf.robot_params[robot_name]['q_0']
    ELC = ExtendedLandingController(robot, dt, q0)

    u_test = -ELC.feasibility.points[0, :2]
    v0xy = 4. * u_test/np.linalg.norm(u_test)

    p0 = ELC.L
    pf = ELC.L
    v0 = -3
    a_max = ELC.suboptimal_force(np.arctan2(-u_test[1], -u_test[0] )) / robot.robotMass * 0.5
    af = 0
    ELC._z_dynamics(p0, pf, v0, a_max, af)

    fig = plt.figure()

    plt.title("thrusting traj")
    # plt.xlim(0, T_thrust)
    plt.xlabel("time [$s$]")
    plt.ylabel("posx ")
    plt.plot(ELC.time, ELC.T_p_com_ref[2,:ELC.time.shape[0]], "ob")
    plt.grid()

    fig = plt.figure()
    plt.title("thrusting traj der")
    # plt.xlim(0, T_thrust)
    plt.xlabel("time [$s$]")
    plt.ylabel("posdx ")
    plt.plot(ELC.time, ELC.T_v_com_ref[2,:ELC.time.shape[0]], "or")
    plt.grid()

    fig = plt.figure()
    plt.title("thrusting traj dder")
    # plt.xlim(0, T_thrust)
    plt.xlabel("time [$s$]")
    plt.ylabel("posddx ")
    plt.plot(ELC.time, ELC.T_a_com_ref[2,:ELC.time.shape[0]], "og")
    plt.grid()

    plt.show()






