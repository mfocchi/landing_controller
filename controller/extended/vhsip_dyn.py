import numpy as np
import casadi as ca
from .utils import *

class VHSIP:
    def __init__(self, dt, L, max_spring_compression, m, g_mag, w_v, w_p, w_u, max_settling_time):
        self.dt = dt
        self.L = L
        self.max_spring_compression=max_spring_compression
        self.m=m
        self.g_mag=g_mag
        self.w_v=w_v
        self.w_p=w_p
        self.w_u=w_u
        self.max_settling_time=max_settling_time

        self.k = 0.
        self.d = 0.
        self.eig_z = 0.
        self.settling_time = 0.

        self.k_limit = self.m * (8 / self.max_settling_time) ** 2
        self.eig_z_limit = -np.sqrt(self.k_limit / self.m)

        self.vz_limit = np.exp(1) * self.max_spring_compression * self.eig_z_limit

        self.Axy = np.array([[0., 0.],
                             [1., 0.]])

        self.Bxy = np.array([[0.],
                             [0.]])


        self.Axy_d = np.eye(2)
        self.Axy_d[1, 0] = self.dt
        self.Bxy_d = np.zeros((2, 1))
        self.PHI_xy = np.empty((2, 2, 0)) * np.nan
        self.GAMMA_xy = np.empty((2, 1, 0)) * np.nan

        self.state_x0 = np.empty((2, 1)) * np.nan
        self.state_y0 = np.empty((2, 1)) * np.nan
        self.state_z0 = np.empty((2, 1)) * np.nan

        self.Cv = np.array([[1.0, 0.0]])
        self.Cp = np.array([[0.0, 1.0]])

        self.Q_xy = np.empty([2, 2]) * np.nan
        self.Q_xyu = np.empty([1, 2]) * np.nan
        self.Q_u = np.empty([1, 1]) * np.nan

        self.Q_vx = np.empty([1, 2]) * np.nan
        self.Q_vu = np.empty([1, 1]) * np.nan

        self.ctrl_knots = np.linspace(0, 1, num=6, endpoint=True)
        self.ctrl_indexes = np.zeros_like(self.ctrl_knots)

        self.zmp_xy = np.empty(2) * np.nan

    def set_init(self, init_vel):
        self.state_x0[0] = init_vel[0]
        self.state_y0[0] = init_vel[1]
        self.state_z0[0] = init_vel[2]

        self.state_x0[1] = 0.
        self.state_y0[1] = 0.
        self.state_z0[1] = 0.

    def propagation_matrices(self):
        # matrices that describe xy states starting from 0 to ctrl_horz

        self.PHI_xy[:, :, 0] = np.eye(2)
        self.GAMMA_xy[:, :, 0] = np.zeros((2, 1))

        for ii in range(1, self.ctrl_horz):
            omega_sq_dt = self.dt * self.omega_sq_ref[ii]
            self.Axy_d[0, 1] = omega_sq_dt
            self.Bxy_d[0, 0] = -omega_sq_dt

            self.PHI_xy[:, :, ii] = self.Axy_d @ self.PHI_xy[:, :, ii - 1]
            self.GAMMA_xy[:, :, ii] = self.Axy_d @ self.GAMMA_xy[:, :, ii - 1] + self.Bxy_d

    def compute_xy_dynamics(self, zmp_x=None, zmp_y=None):
        # state = [velocity]
        #         [position]

        if zmp_x is None:
            zmp_x = self.zmp_xy[0]
        if zmp_y is None:
            zmp_y = self.zmp_xy[1]

        for ii in range(self.ctrl_horz):
            self.state_x = self.PHI_xy[:, :, ii] @ self.state_x0 + self.GAMMA_xy[:, :, ii] * zmp_x
            self.state_y = self.PHI_xy[:, :, ii] @ self.state_y0 + self.GAMMA_xy[:, :, ii] * zmp_y

            self.T_p_com_ref[0, ii] = self.state_x[1]
            self.T_v_com_ref[0, ii] = self.state_x[0]
            self.T_a_com_ref[0, ii] = self.omega_sq_ref[ii] * (self.state_x[1] - zmp_x)

            self.T_p_com_ref[1, ii] = self.state_y[1]
            self.T_v_com_ref[1, ii] = self.state_y[0]
            self.T_a_com_ref[1, ii] = self.omega_sq_ref[ii] * (self.state_y[1] - zmp_y)

    def _cost_matrices(self):
        CvPxy = self.Cv @ self.PHI_xy[:, :, -1]
        CpPxy = self.Cp @ self.PHI_xy[:, :, -1]
        CvPu = self.Cv @ self.GAMMA_xy[:, :, -1]
        CpPu = self.Cp @ self.GAMMA_xy[:, :, -1]

        I1 = np.eye(1)

        # self.Q_xy = self.w_v * CvPxy.T @ CvPxy + self.w_p * CpPxy.T @ CpPxy # not usefull
        self.Q_xyu = self.w_v * CvPu.T @ CvPxy + self.w_p * (CpPu - I1).T @ CpPxy
        self.Q_u = self.w_v * CvPu.T @ CvPu + self.w_p * (CpPu - I1).T @ (CpPu - I1) + self.w_u * I1

        # self.Q_vx = self.w_v * CvPxy.copy()  # not usefull
        self.Q_vu = self.w_v * CvPu.copy()

    def _compute_zmp(self, vx_f, vy_f):
        Q_u_inv = np.linalg.inv(self.Q_u)
        Q_hash = -Q_u_inv @ self.Q_xyu
        Q_dagger = Q_u_inv @ self.Q_vu.T
        self.zmp_xy[0] = Q_dagger * vx_f + Q_hash @ self.state_x0
        self.zmp_xy[1] = Q_dagger * vy_f + Q_hash @ self.state_y0

    def solve_ocp(self, vx_f=0., vy_f=0.):
        self.propagation_matrices()
        self._cost_matrices()
        self._compute_zmp(vx_f, vy_f)

    def _reshape_arrays(self):
        self.T_p_com_ref = np.empty([3, self.ctrl_horz]) * np.nan
        self.T_v_com_ref = np.empty([3, self.ctrl_horz]) * np.nan
        self.T_a_com_ref = np.empty([3, self.ctrl_horz]) * np.nan

        self.omega_sq_ref = np.empty(self.ctrl_horz) * np.nan

        self.PHI_xy = np.empty((2, 2, self.ctrl_horz)) * np.nan
        self.GAMMA_xy = np.empty((2, 1, self.ctrl_horz)) * np.nan

    def set_z_dynamics(self, time, pz, vz, az):
        self.time = time.copy()
        self.ctrl_horz = time.shape[0]

        self.ctrl_indexes = (self.ctrl_knots * self.ctrl_horz).astype(int)

        self._reshape_arrays()

        self.T_p_com_ref[2, :] = pz
        self.T_v_com_ref[2, :] = vz
        self.T_a_com_ref[2, :] = az

        for i in range(self.ctrl_horz):
            self.omega_sq_ref[i] = (self.T_a_com_ref[2, i] + self.g_mag) / self.T_p_com_ref[2, i]

    def MSD_z_dynamics(self):
        # state = [velocity]
        #         [position]

        if self.state_z0[0, 0] < self.vz_limit:
            self.k = self.m * (self.state_z0[0, 0] / (np.exp(1) * self.max_spring_compression)) ** 2
        else:
            self.k = self.k_limit
        self.d = 2 * np.sqrt(self.m * self.k)

        self.eig_z = -np.sqrt(self.k / self.m)
        self.settling_time = -8 / self.eig_z

        time = np.arange(0, self.settling_time, self.dt)
        ctrl_horz = time.shape[0]

        pz = np.zeros([1, ctrl_horz])
        vz = np.zeros([1, ctrl_horz])
        az = np.zeros([1, ctrl_horz])

        for ii in range(ctrl_horz):
            t = time[ii]
            exp_eig_t = np.exp(self.eig_z * t)

            pz[:, ii] = exp_eig_t * t * self.state_z0[0, 0] + self.L
            vz[:, ii] = exp_eig_t * (self.eig_z * t + 1) * self.state_z0[0, 0]
            az[:, ii] = -self.d / self.m * vz[:, ii] - self.k / self.m * (pz[:, ii] - self.L)

        return time, pz, vz, az

    # def horz_velocity_reduction(self, gain):
    #     i = 0#self.ctrl_horz - 1
    #     while i <self.ctrl_horz:
    #         if np.all(self.T_v_com_ref[:2, i] >= gain * self.T_v_com_ref[:2, 0]):
    #             i += 1
    #         else:
    #             break
    #     t = i * self.dt
    #     return t, i

    def horz_velocity_reduction(self, gain):
        i = self.ctrl_horz - 1
        while i >= 0:
            if np.linalg.norm(self.T_p_com_ref[:2, i] - self.zmp_xy) > 1e-4 and \
                            np.all(np.sign(self.T_v_com_ref[:2, i]) != np.sign(self.T_v_com_ref[:2, 0])):
                i -= 1
            else:
                break
        t = i * self.dt
        return t, i


if __name__ == '__main__':
    ### INPUTS ###
    dt = 0.002
    m = 12.
    L = 0.25
    max_spring_compression = 0.1
    max_settling_time = 1.2
    apex_height = .6
    g_mag = 9.81
    pmin = 0.1
    pmax = 0.3
    Tfmax = 2

    state_x0 = np.array([[1],
                         [0.]])

    state_y0 = np.array([[0],
                         [0.]])

    state_z0 = np.array([[- np.sqrt(2 * apex_height * g_mag)],
                         [0.]])

    init_vel = np.array([state_x0[0], state_y0[0], state_z0[0]])

    w_v = 1.
    w_p = 1.
    w_u = 1.


    model = VHSIP(dt, L, max_spring_compression, m, g_mag, w_v, w_p, w_u, max_settling_time)
    model.set_init(init_vel)
    time, posz, velz, accz = model.MSD_z_dynamics()
    model.set_z_dynamics(time, posz, velz, accz)
    model.propagation_matrices()
    model.solve_ocp(vx_f=0, vy_f=0)
    model.compute_xy_dynamics(model.zmp_xy[0], model.zmp_xy[1])
