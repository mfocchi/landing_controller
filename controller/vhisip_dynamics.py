import numpy as np

class VHSIP:
    def __init__(self, L, dt=0.002, g_mag=9.81, w_v=1., w_p=1., w_u=1.):

        self.L = L

        self.dt = dt
        self.g_mag = g_mag

        self.ctrl_horz = 0
        self.time = np.empty(0) * np.nan

        self.T_p_com_ref = np.empty((3, 0)) * np.nan
        self.T_v_com_ref = np.empty((3, 0)) * np.nan
        self.T_a_com_ref = np.empty((3, 0)) * np.nan
        self.omega_sq_ref = np.empty(0) * np.nan


        self.Axy = np.array([[0., 0.],
                             [1., 0.]])

        self.Bxy = np.array([[0.],
                             [0.]])


        self.Axy_d = np.eye(2)
        self.Axy_d[1, 0] = self.dt
        self.Bxy_d = np.zeros((2, 1))
        self.PHI_xy = np.empty((2, 2, 0)) * np.nan
        self.GAMMA_xy = np.empty((2, 1, 0)) * np.nan

        self.state_x = np.empty((2, 1)) * np.nan
        self.state_y = np.empty((2, 1)) * np.nan

        self.state_x0 = np.empty((2, 1)) * np.nan
        self.state_y0 = np.empty((2, 1)) * np.nan
        self.state_z0 = np.empty((2, 1)) * np.nan

        self.Cv = np.array([[1.0, 0.0]])
        self.Cp = np.array([[0.0, 1.0]])

        self.Q_xy = np.empty([2,2]) * np.nan
        self.Q_xyu = np.empty([1,2]) * np.nan
        self.Q_u = np.empty([1,1]) * np.nan

        self.w_v = w_v
        self.w_p = w_p
        self.w_u = w_u

        self.zmp_xy = np.empty(2) * np.nan


    def _propagation_matrices(self):
        # matrices that describe xy states starting from 0 to ctrl_horz

        self.PHI_xy[:, :, 0] = np.eye(2)
        self.GAMMA_xy[:, :, 0] = np.zeros((2, 1))

        for ii in range(1, self.ctrl_horz):
            omega_sq_dt =  self.dt * self.omega_sq_ref[ii]
            self.Axy_d[0, 1] = omega_sq_dt
            self.Bxy_d[0, 0] = -omega_sq_dt

            self.PHI_xy[:, :, ii] = self.Axy_d @ self.PHI_xy[:, :, ii-1]
            self.GAMMA_xy[:, :, ii] = self.Axy_d @ self.GAMMA_xy[:, :, ii-1] + self.Bxy_d

    def compute_xy_dynamics(self, zmp_x, zmp_y):
        # state = [velocity]
        #         [position]

        for ii in range(self.ctrl_horz):
            self.state_x = self.PHI_xy[:, :, ii] @ self.state_x0 + self.GAMMA_xy[:, :, ii] * zmp_x
            self.state_y = self.PHI_xy[:, :, ii] @ self.state_y0 + self.GAMMA_xy[:, :, ii] * zmp_y

            self.T_p_com_ref[0, ii] = self.state_x[1]
            self.T_v_com_ref[0, ii] = self.state_x[0]
            self.T_a_com_ref[0, ii] = self.omega_sq_ref[ii] * (self.state_x[1] - zmp_x )

            self.T_p_com_ref[1, ii] = self.state_y[1]
            self.T_v_com_ref[1, ii] = self.state_y[0]
            self.T_a_com_ref[1, ii] = self.omega_sq_ref[ii] * (self.state_y[1] - zmp_y)

    def _cost_matrices(self):

        CvPxy = self.Cv @ self.PHI_xy[:, :, -1]
        CpPxy = self.Cp @ self.PHI_xy[:, :, -1]
        CvPu  = self.Cv @ self.GAMMA_xy[:, :, -1]
        CpPu  = self.Cp @ self.GAMMA_xy[:, :, -1]

        I1 = np.eye(1)

        self.Q_xy = self.w_v * CvPxy.T @ CvPxy + self.w_p * CpPxy.T @ CpPxy
        self.Q_xyu = self.w_v * CvPu.T @ CvPxy + self.w_p * (CpPu - I1).T @ CpPxy
        self.Q_u = self.w_v * CvPu.T @ CvPu + self.w_p * (CpPu - I1).T @ (CpPu - I1) + self.w_u * I1

    def _compute_zmp(self):
        Q_hash = -np.linalg.inv(self.Q_u) @ self.Q_xyu
        self.zmp_xy[0] = Q_hash @ self.state_x0
        self.zmp_xy[1] = Q_hash @ self.state_y0

    def solve_ocp(self, vf):
        self._propagation_matrices()
        self._cost_matrices()
        self._compute_zmp()


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

        self._reshape_arrays()

        self.T_p_com_ref[2, :] = pz
        self.T_v_com_ref[2, :] = vz
        self.T_a_com_ref[2, :] = az

        for i in range(self.ctrl_horz):
            self.omega_sq_ref[i] = (self.T_a_com_ref[2, i] + self.g_mag) / self.T_p_com_ref[2, i]

    def MSD_z_dynamics(self, m, k, d):
        # state = [velocity]
        #         [position]
        eig_z = -np.sqrt(k / m)
        settling_time = -8/eig_z

        time = np.arange(0, settling_time, self.dt)
        ctrl_horz = time.shape[0]

        pz = np.zeros([1, ctrl_horz])
        vz = np.zeros([1, ctrl_horz])
        az = np.zeros([1, ctrl_horz])


        for ii in range(ctrl_horz):
            t = time[ii]
            exp_eig_t = np.exp(eig_z * t)

            pz[:, ii] = exp_eig_t * t * self.state_z0[0,0] + self.L
            vz[:, ii] = exp_eig_t * (eig_z * t + 1) * self.state_z0[0,0]
            az[:, ii] = -d / m * vz[:, ii] - k / m * (pz[:, ii] - self.L)

        return time, pz, vz, az

    def set_init(self, state_x0, state_y0, state_z0):
        self.state_x0 = state_x0.copy()
        self.state_y0 = state_y0.copy()
        self.state_z0 = state_z0.copy()



if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    m = 12.
    L = 0.25
    max_spring_compression = 0.1
    max_settling_time = 1.2

    state_x0 = np.array([[-0.5],
                         [0.]])

    state_y0 = np.array([[0.],
                         [0.]])

    state_z0 = np.array([[-1.],
                         [0.]])

    vhsip = VHSIP(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)

    k_limit = m * (8 / max_settling_time)**2
    eig_z_limit = -np.sqrt(k_limit/m)

    vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

    if state_z0[0,0] < vz_limit:
        k = m * ( state_z0[0, 0] / (np.exp(1) * max_spring_compression) )**2
    else:
        k = k_limit
    d = 2 * np.sqrt(m * k)

    vhsip.set_init(state_x0, state_y0, state_z0)
    time, pz, vz, az = vhsip.MSD_z_dynamics(m, k, d)
    vhsip.set_z_dynamics(time, pz, vz, az)
    vhsip.solve_ocp()
    vhsip.compute_xy_dynamics(vhsip.zmp_xy[0], vhsip.zmp_xy[1])

    fig, axs = plt.subplots(3, 1,  figsize=(10,5))

    axs[0].set_ylabel("c [$m$]")
    axs[0].plot(vhsip.time, vhsip.T_p_com_ref.T)
    axs[0].grid()

    axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
    axs[1].plot(vhsip.time, vhsip.T_v_com_ref.T)
    axs[1].grid()


    axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
    axs[2].plot(vhsip.time, vhsip.T_a_com_ref.T)
    axs[2].grid()

    plt.show()

    legend = []
    legend.append(Line2D([0], [0], color=axs[0].lines[0].get_color(), lw=4, label="x"))
    legend.append(Line2D([0], [0], color=axs[0].lines[1].get_color(), lw=4, label="y"))
    legend.append(Line2D([0], [0], color=axs[0].lines[2].get_color(), lw=4, label="z"))

    fig.legend(handles=legend,
               loc="center right",  # Position of legend
               borderaxespad=1,  # Small spacing around legend box
               )

    plt.subplots_adjust(right=0.85)


