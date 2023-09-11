import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
import casadi as ca

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

        self.Q_vx = np.empty([1, 2]) * np.nan
        self.Q_vu = np.empty([1, 1]) * np.nan

        self.w_v = w_v
        self.w_p = w_p
        self.w_u = w_u

        self.zmp_xy = np.empty(2) * np.nan

    def duplicate(self):
        return VHSIP(self.L, self.dt, self.g_mag, self.w_v, self.w_p, self.w_u)

    def propagation_matrices(self):
        # matrices that describe xy states starting from 0 to ctrl_horz

        self.PHI_xy[:, :, 0] = np.eye(2)
        self.GAMMA_xy[:, :, 0] = np.zeros((2, 1))

        for ii in range(1, self.ctrl_horz):
            omega_sq_dt =  self.dt * self.omega_sq_ref[ii]
            self.Axy_d[0, 1] = omega_sq_dt
            self.Bxy_d[0, 0] = -omega_sq_dt

            self.PHI_xy[:, :, ii] = self.Axy_d @ self.PHI_xy[:, :, ii-1]
            self.GAMMA_xy[:, :, ii] = self.Axy_d @ self.GAMMA_xy[:, :, ii-1] + self.Bxy_d

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

    def bezier_z_dynamicsOpti(self, p0, v0, amax, pmin, pmax, Tfmax=None, Tf=None, pf=None, vf=None):
        opti = ca.Opti()

        # optimization variables: bezier coefficient, time
        if Tf is None:
            T = opti.variable(1)
        else:
            T = Tf

        # wp0 = opti.variable(1) no need of this
        wp0 = p0
        wp1 = opti.variable(1)
        wp2 = opti.variable(1)
        wp3 = opti.variable(1)
        if pf is None or not (pmin < pf < pmax):  # if pf is given, set w4=pf
            wp4 = opti.variable(1)
        else:
            wp4 = pf

        wp = ca.vcat([wp0, wp1, wp2, wp3, wp4])

        # for simplify computations, let define coeffs of bezier derivatives (do not consider time)
        wv0 = 4 * (wp1 - wp0)
        wv1 = 4 * (wp2 - wp1)
        wv2 = 4 * (wp3 - wp2)
        wv3 = 4 * (wp4 - wp3)
        wv = ca.vcat([wv0, wv1, wv2, wv3])

        wa0 = 3 * (wv1 - wv0)
        wa1 = 3 * (wv2 - wv1)
        wa2 = 3 * (wv3 - wv2)
        wa = ca.vcat([wa0, wa1, wa2])

        # COST
        opti.minimize(-(wv[-1] / T) ** 2)  # 'max vf^2'

        # CONSTRAINTS
        # initial position
        # opti.subject_to(wp0==p0)
        #
        # final position
        if type(wp4) == ca.casadi.MX:  # if pf is given, set w4=pf
            opti.subject_to(wp4 <= pmax)
        #
        # initial velocity
        opti.subject_to(wv0 / T == v0)
        # causality, limited time
        if Tf is None:
            #opti.subject_to(T > 0.1)
            opti.subject_to(T <= Tfmax)

        # positive snap
        s = wa2 - 2 * wa1 + wa0
        snap = 2 * s / T ** 4
        opti.subject_to(s > 0)

        # the minimum of the acceleration is postive
        a = s / (T ** 4)
        b = 2 * (wa1 - wa0) / (T ** 3)
        c = wa0 / (T ** 2)

        Delta = b ** 2 - 4 * a * c
        av = -Delta / (4 * a)
        opti.subject_to(Delta <= 0.)
        # opti.subject_to(av >= 20.)

        # and in (0, T)
        tv = - b / (2 * a)
        tauv = tv / T
        opti.subject_to(tv < T)

        # initial and final acceleration are upper-bounded
        opti.subject_to(wa0 / (T ** 2) <= amax)
        opti.subject_to(wa2 / (T ** 2) <= amax)

        # zero velocity at minimum of acceleration
        vv = self.bezier(wv / T, tauv)
        opti.subject_to(vv == 0)

        # lower bound of position
        pv = self.bezier(wp, tauv)
        opti.subject_to(pv >= pmin)

        if vf is None:
            # positive final velocity
            opti.subject_to(wv3 >= 0)
        else:
            # if vf is given, equality constaint
            opti.subject_to(wv3 / T == vf)  # (not tested)

        # INITIAL GUESS
        opti.set_initial(wp4, 0.98 * pmax)
        if Tf is None:
            opti.set_initial(T, Tfmax / 2)
        # SOLVER
        p_opts = {"expand": True}
        s_opts = {"max_iter": 1000}
        opti.solver("ipopt", p_opts,
                    s_opts)

        sol = opti.solve()

        # evaluate
        Tsol = sol.value(T)
        wpsol = sol.value(wp)
        wvsol = sol.value(wv)
        wasol = sol.value(wa)

        # plot bezier
        time, pz = self.bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
        vz = self.bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=0.002)[1]
        az = self.bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=0.002)[1]

        return time, pz, vz, az

    def set_init(self, state_x0, state_y0, state_z0):
        self.state_x0 = state_x0.copy()
        self.state_y0 = state_y0.copy()
        self.state_z0 = state_z0.copy()


    def plot_ref(self, title=None):
        fig, axs = plt.subplots(3, 1, figsize=(10, 5))
        if title is not None:
            fig.suptitle(title)

        axs[0].set_ylabel("$c$ [$m$]")
        axs[0].plot(self.time, self.T_p_com_ref.T)
        axs[0].grid()

        axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
        axs[1].plot(self.time, self.T_v_com_ref.T)
        axs[1].grid()

        axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
        axs[2].plot(self.time, self.T_a_com_ref.T)
        axs[2].grid()

        plt.show()

        legend = [Line2D([0], [0], color=axs[0].lines[0].get_color(), lw=4, label="x"),
                  Line2D([0], [0], color=axs[0].lines[1].get_color(), lw=4, label="y"),
                  Line2D([0], [0], color=axs[0].lines[2].get_color(), lw=4, label="z")]

        fig.legend(handles=legend,
                   loc="center right",  # Position of legend
                   borderaxespad=1,  # Small spacing around legend box
                   )

        plt.subplots_adjust(right=0.85)
        return fig
        
    def plot_3D(self, title=None, uxlim=None, uylim=None):
        fig = plt.figure()
        if title is not None:
            fig.suptitle(title)
        ax = fig.add_subplot(projection='3d')
        plt.plot(self.T_p_com_ref[0], self.T_p_com_ref[1], self.T_p_com_ref[2])

        ax.set_zlim([0., 0.35])
        ax.set_xlabel("$c^{x}$ [$m$]")
        ax.set_ylabel("$c^{y}$ [$m$]")
        ax.set_zlabel("$c^{z}$ [$m$]")

        ax.scatter([self.zmp_xy[0]], [self.zmp_xy[1]], [[0.]], color='red')

        if uxlim is not None and uylim is not None:
            u_lims = np.array([[uxlim[0], uylim[0], 0.],
                               [uxlim[0], uylim[1], 0.],
                               [uxlim[1], uylim[1], 0.],
                               [uxlim[1], uylim[0], 0.],
                               [uxlim[0], uylim[0], 0.]])

            plt.plot(u_lims[:, 0], u_lims[:, 1], u_lims[:, 2])

        plt.legend(["$c$", "com lims"])

        return fig
        
    def plot_compare(self, other):

        fig, axs = plt.subplots(3, 1, figsize=(10, 5))

        fig.suptitle("Compare strategies")

        axs[0].set_ylabel("c [$m$]")
        axs[0].plot(self.time, self.T_p_com_ref[0].T)
        axs[0].plot(other.time, other.T_p_com_ref[0].T, color=axs[0].lines[-1].get_color(), linestyle='--')
        axs[0].plot(self.time, self.T_p_com_ref[1].T)
        axs[0].plot(other.time, other.T_p_com_ref[1].T, color=axs[0].lines[-1].get_color(), linestyle='--')
        axs[0].plot(self.time, self.T_p_com_ref[2].T)
        axs[0].plot(other.time, other.T_p_com_ref[2].T, color=axs[0].lines[-1].get_color(), linestyle='--')
        axs[0].grid()

        axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
        axs[1].plot(self.time, self.T_v_com_ref[0].T)
        axs[1].plot(other.time, other.T_v_com_ref[0].T, color=axs[1].lines[-1].get_color(), linestyle='--')
        axs[1].plot(self.time, self.T_v_com_ref[1].T)
        axs[1].plot(other.time, other.T_v_com_ref[1].T, color=axs[1].lines[-1].get_color(), linestyle='--')
        axs[1].plot(self.time, self.T_v_com_ref[2].T)
        axs[1].plot(other.time, other.T_v_com_ref[2].T, color=axs[1].lines[-1].get_color(), linestyle='--')
        axs[1].grid()

        axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
        axs[2].plot(self.time, self.T_a_com_ref[0].T)
        axs[2].plot(other.time, other.T_a_com_ref[0].T, color=axs[2].lines[-1].get_color(), linestyle='--')
        axs[2].plot(self.time, self.T_a_com_ref[1].T)
        axs[2].plot(other.time, other.T_a_com_ref[1].T, color=axs[2].lines[-1].get_color(), linestyle='--')
        axs[2].plot(self.time, self.T_a_com_ref[2].T)
        axs[2].plot(other.time, other.T_a_com_ref[2].T, color=axs[2].lines[-1].get_color(), linestyle='--')
        axs[2].grid()

        legend = [Line2D([0], [0], color=axs[0].lines[0].get_color(), lw=axs[0].lines[0].get_lw(),
                         linestyle=axs[0].lines[0].get_linestyle(), label="x (s1)"),
                  Line2D([0], [0], color=axs[0].lines[1].get_color(), lw=axs[0].lines[1].get_lw(),
                         linestyle=axs[0].lines[1].get_linestyle(), label="x (s2)"),
                  Line2D([0], [0], color=axs[0].lines[2].get_color(), lw=axs[0].lines[2].get_lw(),
                         linestyle=axs[0].lines[2].get_linestyle(), label="y (s1)"),
                  Line2D([0], [0], color=axs[0].lines[3].get_color(), lw=axs[0].lines[3].get_lw(),
                         linestyle=axs[0].lines[3].get_linestyle(), label="y (s2)"),
                  Line2D([0], [0], color=axs[0].lines[4].get_color(), lw=axs[0].lines[4].get_lw(),
                         linestyle=axs[0].lines[4].get_linestyle(), label="z (s1)"),
                  Line2D([0], [0], color=axs[0].lines[5].get_color(), lw=axs[0].lines[5].get_lw(),
                         linestyle=axs[0].lines[5].get_linestyle(), label="z (s2)")]

        fig.legend(handles=legend,
                   loc="center right",  # Position of legend
                   borderaxespad=1,  # Small spacing around legend box
                   )

        plt.subplots_adjust(right=0.85)

        return fig
        

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from mpl_toolkits.mplot3d import axes3d

    m = 12.
    L = 0.25
    max_spring_compression = 0.1
    max_settling_time = 1.2

    state_x0 = np.array([[4],
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
    vhsip.solve_ocp(vx_f=0.5, vy_f=0)
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


    ax = plt.figure().add_subplot(projection='3d')
    plt.plot(vhsip.T_p_com_ref[0], vhsip.T_p_com_ref[1], vhsip.T_p_com_ref[2])
    ax.set_zlim([0., 0.35])
    ax.set_xlabel("$c^{x}$ [$m$]")
    ax.set_ylabel("$c^{y}$ [$m$]")
    ax.set_zlabel("$c^{z}$ [$m$]")

    ax.scatter([vhsip.zmp_xy[0]], [vhsip.zmp_xy[1]], [[0.]], color='red')

    plt.legend(["$c$", "$u_{o}$"])
