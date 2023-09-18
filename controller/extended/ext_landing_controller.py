import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
import os
import casadi as ca

from scipy.optimize import linprog

from .feasibility import *
from .utils import *

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)


class ExtendedLandingController:
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

        self.init_velH = np.empty(2) * np.nan

        self.Cv = np.array([[1.0, 0.0]])
        self.Cp = np.array([[0.0, 1.0]])

        self.Q_xy = np.empty([2, 2]) * np.nan
        self.Q_xyu = np.empty([1, 2]) * np.nan
        self.Q_u = np.empty([1, 1]) * np.nan

        self.Q_vx = np.empty([1, 2]) * np.nan
        self.Q_vu = np.empty([1, 1]) * np.nan

        self.w_v = w_v
        self.w_p = w_p
        self.w_u = w_u

        filename = os.environ['LOCOSIM_DIR'] + '/landing_controller/controller/extended/go1_kin_region.mat'
        self.feasibility = Feasibility(filename, "KINEMATICS_AND_FRICTION")
        self.feasibility_l0 = FeasibilityOnSlice(self.feasibility, self.L)

        lookup_table_forces_filename = os.environ[
                                           'LOCOSIM_DIR'] + '/landing_controller/controller/lookup_table_forces.mat'
        data = loadmat(lookup_table_forces_filename)
        self.lookup_table_forces = data['lookup_table_forces']

        self.zmp_xy = np.empty(2) * np.nan
        self.projected_zmp = np.empty(2) * np.nan

        self.ctrl_points = np.linspace(0, 1, num=6, endpoint=True)
        self.ctrl_indexes = np.zeros_like(self.ctrl_points)

    def duplicate(self):
        return ExtendedLandingController(self.L, self.dt, self.g_mag, self.w_v, self.w_p, self.w_u)

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

        self.ctrl_indexes = (self.ctrl_points * self.ctrl_horz).astype(int)

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
        settling_time = -8 / eig_z

        time = np.arange(0, settling_time, self.dt)
        ctrl_horz = time.shape[0]

        pz = np.zeros([1, ctrl_horz])
        vz = np.zeros([1, ctrl_horz])
        az = np.zeros([1, ctrl_horz])

        for ii in range(ctrl_horz):
            t = time[ii]
            exp_eig_t = np.exp(eig_z * t)

            pz[:, ii] = exp_eig_t * t * self.state_z0[0, 0] + self.L
            vz[:, ii] = exp_eig_t * (eig_z * t + 1) * self.state_z0[0, 0]
            az[:, ii] = -d / m * vz[:, ii] - k / m * (pz[:, ii] - self.L)

        return time, pz, vz, az

    def bezier_z_dynamicsN(self, p0, v0, amax, pmin, pmax, Tfmax=None, Tf=None, pf=None, vf=None, plot=False, Y0=None,
                           N=None):

        if N is None:
            N = 3

        X = []
        X0 = []

        wp_list = []

        # optimization variables: bezier coefficient, time
        if Tf is None:
            T = ca.SX.sym('T', 1)
            X.append(T)
            if Tfmax is not None:
                X0.append(Tfmax)
            else:
                X0.append(0.5)
        else:
            T = Tf

        # wp0 = opti.variable(1) no need of this
        wp0 = p0
        wp_list.append(wp0)
        for i in range(1, N):
            X.append(ca.SX.sym('wp' + str(i), 1))
            wp_list.append(X[-1])
            X0.append(0.0)

        if pf is None or not (pmin < pf < pmax):  # if pf is given, set w4=pf
            wpN = ca.SX.sym('wp' + str(N), 1)
            X.append(wpN)
            wp_list.append(X[-1])
            X0.append(0.98 * pmax)
        else:
            wpN = pf
            wp_list.append(wpN)

        # opt variables
        X = ca.vcat(X)

        # for simplify computations, let's define coeffs of bezier derivatives (do not consider time)
        wp = ca.Function('wp', [X], [ca.vcat(wp_list)])

        wv_list = []
        for i in range(N):
            wv_list.append(N * (wp_list[i + 1] - wp_list[i]))
        wv = ca.Function('wv', [X], [ca.vcat(wv_list)])

        wa_list = []
        for i in range(N - 1):
            wa_list.append((N - 1) * (wv_list[i + 1] - wv_list[i]))
        wa = ca.Function('wa', [X], [ca.vcat(wa_list)])

        # COST
        cost = -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

        # CONSTRAINTS
        constrs = []
        ubconstrs = []
        lbconstrs = []
        # initial position
        # opti.subject_to(wp0==p0)
        #

        # initial velocity
        init_vel = wv_list[0] / T
        constrs.append(init_vel - v0)
        ubconstrs.append(0.0)
        lbconstrs.append(0.0)

        # causality, limited time
        if Tf is None:
            constrs.append(T)
            if Tfmax is None:
                ubconstrs.append(+np.inf)
                lbconstrs.append(0.)
            else:
                ubconstrs.append(1.2 * Tfmax)
                lbconstrs.append(0.8 * Tfmax)

        for tau in self.ctrl_points:
            # the acceleration is postive and upper bounded
            constrs.append(bezier(wa(X) / T ** 2, tau))
            ubconstrs.append(amax)
            # lbconstrs.append(-self.g_mag)
            lbconstrs.append(0.0)

            # bounds on position
            constrs.append(bezier(wp(X), tau))
            ubconstrs.append(pmax)
            if tau == self.ctrl_points[-1]:
                lbconstrs.append((pmax + p0) / 2)  # does not allow for squatty jumps
            else:
                lbconstrs.append(pmin)

        if vf is None:
            # positive final velocity
            constrs.append(wv_list[N - 1])
            ubconstrs.append(np.inf)
            lbconstrs.append(0.0)
        else:
            # if vf is given, equality constaint
            constrs.append(wv_list[N - 1] / T - vf)
            ubconstrs.append(0.0)
            lbconstrs.append(0.0)

        # SOLVER
        nlp = {'x': X, 'f': cost, 'g': ca.vcat(constrs)}
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        S = ca.nlpsol('S', 'ipopt', nlp, opts)

        if Y0 is not None:
            X0 = Y0

        r = S(x0=ca.vcat(X0), lbg=ca.vcat(lbconstrs), ubg=ca.vcat(ubconstrs))

        # evaluate
        # X = [T, wp1, wp2, wp3, wp4]
        Tsol = r['x'][0]
        wpsol = wp(r['x'])
        wvsol = wv(r['x'])
        wasol = wa(r['x'])

        time, pz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
        vz = bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=0.002)[1]
        az = bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=0.002)[1]

        # plot bezier
        if plot:
            plotOptimalBezier(time, pz, vz, az, Tsol, pmin, pmax, p0, pf, v0, amax, self.ctrl_points)

        return time, pz, vz, az, np.array(r['x']).tolist()


    def set_init(self, state_x0, state_y0, state_z0):
        self.state_x0[0] = state_x0[0]
        self.state_y0[0] = state_y0[0]
        self.state_z0[0] = state_z0[0]

        self.state_x0[1] = 0.
        self.state_y0[1] = 0.
        self.state_z0[1] = 0.

        self.init_velH[0] = state_x0[0]
        self.init_velH[1] = state_y0[0]

    # this is faster than above
    def projectPointFR(self, point):
        return projectPointToPolygon(self.feasibility_l0.A, self.feasibility_l0.b, point)

    def index_ref_over_projected_zmp(self, strategy):
        # positive strategy means to check forward
        # negative means backward
        if strategy > 0:
            i = 0
            while i <= self.ctrl_horz - 1:
                if np.all(np.sign(self.T_v_com_ref[:2, 0]) * (self.T_p_com_ref[:2, i] - self.projected_zmp) <= 0):
                    i += 1
                else:
                    return i

        elif strategy < 0:
            i = self.ctrl_horz - 1
            while i > 0:
                if np.all(np.sign(self.T_v_com_ref[:2, 0]) * (self.T_p_com_ref[:2, i] - self.projected_zmp) >= 0):
                    i -= 1
                else:
                    return i

    def is_ZMPfeasible(self):
        return self.feasibility_l0.checkPointFeasibility(-self.zmp_xy)

    def is_COMtrajFeasible(self):
        for id in self.ctrl_indexes:
            if id == 0:
                continue
            if id == self.ctrl_indexes[-1]:
                id = -1
            offset = np.array([[self.projected_zmp[0]],
                               [self.projected_zmp[1]],
                               [0]])
            if not self.feasibility.checkPointFeasibility(self.T_p_com_ref[:, id].reshape(3, 1) - offset):
                return False
        return True

    def suboptimal_force(self, state_x0, state_y0):
        theta = np.arctan2(state_y0[0], state_x0[0])
        idx = (np.abs(self.lookup_table_forces[:, 0] - theta)).argmin()
        return self.lookup_table_forces[idx, 1]



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

    ELC = ExtendedLandingController(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)

    k_limit = m * (8 / max_settling_time) ** 2
    eig_z_limit = -np.sqrt(k_limit / m)

    vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

    if state_z0[0, 0] < vz_limit:
        k = m * (state_z0[0, 0] / (np.exp(1) * max_spring_compression)) ** 2
    else:
        k = k_limit
    d = 2 * np.sqrt(m * k)

    ELC.set_init(state_x0, state_y0, state_z0)
    time, pz, vz, az = ELC.MSD_z_dynamics(m, k, d)
    ELC.set_z_dynamics(time, pz, vz, az)
    ELC.solve_ocp(vx_f=0.5, vy_f=0)
    ELC.compute_xy_dynamics(ELC.zmp_xy[0], ELC.zmp_xy[1])

    fig, axs = plt.subplots(3, 1, figsize=(10, 5))

    axs[0].set_ylabel("c [$m$]")
    axs[0].plot(ELC.time, ELC.T_p_com_ref.T)
    axs[0].grid()

    axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
    axs[1].plot(ELC.time, ELC.T_v_com_ref.T)
    axs[1].grid()

    axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
    axs[2].plot(ELC.time, ELC.T_a_com_ref.T)
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
    plt.plot(ELC.T_p_com_ref[0], ELC.T_p_com_ref[1], ELC.T_p_com_ref[2])
    ax.set_zlim([0., 0.35])
    ax.set_xlabel("$c^{x}$ [$m$]")
    ax.set_ylabel("$c^{y}$ [$m$]")
    ax.set_zlabel("$c^{z}$ [$m$]")

    ax.scatter([ELC.zmp_xy[0]], [ELC.zmp_xy[1]], [[0.]], color='red')

    plt.legend(["$c$", "$u_{o}$"])



def ballisticTraj(state_x0, state_y0, state_z0, flight_time, dt, g_mag):
    N = int(flight_time / dt + 1)
    time = np.zeros([N])
    pos = np.zeros([3, N])
    vel = np.zeros([3, N])
    acc = np.zeros([3, N])
    for i in range(0, N):
        t = i * dt
        time[i] = t
        pos[0, i] = state_x0[1] + state_x0[0] * t
        pos[1, i] = state_y0[1] + state_y0[0] * t
        pos[2, i] = state_z0[1] + state_z0[0] * t - 0.5 * g_mag * t ** 2

        vel[0, i] = state_x0[0]
        vel[1, i] = state_y0[0]
        vel[2, i] = state_z0[0] - g_mag * t

        acc[2, i] = -g_mag

    return time, pos, vel, acc