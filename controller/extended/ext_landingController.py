import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
import os
import casadi as ca

from scipy.optimize import linprog
from .support_polygon import supportPolygon
from .feasibility import *
from .utils import *
from landing_controller.controller.landingController import LandingController

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)


class ExtendedLandingController:
    def __init__(self, L, dt=0.002, g_mag=9.81, w_v=1., w_p=1., w_u=1., feet=None):

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

        if feet is not None:
            self.supportPolygon = supportPolygon(feet[:, :2], margin=0.01)

        filename = os.environ['LOCOSIM_DIR'] + '/landing_controller/controller/extended/go1_kin_region.mat'
        self.kinematic_region = Feasibility(filename, "KINEMATICS_AND_FRICTION")
        self.kinematic_slice_l0 = FeasibilityOnSlice(self.kinematic_region, self.L)

        lookup_table_forces_filename = os.environ[
                                           'LOCOSIM_DIR'] + '/landing_controller/controller/lookup_table_forces.mat'
        data = loadmat(lookup_table_forces_filename)
        self.lookup_table_forces = data['lookup_table_forces']

        self.zmp_xy = np.empty(2) * np.nan
        self.projected_zmp = np.empty(2) * np.nan

        self.ctrl_knots = np.linspace(0, 1, num=6, endpoint=True)
        self.ctrl_indexes = np.zeros_like(self.ctrl_knots)

        self.constraints = [None] * 15
        self.constraints_ub = [None] * 15
        self.constraints_lb = [None] * 15

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol("solver", "ipopt", "./nlp.so", opts)
        self.ubconstrsFunction = ca.external("ubconstrs", "./nlp.so")
        self.lbconstrsFunction = ca.external("lbconstrs", "./nlp.so")
        self.wp = ca.external("wp", "./nlp.so")
        self.wv = ca.external("wv", "./nlp.so")
        self.wa = ca.external("wa", "./nlp.so")


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

        self.ctrl_indexes = (self.ctrl_knots * self.ctrl_horz).astype(int)

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

    def bezier_z_dynamics_cpp(self, p0, v0, amax, pmin, pmax, Tfmax, vf, X0):
        self.param = np.array([p0,
                         v0,
                         amax,
                         pmin,
                         pmax,
                         Tfmax,
                         vf])

        res = self.solver(
            ubg=self.ubconstrsFunction(self.param),
            lbg=self.lbconstrsFunction(self.param),
            x0=X0,
            p=self.param)

        Tsol = res['x'][0]
        wpsol = self.wp(p0, res['x'])
        wvsol = self.wv(p0, res['x'])
        wasol = self.wa(p0, res['x'])

        time, pz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
        vz = bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=0.002)[1]
        az = bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=0.002)[1]

        return time, pz, vz, az, np.array(res['x']).tolist()



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
        # in position we have N+1 coeffs
        # in vel we have N coeffs ...
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

        wj_list = []
        for i in range(N - 2):
            wj_list.append((N - 2) * (wa_list[i + 1] - wa_list[i]))
        wj = ca.Function('wj', [X], [ca.vcat(wj_list)])

        # COST
        cost = 1 # -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

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
                ubconstrs.append(1. * Tfmax)
                lbconstrs.append(0. * Tfmax)

        for tau in self.ctrl_knots:
            # the acceleration is postive and upper bounded
            constrs.append(bezier(wa(X) / T ** 2, tau))
            ubconstrs.append(amax)
            # lbconstrs.append(-self.g_mag)
            lbconstrs.append(0.0)

            # bounds on position
            constrs.append(bezier(wp(X), tau))
            ubconstrs.append(pmax)
            if tau == self.ctrl_knots[-1]:
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

        constrs.append(wj_list[0])
        ubconstrs.append(0.0)
        lbconstrs.append(-np.inf)

        # SOLVER
        nlp = {'x': X, 'f': cost, 'g': ca.vcat(constrs)}
        #opts = {'knitro.print_level': 0, 'print_time': 0}
        opts = {
            # "print_level": 8,
            "ipopt.linear_solver": "ma57",
            # "ma57_pivtol": 1e-6
            # "nlp_scaling_max_gradient": 100.0,
            # "nlp_scaling_min_value": 1e-6,
            # "tol": 1e-3,
            # "dual_inf_tol": 1000.0,
            # "compl_inf_tol": 1e-2,
            # "constr_viol_tol": 1e-3,
            # "acceptable_tol": 1e0,
            # "acceptable_iter": 1,
            # #"acceptable_compl_inf_tol": 1,
            # "alpha_for_y": "dual-and-full",
            # "max_iter": 4000,
            # "warm_start_bound_frac": 1e-2,
            # "warm_start_bound_push": 1e-2,
            # "warm_start_mult_bound_push": 1e-2,
            # "warm_start_slack_bound_frac": 1e-2,
            # "warm_start_slack_bound_push": 1e-2,
            # "warm_start_init_point": "yes",
            # "required_infeasibility_reduction": 0.8,
            # "perturb_dec_fact": 0.1,
            # "max_hessian_perturbation": 10000,
            # "fast_step_computation": "yes",
            # "hessian_approximation": "exact",
        }
        S = ca.nlpsol('S', 'ipopt', nlp, opts)

        if Y0 is not None:
            X0 = Y0

        r = S(x0=ca.vcat(X0), lbg=ca.vcat(lbconstrs), ubg=ca.vcat(ubconstrs))

        # constrF = ca.Function('constr', [X], [ca.vcat(constrs)])
        # print(lbconstrs)
        # print(constrF(r['x']))
        # print(ubconstrs)

        # evaluate
        # X = [T, wp1, wp2, wp3, wp4]
        Tsol = r['x'][0]
        wpsol = wp(r['x'])
        wvsol = wv(r['x'])
        wasol = wa(r['x'])
        wjsol = wj(r['x'])
        print(wjsol)

        time, pz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
        vz = bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=0.002)[1]
        az = bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=0.002)[1]

        # plot bezier
        if plot:
            plotOptimalBezier(time, pz, vz, az, Tsol, pmin, pmax, p0, pf, v0, amax, self.ctrl_knots)

        return time, pz, vz, az, np.array(r['x']).tolist()

    def bezier_z_dynamicsN_preset(self, p0, v0, amax, pmin, pmax, Tfmax=None, Tf=None, pf=None, vf=None, plot=False, Y0=None,
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
        # in position we have N+1 coeffs
        # in vel we have N coeffs ...
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
        cost = 1 # -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

        # CONSTRAINTS

        # initial velocity
        init_vel = wv_list[0] / T
        self.constraints[0] = init_vel-v0
        self.constraints_ub[0] = 0.0
        self.constraints_lb[0] = 0.0

        # causality, limited time
        self.constraints[1] = T
        self.constraints_ub[1] = Tfmax
        self.constraints_lb[1] = 0.0


        for ii, tau in enumerate(self.ctrl_knots):
            # the acceleration is postive and upper bounded
            idx = ii+2
            self.constraints[idx] = bezier(wa(X) / T ** 2, tau)
            self.constraints_ub[idx] = amax
            self.constraints_lb[idx] = 0.0

        for ii, tau in enumerate(self.ctrl_knots):
            # bounds on position
            idx = ii + self.ctrl_knots.shape[0] + 2
            self.constraints[idx] = bezier(wp(X), tau)
            self.constraints_ub[idx] = pmax
            if tau == self.ctrl_knots[-1]:
                self.constraints_lb[idx] = (pmax + p0) / 2  # does not allow for squatty jumps
            else:
                self.constraints_lb[idx] = pmin

        last_idx = 2 * self.ctrl_knots.shape[0] + 2

        if vf is None:
            # positive final velocity
            self.constraints[last_idx] = wv_list[N - 1]
            self.constraints_ub[last_idx] = np.inf
            self.constraints_lb[last_idx] = 0.0
        else:
            # if vf is given, equality constaint
            self.constraints[last_idx] = wv_list[N - 1]/T - vf
            self.constraints_ub[last_idx] = 0.0
            self.constraints_lb[last_idx] = 0.0

        # SOLVER
        nlp = {'x': X, 'f': cost, 'g': ca.vcat(self.constraints)}
        opts = {'knitro.print_level': 0, 'print_time': 0}
        S = ca.nlpsol('S', 'knitro', nlp)#, opts)

        if Y0 is not None:
            X0 = Y0

        r = S(x0=ca.vcat(X0), lbg=ca.vcat(self.constraints_lb), ubg=ca.vcat(self.constraints_ub))

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
            plotOptimalBezier(time, pz, vz, az, Tsol, pmin, pmax, p0, pf, v0, amax, self.ctrl_knots)

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


    def projectPointFR(self, point):
        return projectPointToPolygon(self.kinematic_slice_l0.A, self.kinematic_slice_l0.b, point)

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

    def is_ZMPfeasible(self, zmp_xy):
        return self.kinematic_slice_l0.checkPointFeasibility(-zmp_xy)


    def is_COMFeasible(self, i):
        offset = np.array([[self.projected_zmp[0]],
                           [self.projected_zmp[1]],
                           [0]])
        return self.kinematic_region.checkPointFeasibility(self.T_p_com_ref[:, i].reshape(3, 1) - offset)

    def is_COMtrajFeasible(self, i=None):
        for id in self.ctrl_indexes:
            if id == 0:
                continue
            if id == self.ctrl_indexes[-1]:
                if i is None:
                    id = -1
                else:
                    id = i

            offset = np.array([[self.projected_zmp[0]],
                               [self.projected_zmp[1]],
                               [0]])
            if not self.kinematic_region.checkPointFeasibility(self.T_p_com_ref[:, id].reshape(3, 1) - offset):
                return False
        return True

    def suboptimal_force(self, state_x0, state_y0):
        theta = np.arctan2(state_y0[0], state_x0[0])
        idx = (np.abs(self.lookup_table_forces[:, 0] - theta)).argmin()
        return self.lookup_table_forces[idx, 1]


    def ELC_loop(self, t_star, max_iter=10):
        X0 = np.zeros(6)
        X0[0] = t_star
        solved = False
        iter_count = 0
        while not solved or iter_count<max_iter:
            # time, posz, velz, accz, X0 = self.bezier_z_dynamicsN(p0=L, v0=state_z0[0], amax=Fzmax/m, pmin=pmin, pmax=pmax, Tfmax=t_star, Tf=None, pf=None, vf=2, Y0=X0, N=5)

            time, posz, velz, accz, X0 = self.bezier_z_dynamics_cpp(p0=self.L, v0=self.init_vel[2], amax=self.amax,
                                                                   pmin=self.pmin, pmax=self.pmax,
                                                                   Tfmax=t_star, vf=2, X0=np.array(X0))

            self.set_z_dynamics(time, posz, velz, accz)
            self.propagation_matrices()
            self.compute_xy_dynamics(self.projected_zmp[0], self.projected_zmp[1])

            t_star, i = self.compute_instant_of_velocity_reduction(0.8)
            solved = self.is_COMtrajFeasible(i)
            iter_count += 1


    def compute_instant_of_velocity_reduction(self, gain):
        i = self.ctrl_horz - 1
        while i >= 0:
            if np.all(self.T_v_com_ref[:2, i] >= gain * self.init_velH):
                i -= 1
            else:
                break
        t_star = i * self.dt

        return t_star, i





    # overload from landing controller: the function changes when ELC is activated
    def flyingDown_phase(self, B_R_T, com_vel_T):
        self.state_x0[0] = com_vel_T[0]
        self.state_y0[0] = com_vel_T[1]
        self.state_z0[0] = com_vel_T[2]

        self.state_x0[1] = 0.
        self.state_y0[1] = 0.
        self.state_z0[1] = 0.

        self.init_velH[0] = com_vel_T[0]
        self.init_velH[1] = com_vel_T[1]

        if self.useLC:
            # msd dynamics
            k_limit = m * (8 / max_settling_time) ** 2
            eig_z_limit = -np.sqrt(k_limit / m)
            vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

            if state_z0[0, 0] < vz_limit:
                k = m * (state_z0[0, 0] / (np.exp(1) * max_spring_compression)) ** 2
            else:
                k = k_limit
            d = 2 * np.sqrt(m * k)

            time, posz, velz, accz = self.MSD_z_dynamics(m, k, d)
            self.set_z_dynamics(time, posz, velz, accz)
            self.propagation_matrices()
            self.solve_ocp(vx_f=0, vy_f=0)
            self.useLC = self.is_ZMPfeasible()

            if not self.useLC:
                self.projected_zmp = -self.projectPointFR(-self.zmp_xy)
            else:
                self.projected_zmp = self.zmp_xy.copy()
            self.compute_xy_dynamics(self.projected_zmp[0], self.projected_zmp[1])

        else:
            t_star, _ = self.compute_instant_of_velocity_reduction(0.8)
            self.ELC_loop(t_star)

        # task for feet in terrain frame
        if self.alpha < 1.:
            self.alpha = np.around(self.alpha+self.smoothing_param, 3)

        if self.naive:
            self.slip_dyn.zmp_xy[0] = 0.0
            self.slip_dyn.zmp_xy[1] = 0.0

        for leg in range(4):
            self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.alpha * self.projected_zmp[0]
            self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.alpha * self.projected_zmp[1]
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)


    # this function must be called when td is detected. it fills variables
    def landed(self, T_comPose, T_comTwist):
        self.T_comPose_TD = T_comPose.copy()
        self.T_comTwist_TD = T_comTwist.copy()

        self.pose_des[3:5] = self.T_comPose_TD[3:5]
        self.pose_des[5] = 0.
        self.twist_des[3:] = self.T_comTwist_TD[3:]
        if self.useLC:
            self.eig_ang = -np.sqrt(self.slip_dyn.K / self.slip_dyn.m)
        else:
            self.eig_ang = -self.t_star/8

        self.MAT_ANG = np.eye(2) + np.array([[2*self.eig_ang, self.eig_ang**2], [1, 0]]) * self.dt

        self.Rdyn = np.vstack([self.twist_des[3], self.pose_des[3]])
        self.Pdyn = np.vstack([self.twist_des[4], self.pose_des[4]])
        self.Ydyn = np.vstack([self.twist_des[5], self.pose_des[5]])


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