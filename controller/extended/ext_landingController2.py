import os
import sys

import casadi as ca
import numpy as np
import pinocchio as pin

from landing_controller.controller.landingController import LandingController
from .vhsip_dyn import VHSIP
from .support_polygon import supportPolygon
from .feasibility import *
from .utils import *

class ExtendedLandingController(LandingController):
    def __init__(self, robot, dt, q0, g_mag=9.81, smoothing_param=0.005, naive=False, gain_hrv=0.8):
        super().__init__(robot, dt, q0, g_mag, smoothing_param, naive)

        feet_xy = np.vstack(self.T_feet_task)[:, :2]

        self.supportPolygon = supportPolygon(feet_xy)#, margin=0.05)
        self.supportPolygonMargin = supportPolygon(feet_xy, margin=0.01)
        filename = os.environ['LOCOSIM_DIR'] + '/landing_controller/controller/extended/go1_kin_region.mat'
        self.kinematic_region = Feasibility(filename, "KINEMATICS_AND_FRICTION")
        self.kinematic_slice_l0 = FeasibilityOnSlice(self.kinematic_region, self.L)

        lookup_table_forces_filename = os.environ['LOCOSIM_DIR'] + \
                                       '/landing_controller/controller/lookup_table_forces.mat'
        data = loadmat(lookup_table_forces_filename)
        self.lookup_table_forces = data['lookup_table_forces']

        self.zmp_xy = np.empty(2) * np.nan
        self.projected_zmp = np.empty(2) * np.nan
        self.zmp_offset = np.empty([3,1])*np.nan
        self.zmp_offset[2] = 0

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

        self.has_VHSIP_called = False

        self.VHSIP = VHSIP(self.dt,
                           self.L,
                           self.max_spring_compression,
                           self.m,
                           self.g_mag,
                           self.w_v, self.w_p, self.w_u,
                           self.max_settling_time)


        self.gain_hrv = gain_hrv
        self.warm_start_solution = np.zeros(6)
        self.ELC_calls = 0
        self.pmin = 0.1
        self.pmax = 0.3

        self.use_lc = True


    def is_ZMPfeasible(self, zmp = None):
        if zmp is None:
            return self.kinematic_slice_l0.checkPointFeasibility(-self.zmp_xy)
        else:
            return self.kinematic_slice_l0.checkPointFeasibility(-zmp)


    def is_COMtrajFeasible(self, max_check=None):
        if max_check is None:
            max_check = self.VHSIP.ctrl_horz - 1
        for ii, elem in enumerate(self.VHSIP.T_p_com_ref.T):
            if ii <= max_check:
                if not self.kinematic_region.checkPointFeasibility(elem.reshape(3, 1) - self.zmp_offset):
                    return False
        return True

    def projectPointFR(self, point):
        return projectPointToPolygon(self.kinematic_slice_l0.A, self.kinematic_slice_l0.b, point)

    def bezier_z_dynamicsN(self, p0, v0, amax, pmin, pmax, Tfmax=None, vf=None, X0=None, N=None):
        if N is None:
            N = 3
        X = []

        wp_list = []

        # optimization variables: bezier coefficient, time
        T = ca.SX.sym('T', 1)
        X.append(T)

        # wp0 = opti.variable(1) no need of this
        # in position we have N+1 coeffs
        # in vel we have N coeffs ...
        wp0 = p0
        wp_list.append(wp0)
        for i in range(1, N):
            X.append(ca.SX.sym('wp' + str(i), 1))
            wp_list.append(X[-1])


        wpN = ca.SX.sym('wp' + str(N), 1)
        X.append(wpN)
        wp_list.append(X[-1])

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
        cost = 1  # -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

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
        constrs.append(T)
        if Tfmax is None:
            ubconstrs.append(+np.inf)
            lbconstrs.append(0.)
        else:
            ubconstrs.append(1. * Tfmax)
            lbconstrs.append(0. * Tfmax)

        for tt in self.VHSIP.ctrl_knots:
            # the acceleration is postive and upper bounded
            constrs.append(bezier(wa(X) / T ** 2, tt))
            ubconstrs.append(amax)
            # lbconstrs.append(-self.g_mag)
            lbconstrs.append(0.0)

            # bounds on position
            constrs.append(bezier(wp(X), tt))
            ubconstrs.append(pmax)
            if tt == self.VHSIP.ctrl_knots[-1]:
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

        # negative initial jerk
        constrs.append(wj_list[0])
        ubconstrs.append(0.0)
        lbconstrs.append(-np.inf)

        # SOLVER
        nlp = {'x': X, 'f': cost, 'g': ca.vcat(constrs)}
        # opts = {'knitro.print_level': 0, 'print_time': 0}
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

        time, pz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=self.dt)
        vz = bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=self.dt)[1]
        az = bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=self.dt)[1]

        return time, pz, vz, az, np.array(r['x']).tolist()

    def pushing_phase(self): # it does not change
        self.ref_k = 0

    def flyingUp_phase(self, B_R_T): # it does not change
        # kinematic adjustment: drive feet to create a plane parallel to landing surface and located at distance
        # self.L from the base
        # i.e., move feet to the home position according to the rotation of the base

        # at the beginning, B-frame and T-frame have the same orientation, thus at this stage we only have to rotate the
        # feet position expressed in base frame
        for leg in range(4):
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)


    def elc_solve(self, com_vel_T_TD):
        # for now, let assume we know in advance the TD velocity (I want to avoid useless recomputations for ELC-core)
        self.init_vel[0] = com_vel_T_TD[0]
        self.init_vel[1] = com_vel_T_TD[1]
        self.init_vel[2] = com_vel_T_TD[2]

        self.VHSIP.set_init(self.init_vel)
        time, posz, velz, accz = self.VHSIP.MSD_z_dynamics()
        self.VHSIP.set_z_dynamics(time, posz, velz, accz)
        self.VHSIP.propagation_matrices()
        self.VHSIP.solve_ocp(vx_f=0, vy_f=0)
        self.VHSIP.compute_xy_dynamics(self.VHSIP.zmp_xy[0], self.VHSIP.zmp_xy[1])

        solved = self.is_ZMPfeasible()
        if not solved:
            self.projected_zmp = -self.projectPointFR(-self.VHSIP.zmp_xy)
        else:
            self.projected_zmp = self.VHSIP.zmp_xy.copy()

        self.zmp_offset[0,0] = self.projected_zmp[0]
        self.zmp_offset[1,0] = self.projected_zmp[1]

        # t_hvr, i_hvr = self.VHSIP.horz_velocity_reduction(self.gain_hrv)
        i_hvr = self.VHSIP.ctrl_horz
        t_hvr = self.dt * i_hvr

        plot_ref(self.VHSIP.time, self.VHSIP.T_p_com_ref, self.VHSIP.T_v_com_ref, self.VHSIP.T_a_com_ref,
                 self.VHSIP.zmp_xy, self.projected_zmp, title=str(self.ELC_calls)+str(t_hvr), t_star=t_hvr)

        self.warm_start_solution[0] = t_hvr

        while not solved:
            self.ELC_calls += 1
            Fzmax = 600

            time, posz, velz, accz, X0 = self.bezier_z_dynamicsN(p0=self.L,
                                                                 v0=self.init_vel[2],
                                                                 amax=Fzmax / self.m,
                                                                 pmin=self.pmin,
                                                                 pmax=self.pmax,
                                                                 Tfmax=t_hvr,
                                                                 vf=3,
                                                                 X0=self.warm_start_solution,
                                                                 N=5)

            self.VHSIP.set_z_dynamics(time, posz, velz, accz)
            self.VHSIP.propagation_matrices()
            self.VHSIP.compute_xy_dynamics(self.VHSIP.zmp_xy[0], self.VHSIP.zmp_xy[1])

            i_hvr = self.VHSIP.ctrl_horz - 1

            # check the whole com traj is feasible
            check_com = self.is_COMtrajFeasible( i_hvr )

            # check velocity reduction
            check_fin_vel = np.all(self.VHSIP.T_v_com_ref[:2, i_hvr] <= 0.8 * self.VHSIP.T_v_com_ref[:2, 0])

            print('check_com:', check_com, 'check_fin_vel:', check_fin_vel)
            if check_com and check_fin_vel:
                solved = True

            # is it possible to solve the problem simply by cutting the refecence?
            else:
                t_hvr, i_hvr = self.VHSIP.horz_velocity_reduction(self.gain_hrv)
                solved = self.is_COMtrajFeasible(i_hvr)

                # print('vel reduction -> solved:', solved, 't_hvr:', t_hvr, 't_opt:', self.VHSIP.ctrl_horz * self.VHSIP.dt)

            plot_ref(self.VHSIP.time, self.VHSIP.T_p_com_ref, self.VHSIP.T_v_com_ref, self.VHSIP.T_a_com_ref,
                     self.VHSIP.zmp_xy, self.projected_zmp, title=str(self.ELC_calls), t_star=t_hvr)

            self.warm_start_solution = X0
            self.VHSIP.ctrl_horz = i_hvr + 1
            if self.ELC_calls == 5:
                print('Limit loops reached')
                break
        return solved



    def flyingDown_phase(self, B_R_T, com_vel_T):
        # for now, let assume we know in advance the TD velocity
        # (re-)compute zmp and move feet plane accordingly
        # self.init_pos[0] = 0.
        # self.init_pos[1] = 0.
        # self.init_pos[2] = self.L
        if self.use_lc:
            self.init_vel[0] = com_vel_T[0]
            self.init_vel[1] = com_vel_T[1]
            self.init_vel[2] = com_vel_T[2]

            self.VHSIP.set_init(self.init_vel)
            time, posz, velz, accz = self.VHSIP.MSD_z_dynamics()
            self.VHSIP.set_z_dynamics(time, posz, velz, accz)
            self.VHSIP.propagation_matrices()
            self.VHSIP.solve_ocp(vx_f=0, vy_f=0)
            self.VHSIP.compute_xy_dynamics(self.VHSIP.zmp_xy[0], self.VHSIP.zmp_xy[1])

            # task for feet in terrain frame
            if self.alpha < 1.:
                self.alpha = np.around(self.alpha+self.smoothing_param, 3)
            else:
                self.alpha = 1.


            self.zmp_xy = self.alpha * self.VHSIP.zmp_xy

            solved = self.is_ZMPfeasible(self.zmp_xy)  # = it is possible to put the feet's centroid on the computed zmp
            if not solved:
                self.projected_zmp = -self.projectPointFR(-self.zmp_xy)
                self.zmp_offset[0, 0] = self.projected_zmp[0]
                self.zmp_offset[1, 0] = self.projected_zmp[1]
                self.use_lc =self.supportPolygon.contains(self.zmp_xy-self.T_centroid)
                #self.use_lc =self.kinematic_slice_l0.checkPointFeasibility(self.zmp_xy-self.T_centroid)


            print('solved:',solved,
                  '\nuse_lc:', self.use_lc,
                  '\nzmp:', self.zmp_xy,
                  '\nVHSIP.zmp:', self.VHSIP.zmp_xy,
                  '\nprojected_zmp:',  self.projected_zmp,
                  '\nzmp_xy - projected_zmp', self.zmp_xy - self.projected_zmp,
                  '\nT_centroid', self.T_centroid,
                  '\nzmp_xy - T_centroid', self.zmp_xy - self.T_centroid,
                  '\nsupport polygon', self.supportPolygon.points,
                  '\nsupportPolygon.contains(self.zmp_xy - self.T_centroid)', self.supportPolygon.contains(self.zmp_xy-self.T_centroid),
                  '\nfeasible', self.kinematic_slice_l0.checkPointFeasibility(self.zmp_xy-self.T_centroid)
                  )

            for leg in range(4):
                if solved and self.kinematic_slice_l0.checkPointFeasibility(self.zmp_xy-self.T_centroid):
                    self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.zmp_xy[0]
                    self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.zmp_xy[1]
                    self.T_centroid = self.zmp_xy.copy()
                self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)


    def landedELC(self, T_comPose, T_comTwist, use_lc=True):
        self.T_comPose_TD = T_comPose.copy()
        self.T_comTwist_TD = T_comTwist.copy()
        if not use_lc:
            sol = self.elc_solve(T_comTwist[:3])
            if not sol:
                print('problem is unsolvable')
                #sys.exit(-1)

        self.eig_ang = self.VHSIP.eig_z

        self.pose_des[3:5] = self.T_comPose_TD[3:5]
        self.pose_des[5] = 0.
        self.twist_des[3:] = self.T_comTwist_TD[3:]

        self.MAT_ANG = np.eye(2) + np.array(
            [[- self.VHSIP.d / self.VHSIP.m, -self.VHSIP.k / self.VHSIP.m],
             [1, 0]]) * self.dt

        self.Rdyn = np.vstack([self.twist_des[3], self.pose_des[3]])
        self.Pdyn = np.vstack([self.twist_des[4], self.pose_des[4]])
        self.Ydyn = np.vstack([self.twist_des[5], self.pose_des[5]])



    def landed_phaseELC(self, t, use_lc):
        self.ref_k += 1

        self.Rdyn = self.MAT_ANG @ self.Rdyn
        self.Pdyn = self.MAT_ANG @ self.Pdyn
        self.Ydyn = self.MAT_ANG @ self.Ydyn
        # ---> POSE
        # to avoid reshape, use these three lines
        self.pose_des[0] = self.T_comPose_TD[0] + self.VHSIP.T_p_com_ref[0, self.ref_k]
        self.pose_des[1] = self.T_comPose_TD[1] + self.VHSIP.T_p_com_ref[1, self.ref_k]
        self.pose_des[2] = self.VHSIP.T_p_com_ref[2, self.ref_k]

        self.pose_des[3] = self.Rdyn[1]
        self.pose_des[4] = self.Pdyn[1]
        self.pose_des[5] = self.Ydyn[1] + self.T_comPose_TD[5]

        B_R_T_des = pin.rpy.rpyToMatrix(self.pose_des[3], self.pose_des[4], 0).T

        # ---> TWIST
        self.twist_des[0] = self.VHSIP.T_v_com_ref[0, self.ref_k]
        self.twist_des[1] = self.VHSIP.T_v_com_ref[1, self.ref_k]
        self.twist_des[2] = self.VHSIP.T_v_com_ref[2, self.ref_k]

        self.twist_des[3] = self.Rdyn[0]
        self.twist_des[4] = self.Pdyn[0]
        self.twist_des[5] = self.Ydyn[0]

        # ---> ACCELERATION
        self.acc_des[0] = self.VHSIP.T_a_com_ref[0, self.ref_k]
        self.acc_des[1] = self.VHSIP.T_a_com_ref[1, self.ref_k]
        self.acc_des[2] = self.VHSIP.T_a_com_ref[2, self.ref_k]

        Dm = self.VHSIP.d / self.VHSIP.m
        Km = self.VHSIP.k / self.VHSIP.m
        self.acc_des[3] = -Dm * self.Rdyn[0] - Km * self.Rdyn[1]
        self.acc_des[4] = -Dm * self.Pdyn[0] - Km * self.Pdyn[1]
        self.acc_des[5] = -Dm * self.Ydyn[0] - Km * self.Ydyn[1]

        self.T_o_B[2] = self.pose_des[2]

        for leg in range(4):
            if use_lc:
                self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.zmp_xy[0]
                self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.zmp_xy[1]
            else:
                self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.projected_zmp[0]
                self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.projected_zmp[1]
            self.B_feet_task[leg] = B_R_T_des @ (
                        self.T_feet_task[leg] - self.VHSIP.T_p_com_ref[:, self.ref_k])



    def expected_td_time(self, z0, zdot0, zf):
        # t^2 - 2 * zdot0 / g  * t - 2/g * (z0-zf) == 0
        b = - 2 * zdot0 / self.g_mag
        c = - 2 * (z0-zf) / self.g_mag
        delta = b**2 - 4 * c
        if delta < -10**4:
            print('Huston, we have a problem')
        else:
            delta = max(delta, 0.)
        return (-b + np.sqrt(delta))/2