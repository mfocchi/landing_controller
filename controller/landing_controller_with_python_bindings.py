import numpy as np
import pinocchio as pin
from base_controllers.utils.utils import Utils
from base_controllers.components.leg_odometry.leg_odometry import LegOdometry
from base_controllers.components.inverse_kinematics.inv_kinematics_quadruped import InverseKinematics
from controller.SLIP_dynamics.SLIP_dynamics_lib import SLIP_dynamics
import tabulate

from matplotlib import pyplot as plt

###############################
# TIMINGS DESCRIPTION CLASSES #
###############################

class Event:
    def __init__(self, name):
        self.name = name
        self.t = 0.
        self.sample = -1

    def set(self, t, sample):
        self.t = t
        self.sample = sample

class JumpingDataTimes:
    def __init__(self):
        self.lift_off = Event('Lift off')
        self.apex = Event('Apex')
        self.touch_down = Event('Touch down')

        self._list_of_events = [self.lift_off, self.apex, self.touch_down]
        self._headers = ['Event', 'Time (s)', '# Sample']

    def toString(self):
        l = [[e.name, e.t, e.sample] for e in self._list_of_events]
        return tabulate.tabulate(l, headers=self._headers)


# In the class appears vectors and matrices describing pose and twist. Explanation:
# A_p     : position of a generic point expressed wrt frame A
# A_o_B   : position of the origin of frame B expressed wrt frame A
# A_R_B   : rotation of frame B wrt frame A
# A_v     : linear velocity of a generic point expressed wrt frame A
# A_v_B   : linear velocity of the origin of frame B wrt frame A
# A_omega : angular velocity of frame A

# The variables defining the state along a certain axis respect the following convention:
# stateX = [velocityX]
#          [positionX]

class LandingController:
    def __init__(self, robot, dt, q0, g_mag=9.81):
        self.u = Utils()
        
        self.robot = robot
        self.qj_home = q0[7:]
        self.v_home = pin.utils.zero(self.robot.nv)

        self.dt = dt


        self.ref_k = -1
        self.lp_counter = -1



        # checking times: setCheckTimings() should be called before run
        self.check_lift_off_time = 0.
        self.check_apex_time = 0.
        self.check_touch_down_time = 0.

        # MSD params
        self.g_mag = g_mag
        self.m = self.robot.robot_mass

        # these variables are useful for computations
        # Neutral: base expressed in base frame
        # q_neutral[0:7] and v_neutral[0:6] cannot be modified
        self._q_neutral = pin.neutral(self.robot.model)


        floor2base = self.base_height(q_j=self.u.mapToRos(self.qj_home))
        self.L = floor2base
        self.max_spring_compression = 0.5 * self.L
        w_v = 1.
        w_p = 1.
        w_u = 0.
        max_settling_time = 0.8

        self.slip_dyn = SLIP_dynamics(  self.dt,
                                        self.L,
                                        self.max_spring_compression,
                                        self.m,
                                        self.g_mag,
                                        w_v, w_p, w_u,
                                        max_settling_time)

        self.euler_TD = np.zeros(3)
        self.eig_ang = 0.

        self.pose_des = np.zeros(6)
        self.twist_des = np.zeros(6)
        self.acc_des = np.zeros(6)



        self.T_o_B = np.array([0, 0, self.L])

        self.B_feet_home = []
        self.T_feet_home = []

        self.B_feet_task = []
        self.T_feet_task = []

        self.legs = ['lf', 'rf', 'lh', 'rh']

        for leg in self.legs:
            name = leg+'_foot'
            B_foot = self.robot.data.oMf[self.robot.model.getFrameId(name)].translation
            self.B_feet_home.append(B_foot.copy())
            self.B_feet_task.append(B_foot.copy())

            T_foot = np.array([B_foot[0], B_foot[1], 0.0])
            self.T_feet_home.append(T_foot.copy())
            self.T_feet_task.append(T_foot.copy())


        self.init_pos = np.zeros([3, 1])
        self.init_vel = np.zeros([3, 1])


        self.alpha = 0.
        self.smoothing_param = 0.015

        self.euler_final = np.zeros(3)

        self.landed_phase_flag = False

        self.jumping_data_times = JumpingDataTimes()


    def base_height(self, q_j):
        # assumes all the feet in contact and contact surface horizontal
        self._q_neutral[7:] = q_j
        pin.forwardKinematics(self.robot.model, self.robot.data, self._q_neutral)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        floor2base = 0.
        for ee_id in self.robot.getEndEffectorsFrameId:
            floor2base -= self.robot.data.oMf[ee_id].translation[2].copy()
        return floor2base/4


    def pushing_phase(self):
        pass

    def flyingUp_phase(self, B_R_T):
        # kinematic adjustment: drive feet to create a plane parallel to landing surface and located at distance
        # self.L from the base
        # i.e., move feet to the home position according to the rotation of the base

        # at the beginning, B-frame and T-frame have the same orientation, thus at this stage we only have to rotate the
        # feet position expressed in base frame
        for leg in range(4):
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)




    def flyingDown_phase(self, B_R_T, com_vel_W):
        # (re-)compute zmp and move feet plane accordingly
        self.init_pos[0] = 0.
        self.init_pos[1] = 0.
        self.init_pos[2] = self.L
        self.init_vel[0] = com_vel_W[0]
        self.init_vel[1] = com_vel_W[1]
        self.init_vel[2] = com_vel_W[2]

        self.slip_dyn.def_and_solveOCP(self.init_pos, self.init_vel)


        # task for feet in terrain frame
        if self.alpha < 1.:
            self.alpha = np.min([np.around(self.alpha+self.smoothing_param, 4), 1.])


        for leg in range(4):
            self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.alpha * self.slip_dyn.zmp_xy[0]
            self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.alpha * self.slip_dyn.zmp_xy[1]
            self.B_feet_task[leg] = B_R_T @ (self.T_feet_task[leg] - self.T_o_B)



    def landed_phase(self, t, euler):
        # use the last trajectory computed
        # task for feet in terrain frame
        if not self.landed_phase_flag:
            self.slip_dyn.xy_dynamics()

            self.euler_TD = euler
            self.eig_ang = -2 * np.sqrt(self.slip_dyn.K / self.slip_dyn.m)
            self.landed_phase_flag = True

        # lp_counter counts how many times landed_phase has been called. the reference computed by slip_dyn.xy_dynamics()
        # we apply twice the reference. so ref_k is half of lp_counter, up to its max value
        self.lp_counter += 1
        if self.ref_k < self.slip_dyn.ctrl_horz-1:
            if self.lp_counter %2 == 0:
                self.ref_k += 1


        # REFERENCES
        # ---> POSE
        # to avoid reshape, use these three lines
        self.pose_des[0] = self.slip_dyn.T_p_com_ref[0, self.ref_k]
        self.pose_des[1] = self.slip_dyn.T_p_com_ref[1, self.ref_k]
        self.pose_des[2] = self.slip_dyn.T_p_com_ref[2, self.ref_k]

        self.pose_des[3:6] = np.exp(self.eig_ang * (t-self.jumping_data_times.touch_down.t)) * self.euler_TD

        B_R_T_des = pin.rpy.rpyToMatrix(self.pose_des[3:6]).T

        # ---> TWIST
        self.twist_des[0] = self.slip_dyn.T_v_com_ref[0, self.ref_k]
        self.twist_des[1] = self.slip_dyn.T_v_com_ref[1, self.ref_k]
        self.twist_des[2] = self.slip_dyn.T_v_com_ref[2, self.ref_k]

        self.twist_des[3:6] = self.eig_ang * self.pose_des[3:6]

        # ---> ACCELERATION
        self.acc_des[0] = self.slip_dyn.T_a_com_ref[0, self.ref_k]
        self.acc_des[1] = self.slip_dyn.T_a_com_ref[1, self.ref_k]
        self.acc_des[2] = self.slip_dyn.T_a_com_ref[2, self.ref_k]

        self.acc_des[3:6] = self.eig_ang * self.twist_des[3:6]


        self.T_o_B[2] = self.pose_des[2]
        for leg in range(4):
            self.T_feet_task[leg][0] = self.T_feet_home[leg][0] + self.slip_dyn.zmp_xy[0]
            self.T_feet_task[leg][1] = self.T_feet_home[leg][1] + self.slip_dyn.zmp_xy[1]

            self.B_feet_task[leg] = B_R_T_des @ (self.T_feet_task[leg] - self.pose_des[0:3])


    def setCheckTimings(self, expected_lift_off_time=None, expected_apex_time=None, expected_touch_down_time=None, clearance=None):
        if clearance is None:
            clearance = 0.

        if expected_lift_off_time is not None:
            self.check_lift_off_time = expected_lift_off_time - clearance

        if expected_apex_time is not None:
            self.check_apex_time = expected_apex_time - clearance
        else:
            self.check_apex_time = self.check_lift_off_time

        if expected_touch_down_time is not None:
            self.check_touch_down_time = expected_touch_down_time - clearance
        else:
            self.check_touch_down_time = self.check_apex_time


        assert self.check_lift_off_time <= self.check_apex_time, 'expected_lift_off_time should be >= expected_apex_time'
        assert self.check_apex_time <= self.check_touch_down_time, 'expected_touch_down_time should be >= expected_apex_time'

    def liftOff(self, t, sample, contact_state):
        anyLO = False
        if t > self.check_lift_off_time:
            anyLO = not any(contact_state)
            if anyLO:
                self.jumping_data_times.lift_off.set(t, sample)
        return anyLO

    def apexReached(self, t, sample, vel_z_pre, vel_z_now):

        if t > self.check_apex_time:
            if vel_z_pre >=0. and vel_z_now < 0.:
                self.jumping_data_times.apex.set(t, sample)
                return True

        return False

    def touchDown(self, t, sample, contacts_state):
        if t > self.check_touch_down_time:
            anyTD = any(contacts_state)
            if anyTD:
                self.jumping_data_times.touch_down.set(t, sample)
            return anyTD
        return False


    def velocity_margin(self, sp):
        limit_zmp = self.marginal_zmp(sp)
        P_x_last = self.slip_dyn.P_xy_stack[-2:, -2:]
        P_u_last = self.slip_dyn.P_u_stack[-2:, ].reshape(2, 1)

        # H = np.array([[0], [1]]) - P_u_last
        # G = np.linalg.inv(P_x_last)
        H = np.vstack([np.array([[0], [1]]) - P_u_last, 0])
        G = np.vstack([P_x_last, np.array([0, 1])])

        L = np.linalg.pinv(G) @ H

        limit_vx = (L * limit_zmp[0])[0]
        limit_vy = (L * limit_zmp[1])[0]

        marg_vx = limit_vx - self.init_vel[0]
        marg_vy = limit_vy - self.init_vel[1]

        eta = L[0]

        return eta, limit_zmp, marg_vx, marg_vy, limit_vx, limit_vy


    def marginal_zmp(self, sp):
        # marginal zmp is the intersection between the line defined by the touch down velocity and the support polygon
        # in the direction of the touch down velocity
        # sp lines equation : y = m*x+q
        # v td line equation: y = r*x
        # Wcontacts: lf, rf, lh, rh

        r = self.init_vel[1]/self.init_vel[0]
        angle_vTD = np.arctan2(self.init_vel[1], self.init_vel[0])
        m_zmp = np.zeros(2)
        for side in sp:
            q = sp[side]['q']
            m = sp[side]['m']
            zmp_x = q/(r-m)
            zmp_y = r * zmp_x
            # check if candidate zmp is in the bounds of the line
            if sp[side]['p0'][0] < zmp_x < sp[side]['p1'][0] and sp[side]['p0'][1] < zmp_y < sp[side]['p1'][1]:
                # check if the zmp is in the correct direction
                angle_zmp = np.arctan2(zmp_y, zmp_x)
                if np.abs(angle_vTD - angle_zmp)< 1e-5:
                    m_zmp[0] = zmp_x
                    m_zmp[1] = zmp_y
                    break
        return m_zmp


    def plot_margins(self, sp):
        fig, ax = plt.subplots()
        X_vals = []
        Y_vals = []
        for side in sp:
            X_vals.append(sp[side]['p0'][0])
            Y_vals.append(sp[side]['p0'][1])
        X_vals.append(X_vals[0])
        Y_vals.append(Y_vals[0])
        plt.plot(X_vals, Y_vals)

        self.m_zmp = self.marginal_zmp(sp)
        ax.add_patch(plt.Circle((self.slip_dyn.zmp_xy[0], self.slip_dyn.zmp_xy[1]), 0.02, color='r'))
        ax.add_patch(plt.Circle((self.m_zmp[0], self.m_zmp[1]), 0.02, color='b'))

        plt.plot([0, self.init_vel[0]], [0, self.init_vel[1]])

        ax.set_aspect('equal', adjustable='box')

        plt.show()






    def plot_ref(self, figure_id=None):
        label = 'CoM '
        axes = ['X', 'Y', 'Z']
        if figure_id is None:
            figure_id = 1000
        fig = plt.figure(figure_id)
        ax = fig.subplots(3, 2)

        ax[2, 0].axhline(y=self.slip_dyn.L - self.slip_dyn.max_spring_compression, color='r', linestyle='--')
        for i in range(0, 3):
            ax[i, 0].plot(self.slip_dyn.ctrl_time[:self.slip_dyn.ctrl_horz],
                         self.slip_dyn.T_p_com_ref[i, :self.slip_dyn.ctrl_horz].T, color='r', linewidth=4.)
            ax[i, 0].set_ylabel(label + axes[i] + '[m]')
            ax[i, 0].grid()
            ax[i, 0].set_xlabel('t [s]')

        for i in range(0, 3):
            ax[i, 1].plot(self.slip_dyn.ctrl_time[:self.slip_dyn.ctrl_horz],
                     self.slip_dyn.T_v_com_ref[i, :self.slip_dyn.ctrl_horz].T, color='r', linewidth=4.)
            ax[i, 1].set_ylabel('V '+ label + axes[i] + '[m/s]')
            ax[i, 0].set_xlabel('t [s]')
            ax[i, 1].grid()
            ax[i, 1].axhline(y=0., color='r', linestyle='--')
            ax[i, 0].set_xlabel('t [s]')
