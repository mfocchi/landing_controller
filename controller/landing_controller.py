##############
# DEPRECATED #
##############
import numpy as np
import pinocchio as pin
from pysolo.controllers.inverse_kinematics.inverse_kinematics import InverseKinematics
from pysolo.controllers.leg_odometry.leg_odometry import LegOdometry
from base_controllers.utils.utils import Utils



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
    def __init__(self, robotPin, dt, ctrl_horz, q0, g_mag=9.81, use_real_robot=False ):
        self.robotPin = robotPin
        self.qj_home = q0[7:]
        self.v_home = pin.utils.zero(self.robotPin.nv)

        self.use_real_robot = use_real_robot

        self.dt = dt

        self.c_k = -1
        self.ref_k = 1

        self.ref_com_k = -1

        self._t_apex = 0.

        # number of calls of the landing controller
        # self.n_calls = -1

        # MSD params
        self.g_mag = g_mag
        self.m = self.robotPin.robot_mass

        self.K = 0.
        self.D = 0.

        self.ctrl_horz = ctrl_horz
        self.ctrl_range = range(1, self.ctrl_horz+1)

        pin.forwardKinematics(self.robotPin.model, self.robotPin.data,
                              np.hstack([pin.neutral(self.robotPin.model)[:7], self.qj_home]))
        pin.updateFramePlacements(self.robotPin.model, self.robotPin.data)

        self.L = -self.robotPin.data.oMf[self.robotPin.model.getFrameId('lf_foot')].translation[2].copy()
        self.max_spring_compression = 0.5 * self.L

        self.Ts_log = []

        self.T_o_B = np.array([0,0,self.L])

        self.B_lf_foot_home = self.robotPin.data.oMf[self.robotPin.model.getFrameId('lf_foot')].translation.copy()
        self.B_lh_foot_home = self.robotPin.data.oMf[self.robotPin.model.getFrameId('lh_foot')].translation.copy()
        self.B_rf_foot_home = self.robotPin.data.oMf[self.robotPin.model.getFrameId('rf_foot')].translation.copy()
        self.B_rh_foot_home = self.robotPin.data.oMf[self.robotPin.model.getFrameId('rh_foot')].translation.copy()

        self.T_lf_foot_home = np.array([self.B_lf_foot_home[0], self.B_lf_foot_home[1], 0])
        self.T_lh_foot_home = np.array([self.B_lh_foot_home[0], self.B_lh_foot_home[1], 0])
        self.T_rf_foot_home = np.array([self.B_rf_foot_home[0], self.B_rf_foot_home[1], 0])
        self.T_rh_foot_home = np.array([self.B_rh_foot_home[0], self.B_rh_foot_home[1], 0])

        self.T_lf_foot_task = self.T_lf_foot_home.copy()
        self.T_lh_foot_task = self.T_lh_foot_home.copy()
        self.T_rf_foot_task = self.T_rf_foot_home.copy()
        self.T_rh_foot_task = self.T_rh_foot_home.copy()

        self.B_lf_foot_task = self.B_lf_foot_home.copy()
        self.B_lh_foot_task = self.B_lh_foot_home.copy()
        self.B_rf_foot_task = self.B_rf_foot_home.copy()
        self.B_rh_foot_task = self.B_rh_foot_home.copy()
        # OCP params
        # rho_sq is a trade-off between null velocity and perfect position matching at steady-state
        self.rho_sq_x = 100#0.01
        self.rho_sq_y = 100#0.01
        self.alpha = 0.

        # selector matrices
        self._V = np.array([[1, 0]])
        self._P = np.array([[0, 1]])

        # weights lists
        self.Wx = [np.nan] * 3
        self.Wy = [np.nan] * 3

        self.gravity_Force = -self.m * self.g_mag

        # Discredized version of the matrices for x-y dynamics:
        # Axy_d[1,1]=omega^2 and Bxy_d=[0,0] = -omega^2, updated during computation since omega is time-varying
        # x and y have the same dynamics!
        self.Axy = np.array([[0., 0.],
                             [1., 0.]])
                                               
        self.Bxy = np.array([[0.],
                             [0.]])

        # self.Axy_d = np.empty((2,2,self.ctrl_horz + 1)) * np.nan
        # self.Bxy_d = np.empty((2,1,self.ctrl_horz + 1)) * np.nan
        self.Axy_d = np.empty((2,2,self.ctrl_horz + 1)) * np.nan
        self.Bxy_d = np.empty((2,1,self.ctrl_horz + 1)) * np.nan
        self.PHI_xy = np.empty((2*self.ctrl_horz, 2))*np.nan
        self.GAMMA_xy = np.empty((2*self.ctrl_horz, 1))*np.nan


        # reference trajectories computed by the landing controller (expressed in terrain frame)
        #self.T_p_com_ref_hist = np.empty([3, self.ctrl_horz + 1, 1000 * self.ctrl_horz]) * np.nan
        #self.T_v_com_ref_hist = np.empty([3, self.ctrl_horz + 1, 1000 * self.ctrl_horz]) * np.nan

        self.T_p_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan
        self.T_v_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan
        self.T_a_com_ref = np.empty([3, self.ctrl_horz + 1]) * np.nan

        self.state_x = np.empty([2 * (self.ctrl_horz + 1), 1]) * np.nan
        self.state_y = np.empty([2 * (self.ctrl_horz + 1), 1]) * np.nan

        self.ctrl_time = np.arange(0., (self.ctrl_horz+1)*self.dt, self.dt)


        self.omega_sq_ref = np.empty([self.ctrl_horz + 1]) * np.nan

        self.zmp = np.empty([2,   10*self.ctrl_horz]) * np.nan


        self.zmp_x = 0.
        self.zmp_y = 0.


        self.IK = InverseKinematics(self.robotPin)

        self.leg_odom = LegOdometry(self.robotPin, q0)
        self.q_des  = np.zeros(self.robotPin.na)
        self.qd_des = np.zeros(self.robotPin.na)
        self.a_com_ref = np.zeros([3, 1])

        self._T_state_x_init = np.zeros([2, 1])
        self._T_state_y_init = np.zeros([2, 1])
        self._T_state_z_init = np.zeros([2, 1])

        self.has_never_broke_the_contact = [True]*4
        self.leg_indeces_have_never_broke_the_contact = [0,1,2,3]
        self.leg_odom_pos = np.zeros([3, 1])
        self.leg_odom_vel = np.zeros([3, 1])

        self.u = Utils()
        self.base_vel = np.zeros(3)

    def _z_dynamics(self, T_state_z):
        # I am assuming that the system is autonomous (because gravity is already removed by a gravity compensator)
        # since the system is linear, I have an analytic form of the state evolution
        # moreover, since I am setting coincident eigenvalues, the state evolution equation are the following
        #
        # state = [velocity]
        #         [position]

        D_over_m = self.D / self.m
        K_over_m = self.K / self.m

        self.T_p_com_ref[2, 0] = self.L
        self.T_v_com_ref[2, 0] = T_state_z[0]
        self.T_a_com_ref[2, 0] = -D_over_m * T_state_z[0]

        self.omega_sq_ref[0] = (self.g_mag + self.T_a_com_ref[2, 0]) / (self.T_p_com_ref[2, 0])

        for k in self.ctrl_range:
            t = k*self.dt
            exp_eig_t = np.exp(self._eig_z * t)

            self.T_p_com_ref[2, k] = exp_eig_t * t * self.T_v_com_ref[2, 0] + self.L
            self.T_v_com_ref[2, k] = exp_eig_t * (self._eig_z * t + 1) * self.T_v_com_ref[2, 0]
            self.T_a_com_ref[2, k] = -D_over_m * self.T_v_com_ref[2, k] - K_over_m * self.T_p_com_ref[2, k]

            self.omega_sq_ref[k] = (self.g_mag + self.T_a_com_ref[2, k]) / (self.T_p_com_ref[2, k])

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
            self.Axy_d[0,0] += 1.
            self.Axy_d[1,1] += 1.

            self.Bxy_d = self.dt * self.Bxy

            PHI_xy = self.Axy_d @ PHI_xy
            GAMMA_xy = self.Axy_d @ GAMMA_xy + self.Bxy_d

            self.PHI_xy[2*(k-1):2*k, :] = PHI_xy
            self.GAMMA_xy[2*(k-1):2*k, :] = GAMMA_xy



        return PHI_xy, GAMMA_xy

    def _weights(self, PHI_xy, GAMMA_xy):

        m1 = self._V @ PHI_xy
        m2 = self._V @ GAMMA_xy

        M1 = m1.T @ m1
        M2 = m2.T @ m2
        M3 = m1.T @ m2

        n1 = self._P @ PHI_xy
        n2 = self._P @ GAMMA_xy

        N1 = n1.T @ n1
        N2 = n2.T @ n2 - 2 * n2.T
        N3 = n1.T @ n2 - n1.T

        self.Wx[0] = self.rho_sq_x * M1 + N1
        self.Wx[1] = self.rho_sq_x * M2 + N2
        self.Wx[2] = self.rho_sq_x * M3 + N3

        self.Wy[0] = self.rho_sq_y * M1 + N1
        self.Wy[1] = self.rho_sq_y * M2 + N2
        self.Wy[2] = self.rho_sq_y * M3 + N3
        return self.Wx, self.Wy


    def _xy_dynamics(self, state_x, zmp_x, state_y, zmp_y):
        #
        # state = [velocity]
        #         [position]
        # self.T_p_com_ref[0, 0] = state_x[1]
        # self.T_v_com_ref[0, 0] = state_x[0]
        # self.T_a_com_ref[0, 0] = self.omega_sq_ref[0] * (self.T_p_com_ref[0, 0] - zmp_x)
        #
        # self.T_p_com_ref[1, 0] = state_y[1]
        # self.T_v_com_ref[1, 0] = state_y[0]
        # self.T_a_com_ref[1, 0] = self.omega_sq_ref[0] * (self.T_p_com_ref[1, 0] - zmp_y)

        # for k in self.ctrl_range:
        #
        #     state_x = self.Axy_d[:,:,k] @ state_x + self.Bxy_d[:,:,k] @ zmp_x
        #     state_y = self.Axy_d[:,:,k] @ state_y + self.Bxy_d[:,:,k] @ zmp_y
        #
        #     self.T_p_com_ref[0, k] = state_x[1]
        #     self.T_v_com_ref[0, k] = state_x[0]
        #     self.T_a_com_ref[0, k] = self.omega_sq_ref[k] * (self.T_p_com_ref[0, k] - zmp_x)
        #
        #     self.T_p_com_ref[1, k] = state_y[1]
        #     self.T_v_com_ref[1, k] = state_y[0]
        #     self.T_a_com_ref[1, k] = self.omega_sq_ref[k] * (self.T_p_com_ref[1, k] - zmp_y)
        self.state_x[0:2] = state_x
        self.state_y[0:2] = state_y
        self.state_x[2:] = self.PHI_xy @ state_x + self.GAMMA_xy @ zmp_x
        self.state_y[2:] = self.PHI_xy @ state_y + self.GAMMA_xy @ zmp_y

        self.T_p_com_ref[0, :] = self.state_x[1::2].flatten()
        self.T_v_com_ref[0, :] = self.state_x[0::2].flatten()

        self.T_p_com_ref[1, :] = self.state_y[1::2].flatten()
        self.T_v_com_ref[1, :] = self.state_y[0::2].flatten()


    def _compute_zmp(self):
        self._z_dynamics(self._T_state_z_init.copy())

        PHI_xy, GAMMA_xy = self._propagation_matrices()
        Wx, Wy = self._weights(PHI_xy, GAMMA_xy)

        zmp_x = - 1/Wx[1] @ Wx[2].T @ self._T_state_x_init
        zmp_y = - 1/Wy[1] @ Wy[2].T @ self._T_state_y_init
        self._xy_dynamics(self._T_state_x_init.copy(), zmp_x,
                          self._T_state_y_init.copy(), zmp_y)

        return zmp_x, zmp_y


    def pushing_phase(self, t, quat, q_j, omega_b, v_j, contacts_state):
        self.has_never_broke_the_contact &= contacts_state

        legs_in_contact = []
        for n, value in enumerate(contacts_state):
            if value:
                legs_in_contact.append(n)

                # only compute leg odometry
        # foot frame and terrain frame have the same orientation, so the velocity of the com in both the frames is the same
        T_p_B, T_v_B = self.leg_odom.estimate_base_wrt_world(legs_in_contact, quat, q_j, omega_b, v_j)
        self.leg_odom_pos = T_p_B
        self.leg_odom_vel = T_v_B

        self.base_vel = self.leg_odom_vel.copy()

        # position is not needed. the only parameter needed is com (base) velocity along x,y-axes
        # com velocity along z-axis will be overwritten during fly down phase
        #
        # state = [velocity]
        #         [position]
        self._T_state_x_init[0] = T_v_B[0]
        self._T_state_x_init[1] = 0.

        self._T_state_y_init[0] = T_v_B[1]
        self._T_state_y_init[1] = 0.

        self._T_state_z_init[0] = 0.
        self._T_state_z_init[1] = self.L

        self.V0_z = T_v_B[2].copy()
        self._t_LO = t



    def flyingUp_phase(self, t, quat, q_j):
        # kinematic adjustment: drive feet to create a plane parallel to landing surface and located at distance
        # self.L from the base
        # i.e., move feet to the home position according to the rotation of the base

        self._t_apex = t

        if self.use_real_robot:
            self._T_state_z_init[0] = self.V0_z-self.g_mag * (t-self._t_LO)
            self.base_vel[2] = self._T_state_z_init[0]

        B_R_T = pin.Quaternion(quat).toRotationMatrix()


        # at the beginning, B-frame and T-frame have the same orientation, thus at this stage we only have to rotate the
        # feet position expressed in base frame

        self.B_lf_foot_task = B_R_T @ (self.T_lf_foot_home - self.T_o_B)
        self.B_rf_foot_task = B_R_T @ (self.T_rf_foot_home - self.T_o_B)
        self.B_lh_foot_task = B_R_T @ (self.T_lh_foot_home - self.T_o_B)
        self.B_rh_foot_task = B_R_T @ (self.T_rh_foot_home - self.T_o_B)

        # print('B_R_T', B_R_T)
        #
        # print('B_lf_foot_task_x (home) =',  self.B_lf_foot_task[0])
        # print('B_rf_foot_task_x (home) =',  self.B_rf_foot_task[0])
        # print('B_lh_foot_task_x (home) =',  self.B_lh_foot_task[0])
        # print('B_rh_foot_task_x (home) =',  self.B_rh_foot_task[0])
        self.q_des[0:3]  = self.IK.ik_leg(self.B_lf_foot_task, self.robotPin.model.getFrameId('lf_foot'))[0].flatten()
        self.q_des[3:6]  = self.IK.ik_leg(self.B_rf_foot_task, self.robotPin.model.getFrameId('rf_foot'))[0].flatten()
        self.q_des[6:9]  = self.IK.ik_leg(self.B_lh_foot_task, self.robotPin.model.getFrameId('lh_foot'))[0].flatten()
        self.q_des[9:12] = self.IK.ik_leg(self.B_rh_foot_task, self.robotPin.model.getFrameId('rh_foot'))[0].flatten()


        self.leg_odom_pos = np.zeros(3)
        self.leg_odom_vel = np.zeros(3)


    def flyingDown_phase(self, t, quat, q_j, omega_b, v_j, contacts_state, com_vel_W=None):
        # (re-)compute zmp and move feet plane accordingly
        B_R_T = pin.Quaternion(quat).toRotationMatrix()

        # supposed velocity of com in z-direction immediately before touch down
        #
        # state = [velocity]
        #         [position]
        if not self.use_real_robot:
            self._T_state_x_init[0] = com_vel_W[0].copy()
            self._T_state_y_init[0] = com_vel_W[1].copy()
            self._T_state_z_init[0] = com_vel_W[2].copy()
        else:
            # x and y are computed with lego odom in pushUp_phase and kept constant
            self._T_state_z_init[0] = self.V0_z-self.g_mag * (t-self._t_LO)
            self.base_vel[2] = self._T_state_z_init[0]
        print('V0 lo:', self._T_state_z_init[0])

        self.K = self.m * pow( self._T_state_z_init[0]/(np.e * self.max_spring_compression), 2)
        self.D = 2 * np.sqrt(self.m * self.K)
        self._eig_z = -np.sqrt(self.K/self.m)

        Ts = -8/self._eig_z
        # self.ctrl_horz = int(Ts/self.dt)+1
        # self.ctrl_range = range(1, self.ctrl_horz)
        self.Ts_log.append(Ts)


        # save zmp for latter analysis
        self.zmp_x, self.zmp_y = self._compute_zmp()


        # task for feet in terrain frame
        if self.alpha < 10:
            self.alpha += 1
        a = self.alpha/10.


        self.T_lf_foot_task[0] = self.T_lf_foot_home[0] + a*self.zmp_x #self.T_p_com_ref[0, self.ref_k]
        self.T_lf_foot_task[1] = self.T_lf_foot_home[1] + a*self.zmp_y #self.T_p_com_ref[1, self.ref_k]

        self.T_lh_foot_task[0] = self.T_lh_foot_home[0] + a*self.zmp_x #self.T_p_com_ref[0, self.ref_k]
        self.T_lh_foot_task[1] = self.T_lh_foot_home[1] + a*self.zmp_y #self.T_p_com_ref[1, self.ref_k]

        self.T_rf_foot_task[0] = self.T_rf_foot_home[0] + a*self.zmp_x #self.T_p_com_ref[0, self.ref_k]
        self.T_rf_foot_task[1] = self.T_rf_foot_home[1] + a*self.zmp_y #self.T_p_com_ref[1, self.ref_k]

        self.T_rh_foot_task[0] = self.T_rh_foot_home[0] + a*self.zmp_x #self.T_p_com_ref[0, self.ref_k]
        self.T_rh_foot_task[1] = self.T_rh_foot_home[1] + a*self.zmp_y #self.T_p_com_ref[1, self.ref_k]

        # notice that here the desired position of feet in terrain frame (z = 0) must be mapped into base frame
        # (translation + orientation)



        self.B_lf_foot_task = B_R_T @ (self.T_lf_foot_task-self.T_o_B )
        self.B_rf_foot_task = B_R_T @ (self.T_rf_foot_task-self.T_o_B )
        self.B_lh_foot_task = B_R_T @ (self.T_lh_foot_task-self.T_o_B )
        self.B_rh_foot_task = B_R_T @ (self.T_rh_foot_task-self.T_o_B )

        self.q_des[0:3]  = self.IK.ik_leg(self.B_lf_foot_task, self.robotPin.model.getFrameId('lf_foot'))[0].flatten()
        self.q_des[3:6]  = self.IK.ik_leg(self.B_rf_foot_task, self.robotPin.model.getFrameId('rf_foot'))[0].flatten()
        self.q_des[6:9]  = self.IK.ik_leg(self.B_lh_foot_task, self.robotPin.model.getFrameId('lh_foot'))[0].flatten()
        self.q_des[9:12] = self.IK.ik_leg(self.B_rh_foot_task, self.robotPin.model.getFrameId('rh_foot'))[0].flatten()


        # print('zmp_x', self.zmp_x)
        # print('zmp_y', self.zmp_y)
        # print('B_lf_foot_task_x =',  self.B_lf_foot_task[0])
        # print('B_rf_foot_task_x =',  self.B_rf_foot_task[0])
        # print('B_lh_foot_task_x =',  self.B_lh_foot_task[0])
        # print('B_rh_foot_task_x =',  self.B_rh_foot_task[0])
        #
        # print('delta_lf =', B_R_T.T @ (self.B_lf_foot_task - B_lf_foot_task_pre))
        # print('delta_rf =', B_R_T.T @ (self.B_rf_foot_task - B_rf_foot_task_pre))
        # print('delta_lh =', B_R_T.T @ (self.B_lh_foot_task - B_lh_foot_task_pre))
        # print('delta_rh =', B_R_T.T @ (self.B_rh_foot_task - B_rh_foot_task_pre))

        legs_in_contact = []
        for n, value in enumerate(contacts_state):
            if value:
                legs_in_contact.append(n)

        self.leg_odom.reset(np.hstack((0., 0., self.L, 0., 0., 0., 1., q_j)))
        self.leg_odom_pos = np.zeros(3)
        self.leg_odom_vel = np.zeros(3)
    
    def landed_phase(self, quat, q_j, omega_b, v_j, contacts_state):
        # use the last trajectory computed
        # task for feet in terrain frame
        if self.ref_k < self.ctrl_horz:
            self.ref_k += 1
        
        B_R_T = pin.Quaternion(quat).toRotationMatrix()#.T

        self.T_p_com_ref_k = self.T_p_com_ref[:, self.ref_k]
        self.T_v_com_ref_k = self.T_v_com_ref[:, self.ref_k]

        self.T_o_B[2] = self.T_p_com_ref[2, self.ref_k]


        self.T_lf_foot_task[0] = self.T_lf_foot_home[0] + self.zmp_x
        self.T_lf_foot_task[1] = self.T_lf_foot_home[1] + self.zmp_y

        self.T_lh_foot_task[0] = self.T_lh_foot_home[0] + self.zmp_x
        self.T_lh_foot_task[1] = self.T_lh_foot_home[1] + self.zmp_y

        self.T_rf_foot_task[0] = self.T_rf_foot_home[0] + self.zmp_x
        self.T_rf_foot_task[1] = self.T_rf_foot_home[1] + self.zmp_y

        self.T_rh_foot_task[0] = self.T_rh_foot_home[0] + self.zmp_x
        self.T_rh_foot_task[1] = self.T_rh_foot_home[1] + self.zmp_y


        self.B_lf_foot_task = B_R_T @ (self.T_lf_foot_task - self.T_p_com_ref[:, self.ref_k])
        self.B_rf_foot_task = B_R_T @ (self.T_rf_foot_task - self.T_p_com_ref[:, self.ref_k])
        self.B_lh_foot_task = B_R_T @ (self.T_lh_foot_task - self.T_p_com_ref[:, self.ref_k])
        self.B_rh_foot_task = B_R_T @ (self.T_rh_foot_task - self.T_p_com_ref[:, self.ref_k])

        self.q_des[0:3]  = self.IK.ik_leg(self.B_lf_foot_task, self.robotPin.model.getFrameId('lf_foot'))[0].flatten()
        self.q_des[3:6]  = self.IK.ik_leg(self.B_rf_foot_task, self.robotPin.model.getFrameId('rf_foot'))[0].flatten()
        self.q_des[6:9]  = self.IK.ik_leg(self.B_lh_foot_task, self.robotPin.model.getFrameId('lh_foot'))[0].flatten()
        self.q_des[9:12] = self.IK.ik_leg(self.B_rh_foot_task, self.robotPin.model.getFrameId('rh_foot'))[0].flatten()

        fb_config_des = np.hstack((pin.neutral(self.robotPin.model)[:7], self.u.mapToRos(self.q_des)))

        # in terrain frame, all feet have the same reference for velocity ,that is the opposite of the reference of the com
        T_v_feet = -self.T_v_com_ref[:, self.ref_k]

        for leg in range(0, 4):
            id = self.u.mapIndexToRos(leg)
            footid = self.robotPin.getEndEffectorsFrameId[id]
            leg_joints_init = 6 + id * 3
            leg_joints_end =  6 + id * 3 + 3
            B_Jc = pin.computeFrameJacobian(self.robotPin.model, self.robotPin.data, fb_config_des, footid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            T_Jc_lin_leg = B_R_T.T @ B_Jc[0:3, leg_joints_init:leg_joints_end]
            self.qd_des[leg * 3:leg * 3 + 3] = np.linalg.inv(T_Jc_lin_leg) @ T_v_feet

        legs_in_contact = []
        for n, value in enumerate(contacts_state):
            if value:
                legs_in_contact.append(n)
        T_p_B, T_v_B = self.leg_odom.estimate_base_wrt_world(legs_in_contact, quat, q_j, omega_b, v_j)

        self.leg_odom_pos = T_p_B
        self.leg_odom_vel = T_v_B
        self.base_vel = T_v_B.copy()



    def run(self, FMstate, t, quat, q_j, omega_b, v_j, contacts_state, com_vel_W=None):
        self.c_k += 1

        if FMstate == 0:
            self.pushing_phase(t, quat, q_j, omega_b, v_j, contacts_state)

        elif FMstate == 1:
            self.flyingUp_phase(t, quat, q_j)

        elif FMstate == 2:
            self.flyingDown_phase(t, quat, q_j, omega_b, v_j, contacts_state, com_vel_W)

        elif FMstate == 3:
            self.landed_phase(quat, q_j, omega_b, v_j, contacts_state)