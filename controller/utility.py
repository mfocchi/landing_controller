import matplotlib.pyplot as plt
import numpy as np
from termcolor import colored
DEG2RAD = np.pi/180


def initCond2str(init_cond, speedUpDown=1.):
    ICstr = ''
    ICstr += 'id: '   + init_cond['id'] + '\n'
    ICstr += 'name: ' + init_cond['name'] + '\n'
    ICstr += 'base height: ' + str(np.round(init_cond['pose'][2], 3)) + ' m\n'
    ICstr += 'base pitch: ' + str(np.round(init_cond['pose'][4], 3)/DEG2RAD) + ' deg\n'
    ICstr += 'base fw vel: ' + str(np.round(init_cond['twist'][0], 3)) + ' m/s\n'
    ICstr += 'base lat vel: ' + str(np.round(init_cond['twist'][1], 3)) + ' m/s\n'
    ICstr += 'base pitch vel: ' + str(np.round(init_cond['twist'][4], 3)/DEG2RAD) + ' deg/s\n'
    ICstr += 't_video: ' + str(init_cond['t_video']) + '\n'
    if speedUpDown > 0. and speedUpDown != 1.:
        ICstr += 't_video_speedUpDown: ' + str(init_cond['t_video']/speedUpDown) + '\n'
    return ICstr



def manipulateFig(fig, filename, PLOT_SETTINGS):
    fig.set_size_inches([PLOT_SETTINGS['width_inches'], PLOT_SETTINGS['height_inches']])

    if PLOT_SETTINGS['save']:
        plt.savefig(PLOT_SETTINGS['directory_path'] + '/' + filename+ '.png')
        plt.close()
        del fig
        #print(colored('Plot ' + filename+ '.png saved', color='green'), flush=True)

    if PLOT_SETTINGS['show']:
        plt.show()




from controller.landing_controller_with_python_bindings import LandingController
from controller.settings import PLOT_SETTINGS
from base_controllers.utils.common_functions import * #(matplotlib.pyplot, numpy, plots, ros)
import os
from termcolor import colored
class Simulation:
    def __init__(self, p):
        self.p = p
        self.lc = None

    def succeed(self, simulation_id, basePose_init, baseTwist_init, useIK=False, useWBC=True, typeWBC='projection', simplified=False):

        self.p.unpause_physics_client()



        q_des = self.p.qj_0.copy()
        qd_des = np.zeros_like(q_des)
        tau_ffwd = np.zeros_like(q_des)

        #########
        # reset #
        #########
        self.p.reset(basePoseW=basePose_init,
                baseTwistW=baseTwist_init,
                resetPid=useWBC)  # if useWBC = True, gains of pid are modified in landing phase

        self.lc = LandingController(robot=self.p.robot,
                               dt=2 * self.p.dt,
                               q0=np.hstack([self.p.u.linPart(self.p.basePoseW),
                                             self.p.quaternion,
                                             q_des]))

        self.lc.setCheckTimings(expected_touch_down_time=np.sqrt(2 * basePose_init[2] / 9.81) + self.p.time, clearance=0.05)

        self.p.setGravity(-9.81)

        ############
        # SIMULATE #
        ############
        #### FINITE STATE MACHINE #####
        # STATES
        # fsm_state = 0 - before APEX: kinematic adjustment (not used in simulations)
        # fsm_state = 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment
        # fsm_state = 2 - from TOUCH DOWN to END: use last landing trajectory
        # fsm_state = 3 - quit
        fsm_state = 0
        # TRANSITIONS
        # iff isApexReached == True, fsm_state: 0 -> 1
        # iff isTouchDownOccurred == True, fsm_state: 1 -> 2

        isApexReached = False
        isTouchDownOccurred = False

        # variables to be defined
        vcom_z_now = 0.
        vcom_z_pre = 0.


        while fsm_state < 3:

            # print('fsm_state:', fsm_state, 'isApexReached:', isApexReached, 'isTouchDownOccurred:', isTouchDownOccurred)
            # update kinematic and dynamic model
            self.p.updateKinematics()
            # update estimate on linear velocity
            self.p.imu_utils.compute_lin_vel(self.p.W_base_lin_acc, self.p.loop_time)

            #self.p.visualizeContacts()

            ###############################################
            # STATE 0 - before APEX: kinematic adjustment #
            ###############################################
            if fsm_state == 0:
                # check if apex is reached
                vcom_z_now = self.p.comTwistW[2]
                isApexReached = self.lc.apexReached(t=self.p.time,
                                               sample=self.p.log_counter,
                                               vel_z_pre=vcom_z_pre,
                                               vel_z_now=vcom_z_now)
                vcom_z_pre = vcom_z_now

                if isApexReached:
                    fsm_state += 1
                    self.p.contact_state[:] = False  # to be sure that contact state is false when flying down
                else:
                    # # kinematic adjustment
                    # self.lc.flyingUp_phase(self.p.b_R_w)
                    # # set references
                    # for i, leg in enumerate(self.lc.legs): # ['lf', 'rf', 'lh', 'lh']
                    #     q_des_leg, isFeasible  = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                    #                                          self.p.robot.model.getFrameId(leg+'_foot'),
                    #                                          self.p.legConfig[leg][0],
                    #                                          self.p.legConfig[leg][1])
                    #     if isFeasible:
                    #         self.p.u.setLegJointState(i, q_des_leg, q_des)
                    # qd_des = np.zeros(self.p.robot.na)
                    # tau_ffwd = self.p.self_weightCompensation()
                    pass

            ########################################################################################
            # STATE 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment #
            ########################################################################################

            if fsm_state == 1:
                # check if touch down is occurred
                isTouchDownOccurred = self.lc.touchDown(t=self.p.time,
                                                   sample=self.p.log_counter,
                                                   contacts_state=self.p.contact_state)

                if isTouchDownOccurred:
                    fsm_state += 1
                    self.p.leg_odom.reset(np.hstack([0., 0., self.lc.L, self.p.quaternion, self.p.q]))
                    # if use only ik -> same pid gains
                    # if use ik + wbc -> reduce pid gains
                    # if only wbc -> zero pid gains
                    if not useIK:
                        self.p.pid.setPDs(0., 0., 0.)
                    elif useWBC and useIK:
                        self.p.pid.setPDs(15., 0., 1.)
                    # elif not simulation['useWBC'] and simulation['useIK'] :
                    #       do not change gains

                    # save the base position at touch down
                    W_p_base_TD = self.p.u.linPart(self.p.basePoseW)
                    W_com_TD = self.p.u.linPart(self.p.comPoseW)
                    self.p.zmp[0] = self.lc.slip_dyn.zmp_xy[0] + W_com_TD[0]
                    self.p.zmp[1] = self.lc.slip_dyn.zmp_xy[1] + W_com_TD[1]
                    self.p.W_contacts_TD = copy.deepcopy(self.p.W_contacts)
                else:
                    # compute landing trajectory + kinematic adjustment
                    self.lc.flyingDown_phase(self.p.b_R_w, self.p.imu_utils.W_lin_vel)

                    # set references
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                            self.p.robot.model.getFrameId(leg + '_foot'),
                                                            self.p.legConfig[leg][0],
                                                            self.p.legConfig[leg][1])
                        if isFeasible:
                            self.p.u.setLegJointState(i, q_des_leg, q_des)

                    # joints velocity
                    # do not merge this for loop with the previous one: I need full q_des
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        B_contact_err = self.lc.B_feet_task[i] - self.p.B_contacts[i]
                        kv = .01 / self.p.dt
                        B_vel_contact_des = kv * B_contact_err
                        qd_leg_des = self.p.IK.diff_ik_leg(q_des=q_des,
                                                      B_v_foot=B_vel_contact_des,
                                                      foot_idx=self.p.robot.model.getFrameId(leg + '_foot'),
                                                      damp=1e-6,  # same for all the diag
                                                      update=i == 0)  # update Jacobians only with the first leg

                        self.p.u.setLegJointState(i, qd_leg_des, qd_des)

                    tau_ffwd = self.p.self_weightCompensation()

            #################################################################
            # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
            #################################################################
            if fsm_state == 2:
                if self.lc.lp_counter > 2 * self.lc.ref_k + 100:  # take a bit before quitting
                    fsm_state = 3
                else:
                    self.p.w_p_b_legOdom, self.p.w_v_b_legOdom = self.p.leg_odom.estimate_base_wrt_world(self.p.contact_state,
                                                                                          self.p.quaternion,
                                                                                          self.p.q,
                                                                                          self.p.u.angPart(self.p.baseTwistW),
                                                                                          self.p.qd)
                    # use last computed trajectories
                    self.lc.landed_phase(self.p.time, self.p.euler, simplified)

                    # set references
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                            self.p.robot.model.getFrameId(leg + '_foot'),
                                                            self.p.legConfig[leg][0],
                                                            self.p.legConfig[leg][1])
                        if isFeasible:
                            self.p.u.setLegJointState(i, q_des_leg, q_des)

                    # joints velocity
                    W_v_feet = -self.p.u.linPart(self.lc.twist_des)
                    B_v_feet = self.p.b_R_w @ W_v_feet
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        qd_leg_des = self.p.IK.diff_ik_leg(q_des=q_des,
                                                      B_v_foot=B_v_feet,
                                                      foot_idx=self.p.robot.model.getFrameId(leg + '_foot'),
                                                      damp=1e-6,
                                                      update=i == 0)
                        self.p.u.setLegJointState(i, qd_leg_des, qd_des)

                    if not useWBC:
                        tau_ffwd = self.p.gravityCompensation()

                    if useWBC:
                        off = np.array([W_com_TD[0], W_com_TD[1], 0, 0, 0, 0])
                        tau_ffwd = self.p.WBC(off + self.lc.pose_des, self.lc.twist_des, self.lc.acc_des, type=typeWBC)

                        # save LC references for com, feet, zmp
                        self.p.comPoseW_des = self.lc.pose_des.copy() + off
                        self.p.comTwistW_des = self.lc.twist_des.copy()

            # finally, send commands
            self.p.send_command(q_des, qd_des, tau_ffwd)



            for leg in range(4):
                self.p.W_contacts_des[leg] = self.p.u.linPart(self.p.basePoseW) + self.p.b_R_w.T @ self.lc.B_feet_task[leg]
                self.p.B_contacts_des[leg] = self.lc.B_feet_task[leg].copy()

        ####################
        # store some plots #
        ####################
        self.p.pause_physics_client()
        if PLOT_SETTINGS['save']:
            PLOT_SETTINGS['directory_path'] = PLOT_SETTINGS['save_path'] + '/sim' + simulation_id
            os.mkdir(PLOT_SETTINGS['directory_path'])

        if PLOT_SETTINGS['save'] or PLOT_SETTINGS['show']:
            # joint position
            fig = plotJoint('position', 0, time_log=self.p.time_log.flatten(), q_log=self.p.q_log, q_des_log=self.p.q_des_log)
            manipulateFig(fig, 'q', PLOT_SETTINGS)

            # joint torques
            fig = plotJoint('torque', fig.number + 1, time_log=self.p.time_log.flatten(), tau_ffwd_log=self.p.tau_ffwd_log,
                            tau_log=self.p.tau_log,
                            tau_des_log=self.p.tau_des_log)
            manipulateFig(fig, 'tau', PLOT_SETTINGS)

            # com position
            # self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:] += self.p.comPoseW_log[
            #     0, self.lc.jumping_data_times.touch_down.sample]
            # self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:] += self.p.comPoseW_log[
            #     1, self.lc.jumping_data_times.touch_down.sample]

            fig = plotCoM('position', 1, time_log=self.p.time_log.flatten(), basePoseW=self.p.comPoseW_log,
                          des_basePoseW=self.p.comPoseW_des_log)
            manipulateFig(fig, 'com', PLOT_SETTINGS)

            # com velocity
            fig = plotCoM('velocity', fig.number + 1, time_log=self.p.time_log.flatten(), baseTwistW=self.p.comTwistW_log,
                          des_baseTwistW=self.p.comTwistW_des_log)
            manipulateFig(fig, 'vcom', PLOT_SETTINGS)

            # # feet position in w-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=self.p.time_log.flatten(), des_feet=self.p.W_contacts_des_log,
            #                act_feet=self.p.W_contacts_log, contact_states=self.p.contact_state_log)
            # manipulateFig(fig, 'W_feet', PLOT_SETTINGS)
            #
            # # feet position in b-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=self.p.time_log.flatten(), des_feet=self.p.B_contacts_des_log,
            #                act_feet=self.p.B_contacts_log, contact_states=self.p.contact_state_log)
            # manipulateFig(fig, 'B_feet', PLOT_SETTINGS)

            # force in world
            fig = plotGRFs_withContacts(fig.number + 1, time_log=self.p.time_log.flatten(), des_forces=self.p.grForcesW_gt_log,
                                        act_forces=self.p.grForcesW_des_log, contact_states=self.p.contact_state_log)
            manipulateFig(fig, 'grfs', PLOT_SETTINGS)

            # margins
            # fig = plt.figure(fig.number + 1)
            # ax = fig.subplots()
            # X_vals = []
            # Y_vals = []
            # T_contacts = self.p.W_contacts.copy()
            # for c in T_contacts:
            #     c[0] -= self.p.comPoseW_log[0, self.lc.jumping_data_times.touch_down.sample]
            #     c[1] -= self.p.comPoseW_log[1, self.lc.jumping_data_times.touch_down.sample]
            # T_sp = self.p.support_poly(T_contacts)
            # for side in T_sp:
            #     X_vals.append(T_sp[side]['p0'][0])
            #     Y_vals.append(T_sp[side]['p0'][1])
            # X_vals.append(X_vals[0])
            # Y_vals.append(Y_vals[0])
            # plt.plot(X_vals, Y_vals, lw=6)
            #
            # eta, limit_zmp, marg_vx, marg_vy, limit_vx, limit_vy = self.lc.velocity_margin(T_sp)
            # # for plotting in world, we should add the position as td
            # ax.add_patch(plt.Circle((self.lc.slip_dyn.zmp_xy[0], self.lc.slip_dyn.zmp_xy[1]), 0.005, color='r'))
            # ax.add_patch(plt.Circle((limit_zmp[0], limit_zmp[1]), 0.005, color='b'))
            # n = np.linalg.norm(self.lc.init_vel)
            #
            # plt.plot([0, limit_vx / n], [0, limit_vy / n], lw=4)
            # plt.plot([0, self.lc.init_vel[0] / n], [0, self.lc.init_vel[1] / n], lw=6)
            #
            # # self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:] -= self.p.comPoseW_log[ 0, self.lc.jumping_data_times.touch_down.sample]
            # # self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:] -= self.p.comPoseW_log[1, self.lc.jumping_data_times.touch_down.sample]
            # # plt.plot(self.p.comPoseW_des_log[0, self.lc.jumping_data_times.touch_down.sample:], self.p.comPoseW_des_log[1, self.lc.jumping_data_times.touch_down.sample:])
            # ax.set_aspect('equal', adjustable='box')
            #
            # plt.legend(
            #     ["support polygon", "zmp", "limit zmp", "limit TD velocity 'normalized'", "TD velocity normalized"],
            #     fontsize=20)
            #
            # manipulateFig(fig, 'margins', PLOT_SETTINGS)
            #
            # # init cond file
            # if PLOT_SETTINGS['save']:
            #     f = open(PLOT_SETTINGS['directory_path'] + "/simulation.txt", "w")
            #
            #     f.write(initCond2str(simulation, PLOT_SETTINGS['speedUpDown']))
            #     f.close()
            #     print(colored('File ' + PLOT_SETTINGS['directory_path'] + '/simulation.txt saved', color='green'),
            #           flush=True)

        ######################
        # success conditions #
        ######################
        # com close to des com
        if np.linalg.norm(self.p.comPoseW_des[0:3]-self.p.comPoseW[0:3]) > 0.1 :
            return 1

        # feet don't move too much
        for i in range(4):
            for c in self.p.W_contacts_log[0:3, self.lc.jumping_data_times.touch_down.sample:self.p.log_counter].T:
                if np.linalg.norm(c - self.p.W_contacts_TD[0]) > 0.02:
                    return 2

        return 0



