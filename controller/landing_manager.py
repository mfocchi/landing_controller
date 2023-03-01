import numpy as np
from .landing_controller import LandingController
from .utility import makePlots, saveInitConds
import copy
import datetime
import os
import pinocchio as pin
import rospy as ros

class LandingManager:
    def __init__(self, p, settings):
        self.p = p
        self.lc = None

        self.settings = settings

        if self.settings['WORKSPACE']['save'] or \
                self.settings['PLOTS']['save'] or \
                self.settings['save_log'] or \
                self.settings['INIT_CONDS']['save_all']:
            now = datetime.datetime.now()
            now_s = str(now)
            now_s = now_s.replace('-', '')
            now_s = now_s.replace(' ', '_')
            now_s = now_s.replace(':', '')
            now_s = now_s[: now_s.find('.')]
            self.settings['save_path'] = os.environ['LOCOSIM_DIR']+'/landing_controller/simulations/' + now_s
            os.mkdir( self.settings['save_path'])

    def returnValue(self):
        # com close to des com
        if np.linalg.norm(self.p.comPoseW_des[0:3] - self.p.comPoseW[0:3]) > 0.1:
            return 1

        # feet don't move too much
        for i in range(4):
            for c in self.p.W_contacts_log[0:3,
                     self.lc.lc_events.touch_down.sample:self.p.log_counter].T:
                if np.linalg.norm(c - self.p.W_contacts_TD[0]) > 0.03:
                    return 2
        return 0


    def run(self, simulation_id, basePose_init=None, baseTwist_init=None, useIK=False, useWBC=True, typeWBC='projection', simplified=False):


        #########
        # reset #
        #########
        if self.p.real_robot:
            q_des = self.p.q.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)

            self.lc = LandingController(robot=self.p.robot,
                                        dt=2 * self.p.dt,
                                        q0=np.hstack([self.p.u.linPart(self.p.basePoseW),
                                                      self.p.quaternion,
                                                      q_des]))

        else:
            q_des = self.p.qj_0.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)
            self.p.unpause_physics_client()
            self.p.reset(basePoseW=basePose_init,
                         baseTwistW=baseTwist_init,
                         resetPid=useWBC)  # if useWBC = True, gains of pid are modified in landing phase

            q_des = self.p.q.copy()
            self.lc = LandingController(robot=self.p.robot,
                                        dt=2 * self.p.dt,
                                        q0=np.hstack([self.p.u.linPart(self.p.basePoseW),
                                                      self.p.quaternion,
                                                      q_des]))

            self.lc.setCheckTimings(expected_touch_down_time=self.p.time + 0.2, clearance=0.05)

            self.p.setGravity(-9.81)

        ###############################
        #### FINITE STATE MACHINE #####
        ###############################
        # STATES
        # fsm_state = 0 - before APEX: kinematic adjustment (not used in simulations)
        # fsm_state = 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment
        # fsm_state = 2 - from TOUCH DOWN to END: use last landing trajectory
        # fsm_state = 3 - quit
        fsm_state = 0
        # TRANSITIONS
        # iff isApexReached == True, fsm_state: 0 -> 1
        # iff isTouchDownOccurred == True, fsm_state: 1 -> 2


        # variables to be defined
        vcom_z_now = 0.
        vcom_z_pre = 0.

        start_time = self.p.time
        flag5s = False
        while not ros.is_shutdown():
            # print('fsm_state:', fsm_state, 'isApexReached:', isApexReached, 'isTouchDownOccurred:', isTouchDownOccurred)
            # update kinematic and dynamic model
            self.p.updateKinematics(update_legOdom=self.lc.lc_events.touch_down.detected)

            # self.p.visualizeContacts()

            ###############################################
            # STATE 0 - before APEX: kinematic adjustment #
            ###############################################
            if fsm_state == 0:
                # check if apex is reached
                if self.p.real_robot:
                    if self.p.time - start_time > 5:
                        if flag5s == False:
                            print("You can drop the robot now")
                            flag5s = True
                        isApexReached = self.lc.apexReachedReal(t=self.p.time, sample=self.p.log_counter,
                                                                baseLinAccW=self.p.baseLinAccW, window=1,
                                                                threshold=-5)
                else:
                    isApexReached = self.lc.apexReached(t=self.p.time,
                                                        sample=self.p.log_counter,
                                                        vel_z_pre=self.p.comTwistW_log[2, self.p.log_counter - 1],
                                                        vel_z_now=self.p.comTwistW[2])

                if self.lc.lc_events.apex.detected:
                    fsm_state += 1
                    for elem in self.p.contact_state:
                        elem = False  # to be sure that contact state is false when flying down
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
                if self.p.real_robot:
                    self.lc.touchDownReal(t=self.p.time,
                                          sample=self.p.log_counter,
                                          contacts_state=self.p.contact_state)
                else:
                    self.lc.touchDown(t=self.p.time,
                                      sample=self.p.log_counter,
                                      contacts_state=self.p.contact_state)

                if self.lc.lc_events.touch_down.detected:
                    fsm_state += 1
                    self.p.leg_odom.reset(np.hstack([0., 0., self.lc.L, self.p.quaternion, self.p.q]))
                    # if use only ik -> same pid gains
                    # if use ik + wbc -> reduce pid gains
                    # if only wbc -> zero pid gains
                    if not useIK:
                        self.p.pid.setPDs(0., 0., 0.)
                    elif useWBC and useIK:
                        self.p.pid.setPDjoints(self.p.kp_wbc_j, self.p.kd_wbc_j, self.p.ki_wbc_j)
                    # elif not simulation['useWBC'] and simulation['useIK'] :
                    #       do not change gains

                    # save the  position at touch down
                    self.lc.landed(self.p.comPoseW, self.p.comTwistW)

                    self.p.zmp[0] = self.lc.slip_dyn.zmp_xy[0] + self.lc.W_comPose_TD[0]
                    self.p.zmp[1] = self.lc.slip_dyn.zmp_xy[1] + self.lc.W_comPose_TD[1]

                    self.p.W_contacts_TD = copy.deepcopy(self.p.W_contacts)

                else:
                    # compute landing trajectory + kinematic adjustment
                    self.lc.flyingDown_phase(self.p.b_R_w, self.p.imu_utils.baseLinTwistImuW)

                    # set references
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                                 leg,
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
                                                           leg=leg,  # same for all the diag
                                                           update=i == 0)  # update Jacobians only with the first leg

                        self.p.u.setLegJointState(i, qd_leg_des, qd_des)

                    tau_ffwd = self.p.self_weightCompensation()

            #################################################################
            # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
            #################################################################
            if fsm_state == 2:
                if self.lc.lp_counter > 2 * self.lc.ref_k + 100:  # take a bit before quitting
                    fsm_state = 3
                    break
                else:
                    # use last computed trajectories
                    self.lc.landed_phase(self.p.time, simplified)

                    # set references
                    # joints position
                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'lh', 'rf','rh']
                        q_des_leg, isFeasible = self.p.IK.ik_leg(self.lc.B_feet_task[i],
                                                                 leg,
                                                                 self.p.legConfig[leg][0],
                                                                 self.p.legConfig[leg][1])
                        if isFeasible:
                            self.p.u.setLegJointState(i, q_des_leg, q_des)

                    # joints velocity
                    # W_v_feet = -self.p.u.linPart(self.lc.twist_des)
                    # B_v_feet = self.p.b_R_w @ W_v_feet

                    b_R_w_des = pin.rpy.rpyToMatrix(self.p.u.angPart(self.lc.pose_des)).T
                    omega_skew = pin.skew(b_R_w_des @ self.p.u.angPart(self.lc.pose_des))
                    # feet ref in B

                    for i, leg in enumerate(self.lc.legs):  # ['lf', 'lh', 'rf','rh']

                        self.p.B_vel_contact_des[i] = omega_skew.T @ self.lc.B_feet_task[i] - b_R_w_des @ self.p.u.linPart(self.lc.twist_des)
                        qd_leg_des = self.p.IK.diff_ik_leg(q_des=q_des,
                                                           B_v_foot=self.p.B_vel_contact_des[i],
                                                           leg=leg,
                                                           update= i == 0)
                        self.p.u.setLegJointState(i, qd_leg_des, qd_des)

                    if not useWBC:
                        tau_ffwd = self.p.gravityCompensation()

                    if useWBC:
                        tau_ffwd = self.p.WBC(self.lc.pose_des, self.lc.twist_des, self.lc.acc_des, type=typeWBC)

                        # save LC references for com, feet, zmp
                        self.p.comPoseW_des = self.lc.pose_des.copy()
                        self.p.comTwistW_des = self.lc.twist_des.copy()

            for leg in range(4):
                self.p.W_contacts_des[leg] = self.p.u.linPart(self.p.basePoseW) + self.p.b_R_w.T @ self.lc.B_feet_task[leg]
                self.p.B_contacts_des[leg] = self.lc.B_feet_task[leg].copy()

            # finally, send commands
            self.p.send_command(q_des, qd_des, tau_ffwd)

        if not self.p.real_robot:
            self.p.pause_physics_client()

        if self.settings['PLOTS']['save']:
            self.settings['SIMS'][simulation_id]['directory'] = self.settings['save_path']+'/sim' + simulation_id
            os.mkdir(self.settings['SIMS'][simulation_id]['directory'])
            saveInitConds(self.settings['SIMS'][simulation_id]['directory'],
                          self.settings['SIMS'][simulation_id],
                          speedUpDown= self.settings['VIDEO']['speedUpDown'],
                          verbose=self.settings['verbose'])

        if self.settings['PLOTS']['save'] or self.settings['PLOTS']['show']:
            makePlots(self.p, self.settings['FIGURES'], self.settings['PLOTS'], self.settings['verbose'])

        return self.returnValue()