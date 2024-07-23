from base_controllers.quadruped_controller import Controller
from landing_controller.controller.utility import *
from landing_controller.controller.extended.ext_landingController2 import *
import numpy as np
ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow250.world'  # if camera is activated , world will become slow
use_gui = True

simulation = {'name': 'elc',
              'pose': np.array([0., 0., 0.5, 0., 0., 0]),
              'twist': np.array([5., 0., 0., 0., 0., 0.]),
              'useWBC': True,
              'useIK': False,
              'typeWBC': 'qp',  # 'projection' or 'qp' (used only if useWBC is True)}
              'verbose': False,
              'save_video': True,
              'save_data': True,
              'save_path': ''
              }

if __name__ == '__main__':

    basePose_init = simulation['pose']
    baseTwist_init = simulation['twist']
    useIK = simulation['useIK']
    useWBC = simulation['useWBC']
    typeWBC = simulation['typeWBC']

    save_video = simulation['save_video']
    save_data = simulation['save_data']

    if save_video or save_data:
        setSavePath(simulation)
        save_path = simulation['save_path']
        saveInitConds(save_path, simulation)


    if save_video:
        #world_name = 'camera_slow.world'
        world_name = 'camera_slow_elc.world'



    try:
        p = Controller(ROBOT_NAME)

        ############
        # start up #
        ############
        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=False,
                          additional_args=['gui:=true',
                                           'go0_conf:=standDown',
                                           'pid_discrete_implementation:=false'])

        ##############
        # initialize #
        ##############
        p.startupProcedure()

        q_des = p.qj_0.copy()
        qd_des = np.zeros_like(q_des)
        tau_ffwd = np.zeros_like(q_des)
        p.unpause_physics_client()
        p.reset(basePoseW=basePose_init,
                baseTwistW=baseTwist_init,
                resetPid=useWBC)  # if useWBC = True, gains of pid are modified in landing phase
        baseLinVelW_init = baseTwist_init[:3].copy()

        q_des = p.q.copy()
        p.pause_physics_client()
        ELC_list = []
        elc = ExtendedLandingController(robot=p.robot,
                                        dt=p.dt,
                                        q0=np.hstack([p.u.linPart(p.basePoseW),p.quaternion,q_des]),
                                        smoothing_param=0.02,
                                        naive=False)
        ELC_list.append(elc)

        delta_expected_td = elc.expected_td_time(p.basePoseW[2], 0., elc.L)
        elc.setCheckTimings(expected_lift_off_time=None,
                                 expected_apex_time=None,
                                 expected_touch_down_time=p.time + delta_expected_td,
                                 clearance=0.05)  # about 12 mm of error in touch down

        # expected_velocity_td = p.u.linPart(p.TwistPoseW).copy()
        # expected_velocity_td[2] -= elc.g_mag * delta_expected_td
        # elc.elc_solve(expected_velocity_td)

        if save_video:
            # check if some old jpg are still in /tmp
            remove_jpg_cmd = "rm /tmp/camera_save/*"
            os.system(remove_jpg_cmd)

        comAccW_des = np.empty(6) * np.nan

        p.unpause_physics_client()
        
        p.setGravity(-elc.g_mag)

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

        base_collided = False
        kfes_collided = False
        while not ros.is_shutdown():

            p.pause_physics_client()
            #print('fsm_state:', fsm_state, 'use_lc:', elc.use_lc, 'isApexReached:', elc.lc_events.apex.detected, 'isTouchDownOccurred:', elc.lc_events.touch_down.detected, 'time:', p.time)
            # update kinematic and dynamic model
            p.updateKinematics(update_legOdom=elc.lc_events.touch_down.detected, noise=None)

            # check for collisions (only in simualtion)
            if not p.real_robot:
                base_collided = base_collided or p.checkBaseCollisions()
                kfes_collided = kfes_collided or p.checkKFECollisions()
                print("base_collided:", base_collided, "kfes_collided:", kfes_collided)

            # p.visualizeContacts()

            ###############################################
            # STATE 0 - before APEX: kinematic adjustment #
            ###############################################
            if fsm_state == 0:
                # check if apex is reached
                elc.apexReached(t=p.time,
                                sample=p.log_counter,
                                vel_z_pre=p.comTwistW_log[2, p.log_counter - 1],
                                vel_z_now=p.comTwistW[2])

                if not elc.lc_events.apex.detected:
                    # # kinematic adjustment
                    elc.flyingUp_phase(p.b_R_w)
                    # set references
                    for i, leg in enumerate(elc.legs): # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible  = p.IK.ik_leg(elc.B_feet_task[i],
                                                             leg,
                                                             p.legConfig[leg][0],
                                                             p.legConfig[leg][1])
                        if isFeasible:
                            p.u.setLegJointState(i, q_des_leg, q_des)
                    qd_des = np.zeros(p.robot.na)
                    tau_ffwd = p.self_weightCompensation()
                    p.comPoseW_des[:] = p.comTwistW_des[:] = comAccW_des[:] = np.nan

                else:
                    fsm_state += 1
                    for i in range(4):
                        p.contact_state[i] = False  # to be sure that contact state is false when flying down

            ########################################################################################
            # STATE 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment #
            ########################################################################################

            if fsm_state == 1:
                # check if touch down is occurred
                elc.touchDown(t=p.time,
                              sample=p.log_counter,
                              contacts_state=p.contact_state)


                if not elc.lc_events.touch_down.detected:
                    # compute landing trajectory + kinematic adjustment
                    w_R_hf = pin.rpy.rpyToMatrix(0, 0, p.u.angPart(p.basePoseW)[2])

                    baseLinVelW_init[2] = -elc.g_mag * (p.time-elc.lc_events.apex.t) #p.baseTwistW[2]
                    elc.flyingDown_phase(p.b_R_w @ w_R_hf, w_R_hf.T @ baseLinVelW_init)

                    # set references
                    # joints position
                    for i, leg in enumerate(elc.legs):  # ['lf', 'rf', 'lh', 'lh']
                        q_des_leg, isFeasible = p.IK.ik_leg(elc.B_feet_task[i],
                                                            leg,
                                                            p.legConfig[leg][0],
                                                            p.legConfig[leg][1],
                                                            simulation['verbose'])

                        if isFeasible:
                            p.u.setLegJointState(i, q_des_leg, q_des)

                        # joints velocity
                        B_contact_err = elc.B_feet_task[i] - p.B_contacts[i]
                        kv = .01 / p.dt
                        p.B_vel_contacts_des[i] = kv * B_contact_err
                        qd_des_leg = p.J_inv[i] @ p.B_vel_contacts_des[i]
                        p.u.setLegJointState(i, qd_des_leg, qd_des)

                    tau_ffwd = p.self_weightCompensation()
                    p.comPoseW_des[:] = p.comTwistW_des[:] = comAccW_des[:] = np.nan

                else:
                    fsm_state += 1
                    height = 0.
                    for leg in range(4):
                        height -= p.B_contacts[leg][2]
                    height /= 4
                    p.leg_odom.reset(np.hstack([0., 0., height, p.quaternion, p.q]))
                    # if use only ik -> same pid gains
                    # if use ik + wbc -> reduce pid gains
                    # if only wbc -> zero pid gains
                    if not useIK:
                        p.pid.setPDs(0., 0., 0.)
                    elif useWBC and useIK:
                        p.pid.setPDjoints(p.kp_wbc_j, p.kd_wbc_j, p.ki_wbc_j)
                    # elif not simulation['useWBC'] and simulation['useIK'] :
                    #       do not change gains

                    # save the  position at touch down
                    comPoseH, comRateH = p.World2Hframe(p.comPoseW, p.comTwistW)

                    elc.landedELC(comPoseH, comRateH, elc.use_lc)
                    p.setWBCConstraints()

                    p.W_contacts_TD = copy.deepcopy(p.W_contacts)


            #################################################################
            # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
            #################################################################
            if fsm_state == 2:

                # use last computed trajectories
                elc.landed_phaseELC(p.time, elc.use_lc)

                # set references
                b_R_w_des = pin.rpy.rpyToMatrix(p.u.angPart(elc.pose_des)).T
                omega_des_skew = pin.skew(b_R_w_des @ p.u.angPart(elc.pose_des))
                # joints position
                for i, leg in enumerate(elc.legs):  # ['lf', 'lh', 'rf','rh']
                    q_des_leg, isFeasible = p.IK.ik_leg(elc.B_feet_task[i],
                                                             leg,
                                                             p.legConfig[leg][0],
                                                             p.legConfig[leg][1],
                                                            simulation['verbose'])
                    if isFeasible:
                        p.u.setLegJointState(i, q_des_leg, q_des)

                    # joints velocity
                    p.B_vel_contacts_des[i] = omega_des_skew.T @ elc.B_feet_task[
                        i] - b_R_w_des @ p.u.linPart(elc.twist_des)
                    qd_leg_des = p.J_inv[i] @ p.B_vel_contacts_des[i]
                    p.u.setLegJointState(i, qd_leg_des, qd_des)

                if not useWBC:
                    tau_ffwd = p.gravityCompensation()

                # save LC references for com, feet, zmp
                p.comPoseW_des, p.comTwistW_des, comAccW_des = p.Hframe2World(elc.pose_des,
                                                                              elc.twist_des,
                                                                              elc.acc_des)

                if useWBC:
                    tau_ffwd = p.WBC(p.comPoseW_des, p.comTwistW_des, comAccW_des, type=typeWBC)

                if elc.ref_k == elc.VHSIP.ctrl_horz - 1:
                    if elc.use_lc:

                        qd_des[:] = 0
                        fsm_state = 3
                        break
                    else:
                        elc = ExtendedLandingController(robot=p.robot,
                                                        dt=p.dt,
                                                        q0=np.hstack([p.u.linPart(p.basePoseW), p.quaternion, q_des]),
                                                        smoothing_param=0.02,
                                                        naive=False)
                        ELC_list.append(elc)
                        elc.setCheckTimings(expected_lift_off_time=None,
                                            expected_apex_time=p.time+p.basePoseW[2]/elc.g_mag,
                                            expected_touch_down_time=p.time + elc.expected_td_time(p.comPoseW[2], p.comTwistW[2],
                                                                                                   elc.L),
                                            clearance=0.05)  # about 12 mm of error in touch down

                        baseLinVelW_init[:2] = p.comTwistW[:2] # z velocity is updated in the loop


                        p.pid.setPDjoints(p.kp_j, p.kd_j, p.ki_j)

                        fsm_state = 0


            for leg in range(4):
                p.W_contacts_des[leg] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ elc.B_feet_task[leg]
                p.B_contacts_des[leg] = elc.B_feet_task[leg].copy()

            # finally, send commands
            p.unpause_physics_client()
            p.send_command(q_des, qd_des, tau_ffwd, log_data_in_send_command=True)

        p.pid.setPDjoints(p.kp_j, p.kd_j, p.ki_j)
        p.gravityCompensation()


    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        print(e)
    finally:
        if save_video:
            # save the video
            p.saveVideo(save_path, speedUpDown=0.2)

        if save_data:
            EXTRADATA = {}
            for ii, elc in enumerate(ELC_list):
                EXTRADATA["lift_off_sample"+str(ii)] = elc.lc_events.lift_off.sample
                EXTRADATA["lift_off_t"+str(ii)] = elc.lc_events.lift_off.t
                EXTRADATA["apex_sample"+str(ii)] = elc.lc_events.apex.sample
                EXTRADATA["apex_t"+str(ii)] = elc.lc_events.apex.t
                EXTRADATA["touch_down_sample"+str(ii)] = elc.lc_events.touch_down.sample
                EXTRADATA["touch_down_t"+str(ii)] = elc.lc_events.touch_down.t
                EXTRADATA["T_centroid"+str(ii)] = elc.T_centroid
                EXTRADATA["T_zmp" + str(ii)] = elc.zmp_xy
                EXTRADATA["kinematic_slice_l0A" + str(ii)] = elc.kinematic_slice_l0.A
                EXTRADATA["kinematic_slice_l0b" + str(ii)] = elc.kinematic_slice_l0.b
                EXTRADATA["supportPolygon" + str(ii)] = elc.supportPolygon.points
                EXTRADATA["supportPolygonMargin" + str(ii)] = elc.supportPolygonMargin.points

            p.saveData(save_path, EXTRADATA=EXTRADATA, start=0, stop=p.log_counter, verbose=True)
        # these plots can be sistematically generated by makePlots() in controller/utility
        # reported here for post generation only
        # copy-past on the python console

        # joint position
        fig = plotJoint('position', time_log=p.time_log,
                        q_log=p.q_log, q_des_log=p.q_des_log,
                        sharex=True, sharey=False)
        # joint velocity
        fig = plotJoint('velocity', time_log=p.time_log,
                        qd_log=p.qd_log, qd_des_log=p.qd_des_log,
                        sharex=True, sharey=False)
        # joint torques
        fig = plotJoint('torque', time_log=p.time_log,
                        tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log, tau_des_log=p.tau_fb_log,
                        sharex=True, sharey=False)
        # com position
        fig = plotFrame('position', time_log=p.time_log,
                        des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                        title='CoM', frame='W',
                        sharex=True, sharey=False)
        # com velocity
        fig = plotFrame('velocity', time_log=p.time_log,
                        des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                        title='CoM', frame='W',
                        sharex=True, sharey=False)
        # feet position in w-frame and contact flag
        fig = plotContacts('position', time_log=p.time_log,
                           des_LinPose_log=p.W_contacts_des_log, LinPose_log=p.W_contacts_log,
                           contact_states=p.contact_state_log,
                           frame='W',
                           sharex=True, sharey=False)
        # feet position in b-frame and contact flag
        fig = plotContacts('position', time_log=p.time_log,
                           des_LinPose_log=p.B_contacts_des_log, LinPose_log=p.B_contacts_log,
                           contact_states=p.contact_state_log,
                           frame='B',
                           sharex=True, sharey=False)
        # force in world
        fig = plotContacts('GRFs', time_log=p.time_log,
                           des_Forces_log=p.grForcesW_des_log, Forces_log=p.grForcesW_log,
                           contact_states=p.contact_state_log,
                           frame='W',
                           sharex=True, sharey=False)

