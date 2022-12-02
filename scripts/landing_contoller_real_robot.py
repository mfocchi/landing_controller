import numpy as np
import matplotlib


matplotlib.use('TkAgg')


np.set_printoptions(linewidth=np.inf,  # number of characters per line befor new line
                    floatmode='fixed',  # print fixed numer of digits ...
                    precision=4,  # ... 4
                    sign=' ',  # print space if sign is plus
                    suppress=True,  # suppress scientific notation
                    threshold=np.inf)

from base_controllers.controller_new import Controller
import rospy as ros
from pysolo.controllers.landing_controller.landing_controller_with_python_bindings import LandingController
from base_controllers.components.leg_odometry.leg_odometry import LegOdometry
import pinocchio as pin


ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)


if __name__ == '__main__':

    p = Controller(ROBOT_NAME)
    p.startController()#, world_name="big_box.world")

    p.startupProcedure(check_contacts=True)  # overloaded method

    q_des = p.qj_0.copy()
    qd_des = np.zeros_like(q_des)
    p.imu_utils.W_lin_vel = p.u.linPart(p.baseTwistW)
    lc = LandingController(robotPin=p.robot,
                           dt=2 * p.dt,
                           q0=np.hstack([p.u.linPart(p.basePoseW), p.quaternion, q_des]),
                           use_real_robot=False)


    ##t_fly_exp = (-p.baseTwistW[2]**2 + np.sqrt(p.baseTwistW[2]**2 + 2 * 9.81 * (p.basePoseW[2]- 0.27)))/9.81
    lc.setCheckTimings(expected_touch_down_time=np.sqrt(2*p.basePoseW[2]/9.81)+p.time, clearance=0.1)


    # p.pause_physics_client()
    # p.updateKinematics()
    #
    # reference_push_up = PushUpReference(q0=p.q.copy(), amplitude=np.pi / 30, frequency=1)
    #
    # start_time = p.time
    #
    # q = np.hstack((0.0, 0.0, 0.0, p.quaternion, p.u.mapToRos(p.q)))
    # p.robot.forwardKinematics(q)
    # pin.updateFramePlacements(p.robot.model, p.robot.data)
    # robot_height = 0.
    # for id in p.robot.getEndEffectorsFrameId:
    #     robot_height += p.robot.data.oMf[id].translation[2]
    # robot_height /= -4.
    #
    #
    # q = np.hstack((0.0, 0.0, robot_height, p.quaternion, p.q))
    # leg_odom = LegOdometry(p.robot)
    # leg_odom.reset(q)
    # print(leg_odom.w_feet_pos_init)
    #
    #

    ############
    # SIMULATE #
    ############
    #### FINITE STATE MACHINE #####
    # STATES
    # fsm_state = 0 -
    # fsm_state = 1 -
    # fsm_state = 2 -

    fsm_state = 0
    # TRANSITIONS
    # iff isLiftedOffOccurred == True, fsm_state: 0 -> 1
    # iff isApexReached == True, fsm_state: 1 -> 2
    # iff isTouchDownOccurred == True, fsm_state: 2 -> 3
    isLiftedOffOccurred = False
    isApexReached = False
    isTouchDownOccurred = False

    # variables to be defined
    vcom_z_now = 0.
    vcom_z_pre = 0.

    # these variables are useful for computations
    # Neutral: base expressed in base frame
    # q_neutral[0:7] and v_neutral[0:6] cannot be modified
    q_neutral = pin.neutral(p.robot.model)
    v_neutral = pin.utils.zero(p.robot.model.nv)

    # Horizontal: base expressed in horizontal frame (there is rotation, there is NOT transaltion)
    # q_horz[0:3] and v_horz[0:3] cannot be modified
    q_horz = pin.neutral(p.robot.model)
    v_horz = pin.utils.zero(p.robot.model.nv)

    # World: base expressed in world frame
    q_w = pin.neutral(p.robot.model)
    v_w = pin.utils.zero(p.robot.model.nv)

    # World leg odom: base expressed in world frame, mesures obtained via leg odometry
    q_legOdom = pin.neutral(p.robot.model)
    v_legOdom = pin.utils.zero(p.robot.model.nv)
    #p.unpause_physics_client()
    try:

        while True:
            p.updateKinematics()
            #p.visualizeContacts()



            ########################################################################################
            # STATE 2 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment #
            ########################################################################################
            if fsm_state == 0:
                t_start = p.time
                fsm_state +=1

            if fsm_state == 1:
                t = p.time-t_start
                if 10. - t > 0:
                    print(t)
                    tau_ffwd = np.zeros(12)
                else:
                    fsm_state += 1


            if fsm_state == 2:
                    p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)
                    # check if touch down is occurred
                    # isTouchDownOccurred = lc.touchDown(t=p.time,
                    #                                    sample=p.log_counter,
                    #                                    contacts_state=p.contact_state)
                    # if isTouchDownOccurred:
                    #     print("*** Starting landing phase ***", p.time)
                    #     fsm_state += 1
                    # else:
                    # compute landing trajectory + kinematic adjustment
                    lc.run(FSMstate=fsm_state,
                           t=p.time,
                           quat=p.quaternion,
                           q_j=p.q,
                           omega_b=p.baseTwistW[3:6],
                           v_j=p.qd,
                           contacts_state=p.contact_state.copy(),
                           com_vel_W=p.imu_utils.W_lin_vel)
                           #com_vel_W=p.u.linPart(p.baseTwistW)) # TODO: replace with com


                    # # set references
                    p.B_contacts_des[0] = lc.B_lf_foot_task
                    p.B_contacts_des[1] = lc.B_rf_foot_task
                    p.B_contacts_des[2] = lc.B_lh_foot_task
                    p.B_contacts_des[3] = lc.B_rh_foot_task

                    damp = np.diag([1e-6] * 3)
                    kp = .01/p.dt
                    # for leg in range(4):
                    #     B_contact_err = p.B_contacts_des[leg] - p.B_contacts[leg]
                    #     B_vel_contact_des = kp * B_contact_err
                    #     leg_joints = range(6 + p.u.mapIndexToRos(leg) * 3, 6 + p.u.mapIndexToRos(leg) * 3 + 3)
                    #     J_des = p.robot.frameJacobian(np.hstack(( pin.neutral(p.robot.model)[0:7],
                    #                                               p.u.mapToRos(p.q_des))),
                    #                                   p.robot.model.getFrameId(p.ee_frames[leg]),
                    #                                   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
                    #     J_dls = np.linalg.inv(J_des + damp) # Jacobian in base frame!
                    #     p.u.setLegJointState(leg, J_dls @ B_vel_contact_des, qd_des)
                    #
                    # q_des += qd_des * p.dt
                    q_des = lc.q_des

                    tau_ffwd = p.self_weightCompensation()

                    # save the base position at touch down
                    W_p_base_TD = p.u.linPart(p.basePoseW)

            #################################################################
            # STATE 3 - from TOUCH DOWN to END: use last landing trajectory #
            #################################################################
            if fsm_state == 3:
                # use last computed trajectories
                lc.run(FSMstate=fsm_state,
                       t=p.time,
                       quat=p.quaternion,
                       q_j=p.q,
                       omega_b=p.baseTwistW[3:6],
                       v_j=p.qd,
                       contacts_state=p.contact_state.copy(),
                       com_vel_W=p.baseTwistW[:3])
                # set references
                q_des = lc.q_des
                qd_des =  lc.qd_des
                tau_ffwd = p.gravityCompensation()  + p.self_weightCompensation()
                p.comPoseW_des[:3] = lc.T_p_com_ref_k.copy()
                p.comPoseW_des[3:6] = lc.euler_des.copy()
                p.comTwistW_des[:3] = lc.T_v_com_ref_k.copy()

            # save leg odom position and velocity, for analysis
            p.T_p_base_leg_odom_lc = lc.leg_odom_pos.flatten()
            p.T_v_base_leg_odom_lc = lc.leg_odom_vel.flatten()

            p.w_p_b_legOdom = lc.leg_odom_pos.flatten()
            p.w_v_b_legOdom = lc.leg_odom_vel.flatten()

            # p.W_contacts_des[0] = lc.T_lf_foot_task.copy()
            # p.W_contacts_des[0][2] += p.W_contacts[0][2]
            #
            # p.W_contacts_des[1] = lc.T_rf_foot_task.copy()
            # p.W_contacts_des[1][2] += p.W_contacts[1][2]
            #
            # p.W_contacts_des[2] = lc.T_lh_foot_task.copy()
            # p.W_contacts_des[2][2] += p.W_contacts[2][2]
            #
            # p.W_contacts_des[3] = lc.T_rh_foot_task.copy()
            # p.W_contacts_des[3][2] += p.W_contacts[3][2]


            p.W_contacts_des[0] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_lf_foot_task.copy()
            p.W_contacts_des[1] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_rf_foot_task.copy()
            p.W_contacts_des[2] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_lh_foot_task.copy()
            p.W_contacts_des[3] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_rh_foot_task.copy()

            p.B_contacts_des[0] = lc.B_lf_foot_task.copy()
            p.B_contacts_des[1] = lc.B_rf_foot_task.copy()
            p.B_contacts_des[2] = lc.B_lh_foot_task.copy()
            p.B_contacts_des[3] = lc.B_rh_foot_task.copy()


            p.zmp[0] = lc.slip_dyn.zmp_xy[0]
            p.zmp[1] = lc.slip_dyn.zmp_xy[1]


            # finally, send commands
            #p.unpause_physics_client()
            p.send_command(q_des, qd_des, tau_ffwd)
            #p.pause_physics_client()


    except (ros.ROSInterruptException, ros.service.ServiceException):#, ValueError):
        ros.signal_shutdown("killed")
        p.deregister_node()

    from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs, plotFeet

    plotJoint('position', 0, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
              qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    plotJoint('velocity', 1, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
               qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    # plotJoint('torque', 2, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
    #           qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    # # plotJoint('torque', 2, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
    # #           qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log, tau_des_log=p.tau_des_log)

    p.comPoseW_des_log[0,:] += p.comPoseW_log[0,lc.jumping_data_times.touch_down.sample]
    p.comPoseW_des_log[1, :] += p.comPoseW_log[1, lc.jumping_data_times.touch_down.sample]
    plotCoM('position', 3, time_log=p.time_log.flatten(), basePoseW=p.comPoseW_log, des_basePoseW=p.comPoseW_des_log)
    # plotCoM('velocity', 4, time_log=p.time_log.flatten(), baseTwistW=p.comVelW_log)
    # plotGRFs(6, time_log=p.time_log.flatten(), des_forces=p.grForcesW_gt_log, act_forces=p.grForcesW_log)
    # plotFeet(7, time_log=p.time_log.flatten(), des_feet=p.B_contacts_des_log, act_feet=p.B_contacts_log)
    plotFeet(8, time_log=p.time_log.flatten(), des_feet=p.B_contacts_des_log, act_feet=p.B_contacts_log,
             contact_states=p.contact_state_log)
    # plotFeet(9, time_log=p.time_log.flatten(), des_feet=p.W_contacts_des_log,
    #          act_feet=p.W_contacts_log)
    plotFeet(10, time_log=p.time_log.flatten(), des_feet=p.W_contacts_des_log, act_feet=p.W_contacts_log,
                          contact_states=p.contact_state_log)

    # lc.plot_ref(11)

    # import matplotlib.pyplot as plt
    # plt.figure()
    # axes = ['x', 'y', 'z']
    # plt.suptitle('zmp in world frame')
    # for i in range(3):
    #     plt.subplot(3, 1, i + 1)
    #     plt.xlabel("t [s]")
    #     plt.ylabel("zmp_" + axes[i] + " [m]")
    #     plt.plot(p.time_log.T, p.zmp_log[i, :].T+p.comPosW_log[i, :].T, linestyle='-', color='red', lw=5)
    #     plt.grid()
    #
    # plt.figure()
    # axes = ['x', 'y', 'z']
    # plt.suptitle('zmp in base frame')
    # for i in range(3):
    #     plt.subplot(3, 1, i + 1)
    #     plt.xlabel("t [s]")
    #     plt.ylabel("zmp_" + axes[i] + " [m]")
    #     plt.plot(p.time_log.T, p.zmp_log[i, :].T, linestyle='-', color='blue', lw=5)
    #     plt.grid()