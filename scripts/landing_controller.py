import matplotlib
matplotlib.use('TkAgg')
from base_controllers.controller_new import Controller
from controller.landing_controller_with_python_bindings import LandingController
from controller.settings import SETTINGS
from controller.utility import initCond2str, manipulateFig
from base_controllers.utils.common_functions import * #(matplotlib.pyplot, numpy, plots, ros)
import os
from termcolor import colored


np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed numer of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)

ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    try:

        if SETTINGS['PLOTS']['video']:
            world_name = 'camera_'+world_name

        p.startController(world_name=world_name,
                          additional_args=['gui:=False', 'go0_conf:=standDown'])

        p.startupProcedure()  # overloaded method

        if SETTINGS['PLOTS']['video']:
            init_video_frame = p.get_current_frame_file()

        sim_counter = -1
        for simulation in SETTINGS['SIMS']:
            sim_counter += 1
            if sim_counter < 10:
                simulation['id'] = '0' + str(sim_counter)
            else:
                simulation['id'] = str(sim_counter)

            q_des = p.qj_0.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)

            t_start_video = p.time.copy()

            simulation['t_video'] = t_start_video
            if simulation['id'] != '00':
                simulation['t_video'] += SETTINGS['SIMS'][sim_counter - 1]['t_video']

            #########
            # reset #
            #########
            print(colored("Starting simulation " + simulation['id'] + ': ' + simulation['name'], "blue"))
            p.reset(basePoseW=simulation['pose'],
                    baseTwistW=simulation['twist'],
                    resetPid=simulation['useWBC'])  # if useWBC = True, gains of pid are modified in landing phase

            lc = LandingController(robot=p.robot,
                                   dt=2 * p.dt,
                                   q0=np.hstack([p.u.linPart(p.basePoseW),
                                                 p.quaternion,
                                                 q_des]))

            lc.setCheckTimings()#expected_touch_down_time=np.sqrt(2 * simulation['pose'][2] / 9.81) + p.time, clearance=0.05)

            p.setGravity(-9.81)
            p.pause_physics_client()

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

            p.unpause_physics_client()
            while fsm_state < 3:

                # print('fsm_state:', fsm_state, 'isApexReached:', isApexReached, 'isTouchDownOccurred:', isTouchDownOccurred)
                # update kinematic and dynamic model
                p.updateKinematics()
                # update estimate on linear velocity
                p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)

                p.visualizeContacts()

                ###############################################
                # STATE 0 - before APEX: kinematic adjustment #
                ###############################################
                if fsm_state == 0:
                    # check if apex is reached
                    vcom_z_now = p.comTwistW[2]
                    isApexReached = lc.apexReached(t=p.time,
                                                   sample=p.log_counter,
                                                   vel_z_pre=vcom_z_pre,
                                                   vel_z_now=vcom_z_now)
                    vcom_z_pre = vcom_z_now

                    if isApexReached:
                        fsm_state += 1
                        p.contact_state[:] = False  # to be sure that contact state is false when flying down
                    else:
                        # # kinematic adjustment
                        # lc.flyingUp_phase(p.b_R_w)
                        # # set references
                        # for i, leg in enumerate(lc.legs): # ['lf', 'rf', 'lh', 'lh']
                        #     q_des_leg, isFeasible  = p.IK.ik_leg(lc.B_feet_task[i],
                        #                                          p.robot.model.getFrameId(leg+'_foot'),
                        #                                          p.legConfig[leg][0],
                        #                                          p.legConfig[leg][1])
                        #     if isFeasible:
                        #         p.u.setLegJointState(i, q_des_leg, q_des)
                        # qd_des = np.zeros(p.robot.na)
                        # tau_ffwd = p.self_weightCompensation()
                        pass

                ########################################################################################
                # STATE 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment #
                ########################################################################################

                if fsm_state == 1:
                    # check if touch down is occurred
                    isTouchDownOccurred = lc.touchDown(t=p.time,
                                                       sample=p.log_counter,
                                                       contacts_state=p.contact_state)

                    if isTouchDownOccurred:
                        fsm_state += 1
                        p.leg_odom.reset(np.hstack([0., 0., lc.L, p.quaternion, p.q]))
                        # if use only ik -> same pid gains
                        # if use ik + wbc -> reduce pid gains
                        # if only wbc -> zero pid gains
                        if not simulation['useIK']:
                            p.pid.setPDs(0., 0., 0.)
                        elif simulation['useWBC'] and simulation['useIK']:
                            p.pid.setPDs(15., 0., 1.)
                        # elif not simulation['useWBC'] and simulation['useIK'] :
                        #       do not change gains

                        # save the base position at touch down
                        # W_p_base_TD = p.u.linPart(p.basePoseW)
                        # W_com_TD = p.u.linPart(p.comPoseW)
                        # p.zmp[0] = lc.slip_dyn.zmp_xy[0] + W_com_TD[0]
                        # p.zmp[1] = lc.slip_dyn.zmp_xy[1] + W_com_TD[1]
                    else:
                        # compute landing trajectory + kinematic adjustment
                        lc.flyingDown_phase(p.b_R_w, p.imu_utils.W_lin_vel)

                        # set references
                        # joints position
                        for i, leg in enumerate(lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                            q_des_leg, isFeasible = p.IK.ik_leg(lc.B_feet_task[i],
                                                                p.robot.model.getFrameId(leg + '_foot'),
                                                                p.legConfig[leg][0],
                                                                p.legConfig[leg][1])
                            if isFeasible:
                                p.u.setLegJointState(i, q_des_leg, q_des)

                        # joints velocity
                        # do not merge this for loop with the previous one: I need full q_des
                        for i, leg in enumerate(lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                            B_contact_err = lc.B_feet_task[i] - p.B_contacts[i]
                            kv = .01 / p.dt
                            B_vel_contact_des = kv * B_contact_err
                            qd_leg_des = p.IK.diff_ik_leg(q_des=q_des,
                                                          B_v_foot=B_vel_contact_des,
                                                          foot_idx=p.robot.model.getFrameId(leg + '_foot'),
                                                          damp=1e-6,  # same for all the diag
                                                          update=i == 0)  # update Jacobians only with the first leg

                            p.u.setLegJointState(i, qd_leg_des, qd_des)

                        tau_ffwd = p.self_weightCompensation()

                #################################################################
                # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
                #################################################################
                if fsm_state == 2:
                    if lc.lp_counter > 2 * lc.ref_k + 100:  # take a bit before quitting
                        fsm_state = 3
                    else:
                        p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.estimate_base_wrt_world(p.contact_state,
                                                                                              p.quaternion,
                                                                                              p.q,
                                                                                              p.u.angPart(p.baseTwistW),
                                                                                              p.qd)
                        # use last computed trajectories
                        lc.landed_phase(p.time, p.euler)

                        # set references
                        # joints position
                        for i, leg in enumerate(lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                            q_des_leg, isFeasible = p.IK.ik_leg(lc.B_feet_task[i],
                                                                p.robot.model.getFrameId(leg + '_foot'),
                                                                p.legConfig[leg][0],
                                                                p.legConfig[leg][1])
                            if isFeasible:
                                p.u.setLegJointState(i, q_des_leg, q_des)

                        # joints velocity
                        W_v_feet = -p.u.linPart(lc.twist_des)
                        B_v_feet = p.b_R_w @ W_v_feet
                        for i, leg in enumerate(lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                            qd_leg_des = p.IK.diff_ik_leg(q_des=q_des,
                                                          B_v_foot=B_v_feet,
                                                          foot_idx=p.robot.model.getFrameId(leg + '_foot'),
                                                          damp=1e-6,
                                                          update=i == 0)
                            p.u.setLegJointState(i, qd_leg_des, qd_des)

                        if not simulation['useWBC']:
                            tau_ffwd = p.gravityCompensation()

                        if simulation['useWBC']:
                            #off = np.array([W_com_TD[0], W_com_TD[1], 0, 0, 0, 0])
                            off = np.zeros(6)
                            tau_ffwd = p.WBC(off + lc.pose_des, lc.twist_des, lc.acc_des, type=simulation['typeWBC'])

                # finally, send commands
                p.send_command(q_des, qd_des, tau_ffwd)

                # save LC references for com, feet, zmp
                p.comPoseW_des = lc.pose_des.copy()
                p.comTwistW_des = lc.twist_des.copy()

                for leg in range(4):
                    p.W_contacts_des[leg] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_feet_task[leg]
                    p.B_contacts_des[leg] = lc.B_feet_task[leg].copy()



            ####################
            # store some plots #
            ####################
            p.pause_physics_client()
            if SETTINGS['PLOTS']['save']:
                SETTINGS['PLOTS']['directory_path'] = SETTINGS['PLOTS']['save_path'] + '/sim' + simulation['id']
                os.mkdir(SETTINGS['PLOTS']['directory_path'])
            #
            # joint position
            fig = plotJoint('position', 0, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log)
            manipulateFig(fig, 'q', SETTINGS['PLOTS'])

            # joint torques
            fig = plotJoint('torque', fig.number + 1, time_log=p.time_log.flatten(), tau_ffwd_log=p.tau_ffwd_log,
                            tau_log=p.tau_log,
                            tau_des_log=p.tau_des_log)
            manipulateFig(fig, 'tau', SETTINGS['PLOTS'])

            # com position
            p.comPoseW_des_log[0, lc.jumping_data_times.touch_down.sample:] += p.comPoseW_log[
                0, lc.jumping_data_times.touch_down.sample]
            p.comPoseW_des_log[1, lc.jumping_data_times.touch_down.sample:] += p.comPoseW_log[
                1, lc.jumping_data_times.touch_down.sample]

            fig = plotCoM('position', 1, time_log=p.time_log.flatten(), basePoseW=p.comPoseW_log,
                          des_basePoseW=p.comPoseW_des_log)
            manipulateFig(fig, 'com', SETTINGS['PLOTS'])

            # com velocity
            fig = plotCoM('velocity', fig.number + 1, time_log=p.time_log.flatten(), baseTwistW=p.comTwistW_log,
                          des_baseTwistW=p.comTwistW_des_log)
            manipulateFig(fig, 'vcom', SETTINGS['PLOTS'])

            # # feet position in w-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=p.time_log.flatten(), des_feet=p.W_contacts_des_log,
            #                act_feet=p.W_contacts_log, contact_states=p.contact_state_log)
            # manipulateFig(fig, 'W_feet', SETTINGS['PLOTS'])
            #
            # # feet position in b-frame and contact flag
            # fig = plotFeet(fig.number + 1, time_log=p.time_log.flatten(), des_feet=p.B_contacts_des_log,
            #                act_feet=p.B_contacts_log, contact_states=p.contact_state_log)
            # manipulateFig(fig, 'B_feet', SETTINGS['PLOTS'])

            # force in world
            fig = plotGRFs_withContacts(fig.number + 1, time_log=p.time_log.flatten(), des_forces=p.grForcesW_gt_log,
                                        act_forces=p.grForcesW_des_log, contact_states=p.contact_state_log)
            manipulateFig(fig, 'grfs', SETTINGS['PLOTS'])

            # margins
            # fig = plt.figure(fig.number + 1)
            # ax = fig.subplots()
            # X_vals = []
            # Y_vals = []
            # T_contacts = p.W_contacts.copy()
            # for c in T_contacts:
            #     c[0] -= p.comPoseW_log[0, lc.jumping_data_times.touch_down.sample]
            #     c[1] -= p.comPoseW_log[1, lc.jumping_data_times.touch_down.sample]
            # T_sp = p.support_poly(T_contacts)
            # for side in T_sp:
            #     X_vals.append(T_sp[side]['p0'][0])
            #     Y_vals.append(T_sp[side]['p0'][1])
            # X_vals.append(X_vals[0])
            # Y_vals.append(Y_vals[0])
            # plt.plot(X_vals, Y_vals, lw=6)
            #
            # eta, limit_zmp, marg_vx, marg_vy, limit_vx, limit_vy = lc.velocity_margin(T_sp)
            # # for plotting in world, we should add the position as td
            # ax.add_patch(plt.Circle((lc.slip_dyn.zmp_xy[0], lc.slip_dyn.zmp_xy[1]), 0.005, color='r'))
            # ax.add_patch(plt.Circle((limit_zmp[0], limit_zmp[1]), 0.005, color='b'))
            # n = np.linalg.norm(lc.init_vel)
            #
            # plt.plot([0, limit_vx / n], [0, limit_vy / n], lw=4)
            # plt.plot([0, lc.init_vel[0] / n], [0, lc.init_vel[1] / n], lw=6)
            #
            # # p.comPoseW_des_log[0, lc.jumping_data_times.touch_down.sample:] -= p.comPoseW_log[ 0, lc.jumping_data_times.touch_down.sample]
            # # p.comPoseW_des_log[1, lc.jumping_data_times.touch_down.sample:] -= p.comPoseW_log[1, lc.jumping_data_times.touch_down.sample]
            # # plt.plot(p.comPoseW_des_log[0, lc.jumping_data_times.touch_down.sample:], p.comPoseW_des_log[1, lc.jumping_data_times.touch_down.sample:])
            # ax.set_aspect('equal', adjustable='box')
            #
            # plt.legend(
            #     ["support polygon", "zmp", "limit zmp", "limit TD velocity 'normalized'", "TD velocity normalized"],
            #     fontsize=20)
            #
            # manipulateFig(fig, 'margins', SETTINGS['PLOTS'])
            #
            # # init cond file
            # if SETTINGS['PLOTS']['save']:
            #     f = open(SETTINGS['PLOTS']['directory_path'] + "/simulation.txt", "w")
            #
            #     f.write(initCond2str(simulation, SETTINGS['PLOTS']['speedUpDown']))
            #     f.close()
            #     print(colored('File ' + SETTINGS['PLOTS']['directory_path'] + '/simulation.txt saved', color='green'),
            #           flush=True)

            p.unpause_physics_client()




    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        print(e)
    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

    # store all the init conds
    if SETTINGS['PLOTS']['save']:
        f = open(SETTINGS['PLOTS']['save_path'] + "/ALL_simulations.txt", "w")
        for simulation in SETTINGS['SIMS']:
            f.write(initCond2str(simulation, SETTINGS['PLOTS']['speedUpDown']) + '\n' + '-' * 10 + '\n')
        f.close()
        print(SETTINGS['PLOTS']['save_path'] + '/ALL_simulations.txt saved')

        # save the video
        p.save_video(SETTINGS['PLOTS']['save_path'], start_file=init_video_frame, speedUpDown=SETTINGS['PLOTS']['speedUpDown'])
