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
from controller.landing_controller_with_python_bindings import LandingController
from base_controllers.components.leg_odometry.leg_odometry import LegOdometry
import pinocchio as pin
from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs, plotFeet
import matplotlib.pyplot as plt
import os
import datetime
from screeninfo import get_monitors
from termcolor import colored

import warnings
warnings.filterwarnings('ignore')

import signal


ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)

robot_height_init = 0.259
speedUpDown = 0.2
DEG2RAD = np.pi/180
INIT_COND = []
# different heights, zero orientation, zero init vel
# INIT_COND.append({'id':'00',
#                   'pose': np.array([0., 0., robot_height_init + 0.20, 0., 0., 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'01',
#                   'pose': np.array([0., 0., robot_height_init + 0.40, 0., 0., 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'02',
#                   'pose': np.array([0., 0., robot_height_init + 0.6, 0., 0., 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'03',
#                   'pose': np.array([0., 0., robot_height_init + 1.0, 0., 0., 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
#
# # same heights, zero orientation, different init vels
# INIT_COND.append({'id':'08',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 0., 0.]),
#                   'twist': np.array([0.3, 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'09',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 0., 0.]),
#                   'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'10',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 0., 0.]),
#                   'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
#                   't_video':0})
#
# # same height, different orientations, zero init vel
#
# INIT_COND.append({'id':'04',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., -20. * DEG2RAD, 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'05',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., -5. * DEG2RAD, 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'06',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 5. * DEG2RAD, 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'07',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 20. * DEG2RAD, 0.]),
#                   'twist': np.array([0., 0., 0., 0., 0., 0.]),
#                   't_video':0})
#
# # same heights, different orientations, different init vels
# INIT_COND.append({'id':'11',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., -10. * DEG2RAD, 0.]),
#                   'twist': np.array([0.3, 0., 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'12',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 0., 10. * DEG2RAD, 0.]),
#                   'twist': np.array([0., 0.5, 0., 0., 0., 0.]),
#                   't_video':0})
# INIT_COND.append({'id':'13',
#                   'pose': np.array([0., 0., robot_height_init + 0.30, 10. * DEG2RAD, 0., 0.]),
#                   'twist': np.array([-0.2, 0.5, 0., 0., 0., 0.]),
#                   't_video':0})


def initCond2str(init_cond, speedUpDown=1.):
    ICstr = ''
    ICstr += 'id: '   + init_cond['id'] + '\n'
    ICstr += 'name: ' + init_cond['name'] + '\n'
    ICstr += 'base height: ' + str(np.round(init_cond['pose'][2], 3)) + ' m\n'
    ICstr += 'base pitch: ' + str(np.round(init_cond['pose'][4], 3)/DEG2RAD) + ' deg\n'
    ICstr += 'base fw vel: ' + str(np.round(init_cond['twist'][0], 3)) + ' m/s\n'
    ICstr += 'base lat vel: ' + str(np.round(init_cond['twist'][1], 3)) + ' m/s\n'
    ICstr += 't_video: ' + str(init_cond['t_video']) + '\n'
    if speedUpDown > 0. and speedUpDown != 1.:
        ICstr += 't_video_speedUpDown: ' + str(init_cond['t_video']/speedUpDown) + '\n'
    return ICstr



INIT_COND.append({'id': '00',
                  'name': 'A_high_jump',
                  'pose': np.array([0., 0., 0.65, 0, 0., 0.]),
                  'twist': np.array([1., 0., 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '01',
                  'name':'A_high_jump',
                  'pose': np.array([0., 0., 0.85, 0, 0., 0.]),
                  'twist': np.array([.5, 0., 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '02',
                  'name':'B_lateral_mix',
                  'pose': np.array([0., 0., 0.55, 0, 0., 0.]),
                  'twist': np.array([0.6, 0.6, 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '03',
                  'name':'C_pitch',
                  'pose': np.array([0., 0., 0.85, 0, -10.*DEG2RAD, 0.]),
                  'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '04',
                  'name':'C_pitch',
                  'pose': np.array([0., 0., 0.85, 0, +10.*DEG2RAD, 0.]),
                  'twist': np.array([0.5, 0., 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '05',
                  'name':'D_pitch_mix',
                  'pose': np.array([0., 0., 0.85, 0, -10.*DEG2RAD, 0.]),
                  'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
                  't_video': 0})

INIT_COND.append({'id': '06',
                  'name':'D_pitch_mix',
                  'pose': np.array([0., 0., 0.85, 0, +10.*DEG2RAD, 0.]),
                  'twist': np.array([0.5, 0.5, 0., 0., 0., 0.]),
                  't_video': 0})


# plot options
plt.ioff()
width_inches = 0.
height_inches = 0.
mm2inches = 0.0393701
for m in get_monitors():
    if m.is_primary:
        width_inches = m.width_mm*mm2inches
        height_inches = m.height_mm*mm2inches
        break

now = datetime.datetime.now()
now_s = str(now)
now_s = now_s.replace('-', '')
now_s = now_s.replace(' ', '_')
now_s = now_s.replace(':', '')
now_s = now_s[: now_s.find('.')]
save_path = os.environ['HOME'] + '/landing_controller/simulations/'+now_s
os.mkdir(save_path)
# plt.rcParams["figure.figsize"] = [7.00, 3.50]
# plt.rcParams["figure.autolayout"] = True


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    try:
        p.startController(world_name='camera_slow.world', additional_args=['gui:=False'])#, world_name="big_box.world")
        p.startupProcedure(check_contacts=True)  # overloaded method

        for init_cond in INIT_COND:

            q_des = p.qj_0.copy()
            qd_des = np.zeros_like(q_des)
            tau_ffwd = np.zeros_like(q_des)
            t_start_video = p.time.copy()

            init_cond['t_video'] = t_start_video
            if init_cond['id'] != '00':
                init_cond['t_video'] += INIT_COND[int(init_cond['id'])-1]['t_video']

            # reset controller
            p.initVars()
            print(colored("Starting simulation "+init_cond['id']+': '+init_cond['name'], "blue"))
            p.freezeBase(flag=True, basePoseW=init_cond['pose'])
            for k in range(100):
                p.send_command(q_des, qd_des, tau_ffwd)
            p.freezeBase(flag=True, basePoseW=init_cond['pose'], baseTwistW=init_cond['twist'])
            p.setGravity(-9.81)
            p.pause_physics_client()


            p.imu_utils.W_lin_vel = p.u.linPart(p.baseTwistW)
            lc = LandingController(robot=p.robot,
                                   dt=2 * p.dt,
                                   q0=np.hstack([p.u.linPart(p.basePoseW), p.quaternion, q_des]))



            lc.setCheckTimings(expected_touch_down_time=np.sqrt(2*p.basePoseW[2]/9.81)+p.time, clearance=0.1)



            ############
            # SIMULATE #
            ############
            #### FINITE STATE MACHINE #####
            # STATES
            # fsm_state = 0 - before APEX: kinematic adjustment (not used in simulations)
            # fsm_state = 1 - from APEX to TOUCH DOWN: compute landing trajectory + kinematic adjustment
            # fsm_state = 2 - from TOUCH DOWN to END: use last landing trajectory
            # fsm_state = 3 - quit
            fsm_state = 1
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
                # update kinematic and dynamic model
                p.updateKinematics()
                # update estimate on linear velocity
                p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)

                p.visualizeContacts()


                vcom_z_now = p.comTwistW[2]

                ###############################################
                # STATE 0 - before APEX: kinematic adjustment #
                ###############################################
                if fsm_state == 0:
                    # check if apex is reached
                    isApexReached = lc.apexReached(t=p.time,
                                                   sample=p.log_counter,
                                                   vel_z_pre=vcom_z_pre,
                                                   vel_z_now=vcom_z_now)
                    if isApexReached:
                        fsm_state += 1
                        p.contact_state[:] = False # to be sure that contact state is false when flying down
                    else:
                        # kinematic adjustment
                        lc.flyingUp_phase(p.b_R_w)
                        # set references
                        for i, leg in enumerate(lc.legs): # ['lf', 'rf', 'lh', 'lh']
                            q_des_leg, isFeasible  = p.IK.ik_leg(lc.B_feet_task[i],
                                                                 p.robot.model.getFrameId(leg+'_foot'),
                                                                 p.legConfig[leg][0],
                                                                 p.legConfig[leg][1])
                            if isFeasible:
                                p.u.setLegJointState(i, q_des_leg, q_des)
                        qd_des = np.zeros(p.robot.na)
                        tau_ffwd = p.self_weightCompensation()



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
                        p.leg_odom.reset(np.hstack([p.u.linPart(p.basePoseW), p.quaternion, p.q ]))
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
                                                          damp=1e-6,      # same for all the diag
                                                          update=i == 0)  # update Jacobians only with the first leg

                            p.u.setLegJointState(i, qd_leg_des, qd_des)

                        tau_ffwd = p.self_weightCompensation()

                        # save the base position at touch down
                        W_p_base_TD = p.u.linPart(p.basePoseW)
                        W_com_TD = p.u.linPart(p.comPoseW)

                #################################################################
                # STATE 2 - from TOUCH DOWN to END: use last landing trajectory #
                #################################################################
                if fsm_state == 2:
                    if lc.lp_counter > 2*lc.ref_k+100: # take a bit before quitting
                        fsm_state = 3
                    else:
                        p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.estimate_base_wrt_world(p.contact_state,
                                                                                              p.quaternion,
                                                                                              p.q,
                                                                                              p.u.angPart(p.baseTwistW),
                                                                                              p.qd)
                        # use last computed trajectories
                        lc.landed_phase(p.euler)

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
                        W_v_feet = -lc.T_v_com_ref_k
                        B_v_feet = p.b_R_w @ W_v_feet
                        for i, leg in enumerate(lc.legs):  # ['lf', 'rf', 'lh', 'lh']
                            qd_leg_des = p.IK.diff_ik_leg(q_des=q_des,
                                                          B_v_foot=B_v_feet,
                                                          foot_idx=p.robot.model.getFrameId(leg + '_foot'),
                                                          damp=1e-6,
                                                          update=i ==0)
                            p.u.setLegJointState(i, qd_leg_des, qd_des)


                        tau_ffwd = p.gravityCompensation() + p.self_weightCompensation()
                # print('T_feet_task: ', lc.T_feet_task)
                # print('T_feet_home: ', lc.T_feet_home)
                # print('B_feet_task: ', lc.B_feet_task)
                # print('B_feet_home: ', lc.B_feet_home)
                # print('-'*25)
                # finally, send commands
                p.send_command(q_des, qd_des, tau_ffwd)

                # save LC references for com, feet, zmp
                #p.u.linPart(p.basePoseW) +
                p.comPoseW_des[:3] =  lc.T_p_com_ref_k.reshape([3,])
                p.comPoseW_des[3:6] = lc.euler_des.reshape([3,])
                p.comTwistW_des[:3] = lc.T_v_com_ref_k.reshape([3,])

                for leg in range(4):
                    p.W_contacts_des[leg] = p.u.linPart(p.basePoseW) + p.b_R_w.T @ lc.B_feet_task[leg]
                    p.B_contacts_des[leg] = lc.B_feet_task[leg].copy()

                p.zmp[0] = lc.slip_dyn.zmp_xy[0]
                p.zmp[1] = lc.slip_dyn.zmp_xy[1]

            ####################
            # store some plots #
            ####################
            p.pause_physics_client()
            directory_path=save_path+'/sim'+init_cond['id']
            os.mkdir(directory_path)

            # joint position
            fig = plotJoint('position', 0, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log)
            fig.set_size_inches([width_inches, height_inches])
            plt.savefig(directory_path + '/q.png')
            plt.close()
            print(colored('Plot ' + directory_path + '/com.png saved', color='green'), flush=True)

            p.comPoseW_des_log[0, lc.jumping_data_times.touch_down.sample:] += p.comPoseW_log[0, lc.jumping_data_times.touch_down.sample]
            p.comPoseW_des_log[1, lc.jumping_data_times.touch_down.sample:] += p.comPoseW_log[1, lc.jumping_data_times.touch_down.sample]

            fig = plotCoM('position', 3, time_log=p.time_log.flatten(), basePoseW=p.comPoseW_log, des_basePoseW=p.comPoseW_des_log)
            fig.set_size_inches([width_inches, height_inches])
            plt.savefig(directory_path + '/com.png')
            plt.close()
            print(colored('Plot ' + directory_path + '/com.png saved', color='green'), flush=True)

            # feet position in w-frame and contact flag
            fig = plotFeet(10, time_log=p.time_log.flatten(), des_feet=p.W_contacts_des_log, act_feet=p.W_contacts_log,
                                  contact_states=p.contact_state_log)
            fig.set_size_inches([width_inches, height_inches])
            plt.savefig(directory_path + '/feet.png')
            plt.close()
            print(colored('Plot ' + directory_path + '/feet.png saved', color='green'), flush=True)

            # init cond file
            f = open(directory_path + "/init_cond.txt", "w")
            init_cond_str = ""
            init_cond_key = init_cond.keys()
            for key in init_cond_key:
                init_cond_str += key+': '
                init_cond_str += str(init_cond[key])
                init_cond_str += '\n'
            f.write( initCond2str(init_cond, speedUpDown) )
            f.close()
            print(colored('File ' + directory_path + '/init_cond.txt saved', color='green'), flush=True)
            p.unpause_physics_client()


    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        print(e)
    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

    # store all the init conds
    f = open(save_path + "/ALL_init_conds.txt", "w")
    for init_cond in INIT_COND:
        f.write(initCond2str(init_cond, speedUpDown)+'\n'+'-'*10+'\n')
    f.close()
    print(save_path + '/ALL_init_conds.txt saved')

    # save the video
    p.save_video(save_path, speedUpDown=speedUpDown)