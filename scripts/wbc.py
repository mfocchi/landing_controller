# this is a copy of wbc.py
import time

from base_controllers.quadruped_controller import Controller
import rospy as ros
import numpy as np

from base_controllers.utils.common_functions import *
import matplotlib.pyplot as plt
import os

import base_controllers.params as conf

robotName = "go1"


#################
# Configuration #
#################
test = {}
test['duration'] = 300.
test['stabilize'] = 10.
test['usePid'] = 5.
test['amp']  = np.array([0.0, 0., 0.05, 0., 0., 0.])
test['freq'] = np.array([0.0, 0., 1/3, 0., 0., 0.])
test['typeWBC'] = 'projection'
#test['typeWBC'] = 'qp'
# if use qp, the following must be set
test['normals'] = [np.array([0.,0.,1.])]*4
test['friction_coeffs'] = [0.8]*4




if __name__ == '__main__':
    p = Controller(robotName)
    test['pulse'] = 2*np.pi*test['freq']
    test['pulse2'] = test['pulse']**2
    try:

        p.startController(world_name='fast.world', use_ground_truth_contacts=False, additional_args=['gui:=True', 'go0_conf:=standDown'])
        # p.startController(use_ground_truth_contacts=False,
        #                   additional_args=['gui:=True', 'go0_conf:=standDown'])
        # p.startController(additional_args=['gui:=False', 'go0_conf:=standDown'])
        p.startupProcedure()  # overloaded method
        p.leg_odom.reset(np.hstack([0., 0., p.robot_height, p.quaternion, p.q]))
        p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.base_in_world(p.contact_state,
                                                                    p.W_contacts,
                                                                    p.wJ,
                                                                    p.u.angPart(p.baseTwistW),
                                                                    p.qd)
        if test['typeWBC'] == 'qp':
            p.setWBCConstraints(test['normals'], test['friction_coeffs'])

        # Reset reference to actual value

        time_init = p.time.copy()
        # p.x0=p.comPoseW.copy()
        # p.x0[:2] = 0
        # p.x0[3:] = 0
        p.x0 = np.zeros(6)

        print('start!')

        changeController = True

        # Control loop
        while not ros.is_shutdown():
            # p.pr.enable()
            # update the kinematics
            p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.base_in_world(p.contact_state,
                                                                        p.W_contacts,
                                                                        p.wJ,
                                                                        p.u.angPart(p.baseTwistW),
                                                                        p.qd)
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)
            # Reference Generation
            if p.time - time_init < test['usePid']:
                p.tau_ffwd = p.gravityCompensation()
            else:
                if p.time-time_init < test['stabilize']:
                    if changeController:
                        p.x0 = p.comPoseW.copy()
                        p.pid.setPDs(0.0, 0.0, 0.0)
                        changeController = False
                    p.comPoseW_des = p.x0
                    p.comTwistW_des = np.zeros(6)
                    p.comAccW_des = np.zeros(6)
                else:
                    p.comPoseW_des = p.x0 + test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )
                    p.comTwistW_des = test['pulse']*test['amp'] * np.cos( test['pulse']*(p.time-time_init-test['stabilize']) )
                    p.comAccW_des = -test['pulse2'] * test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )

                p.tau_ffwd = p.WBC(p.comPoseW_des, p.comTwistW_des, p.comAccW_des, comControlled = True, type=test['typeWBC'])

            # send desired command to the ros controller
            p.send_command(p.q_des, p.qd_des, p.tau_ffwd)



        p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],
                          np.zeros(p.robot.na))

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        if conf.plotting:


            plotCoM('position', 0, time_log=p.time_log.flatten(), des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            plotCoM('velocity', 1, time_log=p.time_log.flatten(), des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            # plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
            plotJoint('torque',3, p.time_log.flatten(), p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)

            plt.figure()
            plt.title('wrench fb')
            plt.plot(p.time_log.T, p.wrench_fbW_log.T)
            plt.legend(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])

            plt.figure()
            plt.title('wrench ff')
            plt.plot(p.time_log.T, p.wrench_ffW_log.T)
            plt.legend(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])

            plt.figure()
            plt.title('wrench gravity')
            plt.plot(p.time_log.T, p.wrench_gW_log.T)
            plt.legend(['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])



