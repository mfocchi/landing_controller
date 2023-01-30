import time

from base_controllers.quadruped_controller import Controller
import rospy as ros
import numpy as np
from numpy import nan
import copy

# L5 Controller specific
from base_controllers.utils.common_functions import plotCoM, plotGRFs, plotConstraitViolation, plotJoint
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
import os

import base_controllers.params as conf

robotName = "go1"


#################
# Configuration #
#################
test = {}
test['duration'] = 3.
test['stabilize'] = .1
test['amp']  = np.array([0.0, 0., 0.0, 0., 0., 0.])
test['freq'] = np.array([0.1, 0., 0.5, 0., 0., 0.])
test['typeWBC'] = 'projection'
#test['typeWBC'] = 'qp'
# if use qp, the following must be set
test['normals'] = [np.array([0.,0.,1.])]*4
test['friction_coeffs'] = [0.8]*4




if __name__ == '__main__':
    p = Controller(robotName)
    UK_log =  np.zeros(conf.robot_params[p.robot_name]['buffer_size']) * np.nan
    WBC_log = np.zeros(conf.robot_params[p.robot_name]['buffer_size']) * np.nan
    pub_log = np.zeros(conf.robot_params[p.robot_name]['buffer_size']) * np.nan
    test['pulse'] = 2*np.pi*test['freq']
    test['pulse2'] = test['pulse']**2
    try:

        p.startController(world_name='slow.world',
                          use_ground_truth_pose=False, use_ground_truth_contacts=False,
                          additional_args=['gui:=True', 'go0_conf:=standDown'])
        p.startupProcedure()  # overloaded method
        p.leg_odom.reset(np.hstack([0., 0., 0.259, p.quaternion, p.q]))

        if test['typeWBC'] == 'qp':
            p.setWBCConstraints(test['normals'], test['friction_coeffs'])

        p.updateKinematics()
        # Reset reference to actual value
        p.pid.setPDs(0.0, 0.0, 0.0)
        time_init = p.time.copy()
        p.x0=p.basePoseW.copy()
        p.x0[2] = 0.24215
        p.x0[3:] = 0.
        print('start!')

        # Control loop
        while p.time-time_init < test['duration']:
            # p.pr.enable()
            # update the kinematics
            start_time = time.time()
            p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.estimate_base_wrt_world(p.contact_state,
                                                                                  p.quaternion,
                                                                                  p.q,
                                                                                  p.u.angPart(p.baseTwistW),
                                                                                  p.qd)
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)
            UK_log[p.log_counter]= time.time()-start_time
            # Reference Generation
            if p.time-time_init < test['stabilize']:
                p.comPoseW_des = p.x0
                p.comTwistW_des = np.zeros(6)
                p.comAccW_des = np.zeros(6)
            else:
                p.comPoseW_des = p.x0 + test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )
                p.comTwistW_des = test['pulse']*test['amp'] * np.cos( test['pulse']*(p.time-time_init-test['stabilize']) )
                p.comAccW_des = -test['pulse2'] * test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )
            start_time = time.time()

            p.tau_ffwd = p.WBC(p.comPoseW_des, p.comTwistW_des, p.comAccW_des, comControlled = True, type=test['typeWBC'])
            WBC_log[p.log_counter]= time.time()-start_time


            # plot actual (green) and desired (blue) contact forces
            start_time = time.time()
            for leg in range(4):
                p.ros_pub.add_marker(p.W_contacts[leg])
                p.ros_pub.add_arrow(p.W_contacts[leg],
                                    p.u.getLegJointState(leg, p.grForcesW / (5 * p.robot.robotMass)),
                                    "green")
                p.ros_pub.add_arrow(p.W_contacts[leg],
                                    p.u.getLegJointState(leg, p.grForcesW_des / (5 * p.robot.robotMass)),
                                    "blue")
            p.ros_pub.publishVisual()
            pub_log[p.log_counter] = time.time() - start_time



            # send desired command to the ros controller
            p.send_command(p.q_des, p.qd_des, p.tau_ffwd)



        p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],
                          np.zeros(p.robot.na))




    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        if conf.plotting:


            plotCoM('position', 10, time_log=p.time_log.flatten(), des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            plotCoM('velocity', 11, time_log=p.time_log.flatten(), des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            # plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
            plotJoint('torque',4, p.time_log.flatten(), p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)

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

            plt.figure()
            ax = plt.subplot(3,1,1)
            ax.plot(p.time_log.T, UK_log)
            ax.set_title('update kinematics')
            ax.set_ylabel('comp time [s]')
            ax = plt.subplot(3,1,2)
            ax.plot(p.time_log.T, WBC_log)
            ax.set_title('Whole Body Controller')
            ax.set_ylabel('comp time [s]')
            ax = plt.subplot(3, 1, 3)
            ax.plot(p.time_log.T, pub_log)
            ax.set_title('Publisher')
            ax.set_xlabel('iter #')
            ax.set_ylabel('comp time [s]')

            for log in [UK_log, WBC_log, pub_log]:
                nonzero = UK_log[np.nonzero(log)]
                avg=np.average(nonzero)
                std = np.std(nonzero)



