import numpy as np
import matplotlib
matplotlib.use('TkAgg')
np.set_printoptions(linewidth=np.inf,  # number of characters per line befor new line
                    floatmode='fixed',  # print fixed numer of digits ...
                    precision=4,  # ... 4
                    sign=' ',  # print space if sign is plus
                    suppress=True,  # suppress scientific notation
                    threshold=np.inf)

from base_controllers.quadruped_controller import Controller
import rospy as ros
import time
from base_controllers.utils.common_functions import *

from base_controllers.utils.pidManager import PidManager
import base_controllers.params as conf

ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    p.startController(use_ground_truth_pose=False, use_ground_truth_contacts=False)
    p.pid = PidManager(p.joint_names)
    p.pid.setPDjoints(np.zeros(p.robot.na),
                      np.zeros(p.robot.na),
                      np.zeros(p.robot.na))
    q_des = p.q_des.copy()
    qd_des = np.zeros(12)
    tau_ffwd = np.zeros(12)

    try:
        while p.imu_utils.counter < p.imu_utils.timeout:
            p.updateKinematics()
            p.imu_utils.IMU_bias_estimation(p.b_R_w, p.baseLinAccB)
            p.tau_ffwd[:] = 0.
            p.send_command(q_des, qd_des, tau_ffwd)
        print('Imu bias estimation compleated')
        while not ros.is_shutdown():
            p.updateKinematics()

            p.send_command(q_des, qd_des, tau_ffwd)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()



    except (ros.ROSInterruptException, ros.service.ServiceException):

        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        if conf.plotting:
            plotCoM('position', 0, time_log=p.time_log, des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log, title='CoM')
            plotCoM('velocity', 1, time_log=p.time_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log, title='CoM')
            plotCoM('position', 2, time_log=p.time_log, des_basePoseW=p.basePoseW_des_log,
                    basePoseW=p.basePoseW_log, title='base')
            plotCoM('velocity', 3, time_log=p.time_log, des_baseTwistW=p.baseTwistW_des_log,
                    baseTwistW=p.baseTwistW_log, title='base')
            plotGRFs(4, p.time_log, p.grForcesW_log, p.grForcesW_des_log, title='world')
            plotGRFs_withContacts(4, p.time_log, p.grForcesW_wbc_log, p.grForcesW_log, p.contact_state_log)

            plotJoint('position', 5, p.time_log, q_log=p.q_log, q_des_log=p.q_des_log)
            plotJoint('velocity', 6, p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log)
            plotJoint('torque', 7, p.time_log, tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log, tau_des_log=p.tau_fb_log)

            plotWrenches('fb', 8, p.time_log, wrench_fb_log=p.wrench_fbW_log)
            plotWrenches('ffwd', 9, p.time_log, wrench_ffwd_log=p.wrench_ffW_log)
            plotWrenches('g', 10, p.time_log, wrench_g_log=p.wrench_gW_log)

            plotWBC_fb('fb wrench and com', 11, p.time_log, wrench_fb_log=p.wrench_fbW_log,
                       comPose_des_log=p.comPoseW_des_log, comPose_log=p.comPoseW_log,
                       comTwist_des_log=p.comTwistW_des_log, comTwist_log=p.comTwistW_log)

            plotFeet(12, p.time_log, des_feet=p.B_contacts_des_log, act_feet=p.B_contacts_log,
                     contact_states=p.contact_state_log)

            plotCoMLinear('leg odom: base', 13, p.time_log, plot_var_log=p.basePoseW_legOdom_log)

            plotSingleJoint('torque', 14, 7, p.time_log, tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log,
                            tau_des_log=p.tau_fb_log)

            plotCoMLinear('imu acceleration', 15, p.time_log, plot_var_log=p.W_base_lin_acc_log)
            plotCoMLinear('imu vel est', 15, p.time_log, plot_var_log=p.W_lin_vel_log)