import numpy as np
import matplotlib
matplotlib.use('TkAgg')
np.set_printoptions(linewidth=np.inf,  # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,  # ... 4
                    sign=' ',  # print space if sign is plus
                    suppress=True,  # suppress scientific notation
                    threshold=np.inf)

from base_controllers.quadruped_controller import Controller
import rospy as ros
from base_controllers.utils.common_functions import *
import base_controllers.params as conf

ROBOT_NAME = 'go1'


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)

    p.startController(world_name='fast.world', additional_args=['go0_conf:=standDown'])
    p.startupProcedure()

    try:
        while not ros.is_shutdown():
            p.updateKinematics()
            p.send_command(p.q_des, np.zeros(p.robot.na), p.self_weightCompensation())
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

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

        plotCoMLinear('imu acceleration', 15, p.time_log, plot_var_log=p.baseLinAccW_log)
        plotCoMLinear('imu vel est', 16, p.time_log, plot_var_des_log=p.baseTwistW_log[:3, :],
                      plot_var_log=p.baseLinTwistImuW_log)

        plt.figure()
        ax0 = plt.subplot(5, 1, 1)
        ax0.plot(p.time_log, p.baseLinAccW_log[2,:].T)
        ax0.set_ylabel('base z acc [m/s^2]')
        ax0.grid()
        ax1 = plt.subplot(5, 1, 2, sharex=ax0)
        ax1.plot(p.time_log, p.contact_state_log[0, :].T)
        ax1.set_ylabel('LF')
        ax1.grid()
        ax2 = plt.subplot(5, 1, 3, sharex=ax0)
        ax2.plot(p.time_log, p.contact_state_log[1, :].T)
        ax2.set_ylabel('LH')
        ax2.grid()
        ax3 = plt.subplot(5, 1, 4, sharex=ax0)
        ax3.plot(p.time_log, p.contact_state_log[2, :].T)
        ax3.set_ylabel('RF')
        ax3.grid()
        ax4 = plt.subplot(5, 1, 5, sharex=ax0)
        ax4.plot(p.time_log, p.contact_state_log[3, :].T)
        ax4.set_ylabel('RH')
        ax4.set_xlabel('Time [s]')
        ax4.grid()
        plt.show()
