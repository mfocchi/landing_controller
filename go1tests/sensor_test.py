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

from base_controllers.utils.pidManager import PidManager

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
            p.imu_utils.IMU_bias_estimation(p.b_R_w, p.B_imu_lin_acc)
            p.tau_ffwd[:] = 0.
            p.send_command(q_des, qd_des, tau_ffwd)
        print('Imu bias estimation completed')
        while not ros.is_shutdown():
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)

            p.send_command(q_des, qd_des, tau_ffwd)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()






    from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs

    # plotJoint('position', 0, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
    #           qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    # plotJoint('velocity', 1, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
    #            qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    # plotJoint('torque', 2, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
    #           qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log, tau_des_log=p.tau_des_log)
    plotCoM('position', 3, time_log=p.time_log.flatten(), basePoseW=p.basePoseW_log)
    plotCoM('velocity', 4, time_log=p.time_log.flatten(), baseTwistW=p.baseTwistW_log)
    #plotGRFs(6, time_log=p.time_log.flatten(), des_forces=p.grForcesW_des_log, act_forces=p.grForcesW_log)

    import matplotlib.pyplot as plt

    plt.figure(11)
    plt.plot(p.imu_utils.IMU_accelerometer_bias_log.T)
    plt.legend(['x', 'y', 'z'])
