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


from base_controllers.utils.pidManager import PidManager

ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    p.startController(additional_args=['go0_conf:=standDown'])#, world_name="big_box.world")
    p.pid = PidManager(p.joint_names)
    p.pid.setPDjoints(np.zeros(p.robot.na),
                      np.zeros(p.robot.na),
                      np.zeros(p.robot.na))

    try:

        while not ros.is_shutdown():
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)

            p.send_command(np.zeros(p.robot.na), np.zeros(p.robot.na), np.zeros(p.robot.na))
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()




    from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs

    plotJoint('position', 0, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
              qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    plotJoint('velocity', 1, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
               qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log)
    plotJoint('torque', 2, time_log=p.time_log.flatten(), q_log=p.q_log, q_des_log=p.q_des_log, qd_log=p.qd_log,
              qd_des_log=p.qd_des_log, tau_ffwd_log=p.tau_ffwd_log, tau_log=p.tau_log, tau_des_log=p.tau_des_log)
    plotCoM('position', 3, time_log=p.time_log.flatten(), basePoseW=p.basePoseW_log)
    # plotCoM('velocity', 4, time_log=p.time_log.flatten(), baseTwistW=p.baseTwistW_log)
    plotGRFs(6, time_log=p.time_log.flatten(), des_forces=p.grForcesW_des_log, act_forces=p.grForcesW_log)


