import time

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
    p.startController(additional_args=['gui:=False', 'go0_conf:=standDown'])#, world_name="big_box.world")
    uk_time = np.full(p.time_log.shape, np.nan)
    sc_time = np.full(p.time_log.shape, np.nan)
    p.startupProcedure()

    W_base_TD = np.array([0., 0., 0.259])
    conf_TD = np.hstack([W_base_TD, p.quaternion, p.q])
    p.leg_odom.reset(conf_TD)
    q_des = p.q_des.copy()
    qd_des = np.zeros(12)
    try:

        while not ros.is_shutdown():
            start = time.time()
            p.updateKinematics()
            tau_ffwd = p.gravityCompensation()
            uk_time[p.log_counter] = time.time()-start

            # p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)
            # p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.estimate_base_wrt_world(p.contact_state,
            #                                                                       p.quaternion,
            #                                                                       p.q,
            #                                                                       p.u.angPart(p.baseTwistW),
            #                                                                       p.qd)
            start = time.time()
            p.send_command(q_des, qd_des, tau_ffwd)
            sc_time[p.log_counter] = time.time() - start
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

    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(p.time_log[:p.log_counter-1], sc_time[:p.log_counter-1]*10e3)
    plt.plot(p.time_log[:p.log_counter-1], uk_time[:p.log_counter-1]*10e3)
    plt.legend(['updateKinematics', 'send_command'])
    plt.xlabel('time [s]')
    plt.ylabel('computation time [ms]')
    plt.title('Desktop PC, in Docker')


