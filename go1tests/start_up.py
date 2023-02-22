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


from base_controllers.utils.pidManager import PidManager

ROBOT_NAME = 'go1'


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    # p.startController(world_name='fast.world',
    #                   use_ground_truth_pose=True,
    #                   use_ground_truth_contacts=False,
    #                   additional_args=['gui:=True',
    #                                    'go0_conf:=standDown'])

    p.startController(additional_args=['go0_conf:=standDown'])
    p.startupProcedure()

    try:

        while not ros.is_shutdown():
            p.updateKinematics()
            p.send_command(p.q_des, np.zeros(p.robot.na), p.gravityCompensation())
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    from base_controllers.utils.common_functions import *

    plotJoint('position', 5, p.time_log, q_log=p.q_log, q_des_log=p.q_des_log)
    plotJoint('velocity', 6, p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log)
    plotJoint('torque',7, p.time_log, tau_log=p.tau_log, tau_ffwd_log = p.tau_ffwd_log, tau_des_log=p.tau_fb_log)
