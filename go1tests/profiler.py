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

import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()

ROBOT_NAME = 'go1'                         # go1, solo, (aliengo)



def updateKinematics(p):
    while p.time < 120:
        p.updateKinematics()
        #p.gravityCompensation()
        p.time += 0.002


if __name__ == '__main__':
    p = Controller(ROBOT_NAME)
    p.startController(use_ground_truth_contacts=False, additional_args=['go0_conf:=standDown'])
    p.pid = PidManager(p.joint_names)
    p.pid.setPDjoints(np.zeros(p.robot.na),
                      np.zeros(p.robot.na),
                      np.zeros(p.robot.na))
    q_des = p.q_des.copy()
    qd_des = np.zeros(12)
    tau_ffwd = np.zeros(12)

    try:
        #cProfile.run('updateKinematics(p)', filename='/home/froscia/ros_ws/src/landing_controller/go1tests/stats_new1.cprof')
        pr.enable()
        p.updateKinematics()
        pr.disable()
        ros.signal_shutdown("killed")
        p.deregister_node()
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    ps = pstats.Stats(pr)
    ps.dump_stats('/home/froscia/ros_ws/src/landing_controller/go1tests/stats_new1.cprof')
    # then go in go1test with the terminal and run
    # pyprof2calltree -k -i stats_new.cprof



