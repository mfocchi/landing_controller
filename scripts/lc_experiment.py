# This script is similar to lc_simulation.py but is tailored for real robot. The differences are:
# it uses functions for real robot (just setting real_robot:True in params.py),
# it does not call reset,
# (not used yet)


import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import rospy as ros
import sys
import os
from base_controllers.quadruped_controller import Controller
from controller.landing_manager import LandingManager
from controller import SETTINGS
from controller.utility import *


np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)


ROBOT_NAME = 'go1'  # go1, solo, (aliengo)
world_name = 'slow.world'
use_gui = True


if __name__ == '__main__':
    # if SETTINGS['save_log']:
    #     stdout = sys.stdout
    #     logfile = open(SETTINGS['save_path'] + "/log.txt", "w")

    try:
        p = Controller(ROBOT_NAME)
        # p.startController(world_name='fast.world', use_ground_truth_contacts=False,
        #                   additional_args=['gui:=true', 'go0_conf:=standDown', 'pid_discrete_implementation:=true'])

        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=False,
                          additional_args=['gui:='+str(use_gui),
                                           'go0_conf:=standDown', 'pid_discrete_implementation:=true'])

        p.startupProcedure()


        lm = LandingManager(p, SETTINGS)

        ret = lm.run(0)
        # while not ros.is_shutdown():
        #     p.send_command(p.q_des, p.qd_des, p.gravityCompensation())



    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        print(e)
    finally:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        ros.signal_shutdown("killed")
        p.deregister_node()
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

        # save all for later analysis
        if not p.real_robot:
            if SETTINGS['WORKSPACE']['save']:
                saveWorkspace(SETTINGS['save_path'])

            # store all the init conds
            if SETTINGS['INIT_CONDS']['save_all']:
                saveAllInitConds(SETTINGS['save_path'], SETTINGS['SIMS'], SETTINGS['VIDEO']['speedUpDown'], SETTINGS['verbose'])


            if SETTINGS['VIDEO']['save']:
                # save the video
                p.save_video(SETTINGS['save_path'], start_file=init_video_frame, speedUpDown=SETTINGS['VIDEO']['speedUpDown'])