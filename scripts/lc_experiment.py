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
sys.path.append('../')
from base_controllers.quadruped_controller import Controller
from landing_controller.controller.landing_manager import LandingManager
from landing_controller.controller import SETTINGS
from landing_controller.controller.utility import *


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

        ret = lm.run(0, useIK=True, useWBC=False)
        # while not ros.is_shutdown():
        #     p.send_command(p.q_des, p.qd_des, p.gravityCompensation())



    except (ros.ROSInterruptException, ros.service.ServiceException) as e:
        if SETTINGS['save_log']:
            logfile.close()
            sys.stdout = stdout
        print(e)
    finally:
        # if SETTINGS['save_log']:
        #     logfile.close()
        #     sys.stdout = stdout
        # ros.signal_shutdown("killed")
        # p.deregister_node()
        # os.system("killall rosmaster rviz gzserver gzclient ros_control_node")

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

        plotCoM('position', 0, time_log=p.time_log[lm.lc.lc_events.apex.sample:],
                des_basePoseW=p.comPoseW_des_log[:, lm.lc.lc_events.apex.sample:],
                basePoseW=p.comPoseW_log[:, lm.lc.lc_events.apex.sample:], title='CoM')

        plotCoM('velocity', 1, time_log=p.time_log[lm.lc.lc_events.apex.sample:],
                des_baseTwistW=p.comTwistW_des_log[:, lm.lc.lc_events.apex.sample:],
                baseTwistW=p.comTwistW_log[:, lm.lc.lc_events.apex.sample:], title='CoM')

        plotCoMLinear('imu vel est', 2, p.time_log[lm.lc.lc_events.apex.sample:],
                      plot_var_des_log=p.baseTwistW_legOdom_log[:3, lm.lc.lc_events.apex.sample:],
                      plot_var_log=p.baseLinTwistImuW_log[:, lm.lc.lc_events.apex.sample:])
        plotJoint('position', 3, p.time_log[lm.lc.lc_events.apex.sample:],
                  q_log=p.q_log[:, lm.lc.lc_events.apex.sample:],
                  q_des_log=p.q_des_log[:, lm.lc.lc_events.apex.sample:])

        # plotGRFs(4, time_log=p.time_log[lm.lc.lc_events.apex.sample:], act_forces = p.grForcesW_log[:, lm.lc.lc_events.apex.sample:],
        #          des_forces=p.grForcesW_des_log[:, lm.lc.lc_events.apex.sample:], title='world')

        plotGRFs_withContacts(5, time_log=p.time_log[lm.lc.lc_events.apex.sample:],
                 act_forces=p.grForcesW_log[:, lm.lc.lc_events.apex.sample:],
                 des_forces=p.grForcesW_des_log[:, lm.lc.lc_events.apex.sample:],
                              contact_states=p.contact_state_log[:, lm.lc.lc_events.apex.sample:])

        plotJoint('velocity', 6, p.time_log[lm.lc.lc_events.apex.sample:],
                  qd_log=p.qd_log[:, lm.lc.lc_events.apex.sample:],
                  qd_des_log=p.qd_des_log[:, lm.lc.lc_events.apex.sample:])

        plotWrenches('fb', 8, p.time_log[lm.lc.lc_events.apex.sample:],
                     wrench_fb_log=p.wrench_fbW_log[:, lm.lc.lc_events.apex.sample:])
        plotWrenches('ffwd', 9, p.time_log[lm.lc.lc_events.apex.sample:],
                     wrench_ffwd_log=p.wrench_ffW_log[:, lm.lc.lc_events.apex.sample:])
        plotWrenches('g', 10, p.time_log[lm.lc.lc_events.apex.sample:],
                     wrench_g_log=p.wrench_gW_log[:, lm.lc.lc_events.apex.sample:])