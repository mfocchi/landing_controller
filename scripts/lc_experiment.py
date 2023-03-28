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
import base_controllers.params as conf
import datetime

np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)


ROBOT_NAME = 'go1'  # go1, solo, (aliengo)


if __name__ == '__main__':
    setSavePath(SETTINGS, 'experiments')

    if SETTINGS['save_log']:
        stdout = sys.stdout
        logfile = open(SETTINGS['save_path'] + "/log.txt", "w")
    try:
        p = Controller(ROBOT_NAME)

        p.startController(additional_args=['go0_conf:=standDown',
                                           'pid_discrete_implementation:=true'])

        p.startupProcedure()

        lm = LandingManager(p, SETTINGS)

        ret = lm.run(0, useIK=True, useWBC=True)



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

        if conf.plotting:
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.comPoseW_des_log, Pose_log=p.comPoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.comTwistW_des_log, Twist_log=p.comTwistW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)
            plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                      title='base', frame='W', sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)

            plotContacts('position', time_log=p.time_log, des_LinPose_log=p.B_contacts_des_log,
                         LinPose_log=p.B_contacts_log,
                         contact_states=p.contact_state_log, frame='B', sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)

            plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                         Forces_log=p.grForcesW_log, contact_states=p.contact_state_log, frame='W',
                         sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)

            plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, sharex=True, sharey=False,
                      start=lm.lc.lc_events.apex.sample, end=-1)
            plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True,
                      sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)
            plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, tau_ffwd_log=p.tau_ffwd_log,
                      tau_des_log=p.tau_fb_log, sharex=True, sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)

            plotWrenches('fb', 8, p.time_log, des_Wrench_fb_log=p.wrench_fbW_log)
            plotWrenches('ffwd', 9, p.time_log, des_Wrench_ffwd_log=p.wrench_ffW_log)
            plotWrenches('g', 10, p.time_log, des_Wrench_g_log=p.wrench_gW_log)

            plotFrameLinear('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_legOdom_log,
                            Twist_log=p.baseLinTwistImuW_log, title='Base velocity estimate', frame='W', sharex=True,
                            sharey=False, start=lm.lc.lc_events.apex.sample, end=-1)

        if input('Do you want to save plots? [y/n]')=='y':
            now = datetime.datetime.now()
            now_s = str(now)
            now_s = now_s.replace('-', '')
            now_s = now_s.replace(' ', '_')
            now_s = now_s.replace(':', '')
            now_s = now_s[: now_s.find('.')]
            SETTINGS['PLOTS']['directory_path'] = os.environ['LOCOSIM_DIR'] + '/landing_controller/experiments/' + now_s
            os.mkdir(SETTINGS['PLOTS']['directory_path'])
            makePlots(p, SETTINGS['FIGURES'], SETTINGS['PLOTS'], start=lm.lc.lc_events.apex.sample, end=-1, verbose = True)