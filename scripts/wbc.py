# this is a copy of wbc.py
import time

from base_controllers.quadruped_controller import Controller
import rospy as ros
import numpy as np

from base_controllers.utils.common_functions import *
from base_controllers.utils.math_tools import polynomialRef
import matplotlib.pyplot as plt
import os

import pinocchio as pin

import base_controllers.params as conf

import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()

robotName = "go1"


#################
# Configuration #
#################
test = {}
test['duration'] = 8.
test['stabilize'] = 5.
test['usePid'] = 5.
test['amp']  = np.array([0.00, 0., 0.0, 0.0, 0.0, 0.1])
test['phase']  = np.array([0., 0., np.pi/2, 0., 0, 0.])
test['delta']  = np.array([0.00, 0., 0.0, 0., 0.0, 0.])
test['freq'] = 1/8
test['typeWBC'] = 'projection'
# test['typeWBC'] = 'qp'
# if use qp, the following must be set
test['normals'] = [np.array([0.,0.,1.])]*4
test['friction_coeffs'] = [0.8]*4
 

if __name__ == '__main__':
    p = Controller(robotName)
    test['pulse'] = 2*np.pi*test['freq']
    test['pulse2'] = test['pulse']**2
    try:

        p.startController(world_name='fast.world', use_ground_truth_contacts=False, additional_args=['gui:=False', 'go0_conf:=standDown'])
        # p.startController(use_ground_truth_contacts=False,
        #                   additional_args=['gui:=True', 'go0_conf:=standDown'])
        # p.startController(additional_args=['gui:=False', 'go0_conf:=standDown'])
        p.startupProcedure()  # overloaded method

        if test['typeWBC'] == 'qp':
            p.setWBCConstraints(test['normals'], test['friction_coeffs'])

        # Reset reference to actual value
        state = 0
        time_init = p.time.copy()
        p.x0 = p.comPoseW.copy()
        p.comAccW_des = np.zeros(6)

        ####
        # end of sinusoid - start of polynomial
        time_change = test['duration'] - (1 / 4) * (1 / test['freq'])
        comPoseW_des = p.x0 + test['amp'] * np.sin(test['pulse'] * time_change + test['phase']) - test['delta']
        comTwistW_des = test['pulse'] * test['amp'] * np.cos(test['pulse'] * time_change + test['phase'])
        comAccW_des = -test['pulse2'] * test['amp'] * np.sin(test['pulse'] * time_change + test['phase'])

        pos, vel, acc = polynomialRef(comPoseW_des, p.x0,
                                      comTwistW_des, np.zeros(6),
                                      comAccW_des, np.zeros(6),
                                      (1 / 4) * (1 / test['freq']))

        print('start!')

        p.q_des = p.q.copy()
        p.qd_des = p.qd.copy()

        # Control loop
        while not ros.is_shutdown():
            #print(p.time, state)
            # update the kinematics
            p.updateKinematics()
            # Reference Generation
            if state == 0:
                if p.time - time_init < test['usePid']:
                    p.gravityCompensation()
                else:
                    state += 1
                    p.x0 = p.comPoseW.copy()
                    # p.pid.setPDjoints(conf.robot_params[self.robot_name]['kd_real_wbc'],
                    #                   conf.robot_params[self.robot_name]['kp_real_wbc'],
                    #                   np.zeros(self.robot.na))
                    p.pid.setPDjoints(np.ones(p.robot.na)*5,
                                      np.ones(p.robot.na)*0.02,
                                      np.zeros(p.robot.na))
                    # p.pid.setPDjoints(np.ones(p.robot.na) * 0.,
                    #                   np.ones(p.robot.na) * 0.,
                    #                   np.zeros(p.robot.na))
                    time_stabilize = p.time.copy()

                    p.W_contacts_des = p.W_contacts.copy()

                    print('starting wbc ' + str(p.time) + " [s]")
            elif state == 1:
                if p.time - time_stabilize < test['stabilize']:

                    p.comPoseW_des = p.x0
                    p.comTwistW_des[:] = 0.
                    p.comAccW_des[:] = 0.

                    p.Wcom2Joints_des()

                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])
                else:
                    time_sinusoid = p.time.copy()
                    print('starting sinusoid ' + str(p.time) + " [s]")
                    state += 1

            elif state == 2:
                if p.time - time_sinusoid < test['duration']-(1/4)*(1/test['freq']):
                    # com ref
                    p.comPoseW_des = p.x0 + test['amp'] * np.sin( test['pulse']*(p.time - time_sinusoid) + test['phase']) -  test['delta']
                    p.comTwistW_des = test['pulse'] * test['amp'] * np.cos( test['pulse']*(p.time - time_sinusoid) + test['phase'] )
                    p.comAccW_des = -test['pulse2'] * test['amp'] * np.sin( test['pulse']*(p.time - time_sinusoid) + test['phase'] )

                    p.Wcom2Joints_des()
                    
                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])
                else:
                    time_safe_stop = p.time
                    print('safe stop ' + str(p.time) + " [s]")
                    state += 1

            elif state == 3:
                if p.time - time_safe_stop < (1/4)*(1/test['freq']):
                    time = p.time - time_safe_stop
                    p.comPoseW_des = pos(time)
                    p.comTwistW_des = vel(time)
                    p.comAccW_des = acc(time)

                    p.Wcom2Joints_des()
                    
                    p.WBC(p.comPoseW_des,
                          p.comTwistW_des,
                          p.comAccW_des,
                          comControlled=True,
                          type=test['typeWBC'])

                else:
                    state += 1
                    p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp'],
                                      conf.robot_params[p.robot_name]['kd'],
                                      conf.robot_params[p.robot_name]['ki'])
                    p.q_des = p.q.copy()
                    p.qd_des[:] = 0.
                    print('restore PDs ' + str(p.time) +" [s]")

            else:
                p.gravityCompensation()

            # send desired command to the ros controller
            p.send_command()



    except (ros.ROSInterruptException, ros.service.ServiceException):

        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        os.system("killall rosmaster rviz gzserver gzclient ros_control_node")


        if conf.plotting:
            plotCoM('position', 0, time_log=p.time_log, des_basePoseW=p.comPoseW_des_log,
                    basePoseW=p.comPoseW_log)
            plotCoM('velocity', 1, time_log=p.time_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            plotCoM('position', 2, time_log=p.time_log, des_basePoseW=p.basePoseW_des_log,
                    basePoseW=p.basePoseW_log)
            plotCoM('velocity', 3, time_log=p.time_log, des_baseTwistW=p.comTwistW_des_log,
                    baseTwistW=p.comTwistW_log)
            plotGRFs(4, p.time_log, p.grForcesW_log, p.grForcesW_des_log)
            plotJoint('position', 5, p.time_log, q_log=p.q_log, q_des_log=p.q_des_log)
            plotJoint('velocity', 6, p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log)
            plotJoint('torque',7, p.time_log, tau_log=p.tau_log, tau_ffwd_log = p.tau_ffwd_log, tau_des_log=p.tau_fb_log)

            plotWrenches('fb', 8, p.time_log, wrench_fb_log=p.wrench_fbW_log)
            plotWrenches('ffwd', 9, p.time_log, wrench_ffwd_log=p.wrench_ffW_log)
            plotWrenches('g', 10, p.time_log, wrench_g_log=p.wrench_gW_log)

            plotWBC_fb('fb wrench and com', 11, p.time_log, wrench_fb_log=p.wrench_fbW_log, comPose_des_log=p.comPoseW_des_log, comPose_log=p.comPoseW_log,
                       comTwist_des_log=p.comTwistW_des_log, comTwist_log=p.comTwistW_log)

            plotFeet(12, p.time_log, des_feet=p.B_contacts_des_log, act_feet=p.B_contacts_log)



