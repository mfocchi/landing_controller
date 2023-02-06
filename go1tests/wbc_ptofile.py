# this is a copy of wbc.py
import time

from base_controllers.quadruped_controller import Controller
import rospy as ros
import numpy as np

from base_controllers.utils.common_functions import plotCoM, plotJoint
import matplotlib.pyplot as plt
import os

import base_controllers.params as conf

robotName = "go1"


#################
# Configuration #
#################
test = {}
test['duration'] = 3.
test['stabilize'] = 1.
test['amp']  = np.array([0.0, 0., 0.05, 0., 0., 0.])
test['freq'] = np.array([0.0, 0., 0.5, 0., 0., 0.])
test['typeWBC'] = 'projection'
#test['typeWBC'] = 'qp'
# if use qp, the following must be set
test['normals'] = [np.array([0.,0.,1.])]*4
test['friction_coeffs'] = [0.8]*4

import cProfile, pstats, io
from pstats import SortKey
# pr = cProfile.Profile(timeunit=1e-6)


if __name__ == '__main__':
    p = Controller(robotName)
    test['pulse'] = 2*np.pi*test['freq']
    test['pulse2'] = test['pulse']**2
    try:

        #p.startController(world_name='slow.world', additional_args=['gui:=True', 'go0_conf:=standDown'])
        p.startController(world_name='slow.world', additional_args=['gui:=False', 'go0_conf:=standDown'])

        p.startupProcedure()  # overloaded method
        p.leg_odom.reset(np.hstack([0., 0., p.robot_height, p.quaternion, p.q]))
        p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.base_in_world(p.contact_state,
                                                                      p.W_contacts,
                                                                      p.wJ,
                                                                      p.u.angPart(p.baseTwistW),
                                                                      p.qd)

        if test['typeWBC'] == 'qp':
            p.setWBCConstraints(test['normals'], test['friction_coeffs'])

        p.updateKinematics()
        # Reset reference to actual value

        time_init = p.time.copy()
        # p.x0=p.comPoseW.copy()
        # p.x0[:2] = 0
        # p.x0[3:] = 0
        p.x0 = np.zeros(6)
        p.x0[2] = 0.240
        print('start!')
        p.pid.setPDs(0.0, 0.0, 0.0)
        # Control loop
        while p.time-time_init < test['duration']:
            # p.pr.enable()
            # update the kinematics
            # pr.enable()
            p.w_p_b_legOdom, p.w_v_b_legOdom = p.leg_odom.base_in_world(p.contact_state,
                                                                          p.W_contacts,
                                                                          p.wJ,
                                                                          p.u.angPart(p.baseTwistW),
                                                                          p.qd)
            p.updateKinematics()
            p.imu_utils.compute_lin_vel(p.W_base_lin_acc, p.loop_time)
            # Reference Generation
            if p.time-time_init < test['stabilize']:
                p.comPoseW_des = p.x0
                p.comTwistW_des = np.zeros(6)
                p.comAccW_des = np.zeros(6)
            else:
                p.comPoseW_des = p.x0 + test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )
                p.comTwistW_des = test['pulse']*test['amp'] * np.cos( test['pulse']*(p.time-time_init-test['stabilize']) )
                p.comAccW_des = -test['pulse2'] * test['amp'] * np.sin( test['pulse']*(p.time-time_init-test['stabilize']) )


            p.tau_ffwd = p.WBC(p.comPoseW_des, p.comTwistW_des, p.comAccW_des, comControlled = True, type=test['typeWBC'])



            # plot actual (green) and desired (blue) contact forces
            # start_time = time.time()
            # for leg in range(4):
            #     p.ros_pub.add_marker(p.W_contacts[leg])
            #     p.ros_pub.add_arrow(p.W_contacts[leg],
            #                         p.u.getLegJointState(leg, p.grForcesW / (5 * p.robot.robotMass)),
            #                         "green")
            #     p.ros_pub.add_arrow(p.W_contacts[leg],
            #                         p.u.getLegJointState(leg, p.grForcesW_des / (5 * p.robot.robotMass)),
            #                         "blue")
            # p.ros_pub.publishVisual()
            # pub_log[p.log_counter] = time.time() - start_time

            # pr.disable()

            # send desired command to the ros controller
            p.send_command(p.q_des, p.qd_des, p.tau_ffwd)



        p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],
                          np.zeros(p.robot.na))

    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        p.pr.dump_stats("/home/froscia/ros_ws/src/landing_controller/go1tests/stats_log.cprof")



