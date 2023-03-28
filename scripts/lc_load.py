import matplotlib
matplotlib.use('TkAgg')
import sys
sys.path.append('../')
from landing_controller.controller.utility import *

np.set_printoptions(linewidth=np.inf,   # number of characters per line before new line
                    floatmode='fixed',  # print fixed number of digits ...
                    precision=4,        # ... 4
                    sign=' ',           # print space if sign is plus
                    suppress=True,      # suppress scientific notation
                    threshold=np.inf)
import os
from scipy.io import loadmat

directory = os.environ['LOCOSIM_DIR'] + '/landing_controller/simulations/20230328_115407/sim00/'
filename = "DATA.mat"
DATA = loadmat(directory + filename)
# Convert dictionary entries into local variables 
locals().update(DATA)


# plotFrame('position', time_log=time_log, des_Pose_log=comPoseW_des_log, Pose_log=comPoseW_log,
#           title='CoM', frame='W', sharex=True, sharey=False, start=apex_sample, end=-1)
# plotFrame('velocity', time_log=time_log, des_Twist_log=comTwistW_des_log, Twist_log=comTwistW_log,
#           title='CoM', frame='W', sharex=True, sharey=False, start=apex_sample, end=-1)
# plotFrame('position', time_log=time_log, des_Pose_log=basePoseW_des_log, Pose_log=basePoseW_log,
#           title='base', frame='W', sharex=True, sharey=False, start=apex_sample, end=-1)
# plotFrame('velocity', time_log=time_log, des_Twist_log=baseTwistW_des_log, Twist_log=baseTwistW_log,
#           title='base', frame='W', sharex=True, sharey=False, start=apex_sample, end=-1)
# plotContacts('position', time_log=time_log, des_LinPose_log=B_contacts_des_log,
#              LinPose_log=B_contacts_log,
#              contact_states=contact_state_log, frame='B', sharex=True, sharey=False, start=apex_sample, end=-1)
# plotContacts('GRFs', time_log=time_log, des_Forces_log=grForcesW_des_log,
#              Forces_log=grForcesW_log, contact_states=contact_state_log, frame='W',
#              sharex=True, sharey=False, start=apex_sample, end=-1)
# plotJoint('position', time_log=time_log, q_log=q_log, q_des_log=q_des_log, sharex=True, sharey=False,
#           start=apex_sample, end=-1)
# plotJoint('velocity', time_log=time_log, qd_log=qd_log, qd_des_log=qd_des_log, sharex=True,
#           sharey=False, start=apex_sample, end=-1)
# plotJoint('torque', time_log=time_log, tau_log=tau_log, tau_ffwd_log=tau_ffwd_log,
#           tau_des_log=tau_fb_log, sharex=True, sharey=False, start=apex_sample, end=-1)
# plotWrenches('fb', 8, time_log, des_Wrench_fb_log=wrench_fbW_log)
# plotWrenches('ffwd', 9, time_log, des_Wrench_ffwd_log=wrench_ffW_log)
# plotWrenches('g', 10, time_log, des_Wrench_g_log=wrench_gW_log)
# plotFrameLinear('velocity', time_log=time_log, des_Twist_log=baseTwistW_legOdom_log,
#                 Twist_log=baseLinTwistImuW_log, title='Base velocity estimate', frame='W', sharex=True,
#                 sharey=False, start=apex_sample, end=-1)
