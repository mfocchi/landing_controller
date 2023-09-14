import numpy as np

from jet_leg.plotting.plotting_tools import Plotter
import random

from jet_leg.kinematics import kinematics_interface
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization import nonlinear_projection
from shapely.geometry import Polygon
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

import pinocchio as pin
from base_controllers.utils.common_functions import getRobotModel
import  base_controllers.params as conf

robot_name = 'go1'
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

math = Math()


robot = getRobotModel(robot_name, generate_urdf=False)

default_q = pin.neutral(robot.model)
pin_Quat = pin.Quaternion(pin.rpy.rpyToMatrix(0, -0., 0))
default_q[3] =  pin_Quat.x
default_q[4] =  pin_Quat.y
default_q[5] =  pin_Quat.z
default_q[6] =  pin_Quat.w
default_q[7:12 + 7] = conf.robot_params[robot_name]['q_0']

legs = ['lf', 'rf','lh', 'rh']
feet_id = [robot.model.getFrameId(leg + '_foot') for leg in legs]

pin.forwardKinematics(robot.model, robot.data, default_q)
pin.updateFramePlacements(robot.model, robot.data)

""" contact points in the World Frame"""
LF_footBF =  robot.data.oMf[robot.model.getFrameId('lf_foot')].translation.copy()
RF_footBF =  robot.data.oMf[robot.model.getFrameId('rf_foot')].translation.copy()
LH_footBF =  robot.data.oMf[robot.model.getFrameId('lh_foot')].translation.copy()
RH_footBF =  robot.data.oMf[robot.model.getFrameId('rh_foot')].translation.copy()


height = 0
for foot in feet_id:
    height -=  robot.data.oMf[foot].translation[2]/4

default_q[2] = height

pin.forwardKinematics(robot.model, robot.data, default_q)
pin.updateFramePlacements(robot.model, robot.data)

comWF = robot.robotComW(default_q)#np.array([-0.00397,  0.00084,  0.3 ])
comBF = robot.robotComB(default_q[7:])  # #!IMPORTANT approx we assume fixed offset computed in default config and that does not change with joint config
rpy = pin.rpy.matrixToRpy(pin.Quaternion(default_q[3:7]).toRotationMatrix()).T

LF_footWF = LF_footBF + np.array([0, 0, height+0.02])
RF_footWF = RF_footBF + np.array([0, 0, height+0.02])
LH_footWF = LH_footBF + np.array([0, 0, height+0.02])
RH_footWF = RH_footBF + np.array([0, 0, height+0.02])

contactsWF = np.vstack((LF_footWF, RF_footWF, LH_footWF, RH_footWF))


''' stanceFeet vector contains 1 if the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]



''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_q_lim_max = robot.model.upperPositionLimit[7:10]  # HAA, HFE, KFE
LF_q_lim_min = robot.model.lowerPositionLimit[7:10]  # HAA, HFE, KFE
RF_q_lim_max = robot.model.upperPositionLimit[10:13]  # HAA, HFE, KFE
RF_q_lim_min = robot.model.lowerPositionLimit[10:13]  # HAA, HFE, KFE
LH_q_lim_max = robot.model.upperPositionLimit[13:16]  # HAA, HFE, KFE
LH_q_lim_min = robot.model.lowerPositionLimit[13:16]  # HAA, HFE, KFE
RH_q_lim_max = robot.model.upperPositionLimit[16:19]  # HAA, HFE, KFE
RH_q_lim_min = robot.model.lowerPositionLimit[16:19]  # HAA, HFE, KFE
joint_limits_max = np.array([LF_q_lim_max, RF_q_lim_max, LH_q_lim_max, RH_q_lim_max])
joint_limits_min = np.array([LF_q_lim_min, RF_q_lim_min, LH_q_lim_min, RH_q_lim_min])

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''

params = IterativeProjectionParameters()
params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)
params.setActiveContacts(stanceFeet)


com_check = None
KIN_REG = []
CZ = []
# kin_reg, computation_time = projection.project_polytope(params, com_check)
for cz in np.arange(0.1, 0.4, 0.01):
    comWF[2] = cz
    params.setCoMPosWF(comWF)
    kin_reg, computation_time = projection.project_polytope(params, com_check, 36 * np.pi / 180, 0.01)
    if len(kin_reg) != 0:
        KIN_REG.append(kin_reg)
        CZ.append(cz)

# kin_reg, computation_time = projection.project_polytope(params)
print( "vertices", kin_reg)
print ("Computation Time: ", computation_time, " seconds")

plt.close()


plotter = Plotter()
nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)

#
#
# import matplotlib.colors as colors
# import matplotlib.cm as cmx
# import mpl_toolkits.mplot3d.art3d as art3d
# from matplotlib.patches import Polygon as Polygon_mpl
#
# scale = np.linspace(50, 150, len(KIN_REG))
# jet = cm = plt.get_cmap('RdBu')
# cNorm = colors.Normalize(vmin=55, vmax=145)
# scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
#
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.set_xlabel('x [m]')
# ax.set_ylabel('y [m]')
# ax.set_zlabel('z [m]')
# idx = len(KIN_REG)-1
# for kin_reg in KIN_REG:
#     colorVal = scalarMap.to_rgba(scale[idx])
#     idx -= 1
#     p = Polygon_mpl(kin_reg[:, :2], label=str(kin_reg[0,2]), edgecolor='k', facecolor=colorVal)
#     ax.add_patch(p)
#     art3d.pathpatch_2d_to_3d(p, z=kin_reg[0,2], zdir="z")
# ax.set_xlim(-0.4, 0.4)
# ax.set_ylim(-0.4, 0.4)
# ax.set_zlim(-0.0, 0.3)
#
# ax.legend(title='height [m]')
# plt.show()
#
# plt.figure()
# plt.scatter(KIN_REG[5][:, 0], KIN_REG[5][:, 1])
#
# #
# # for kin_reg in KIN_REG:
# #     h6 = plotter.plot_kin_reg(np.transpose(kin_reg), '--b', 'Iterative Projection')
#
# for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
#     idx = int(stanceID[j])
#     ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
#     h5 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
#
# plt.show()


KIN_and_FRIC_REG = []
ch = ConvexHull(contactsWF[:, :-1])
SP = Polygon(ch.points[ch.vertices])
for kin_reg in KIN_REG:
    kr = Polygon(kin_reg[: , :-1])
    kin_and_fric_reg = kr.intersection(SP)
    interserction_vertices2d = np.array(kin_and_fric_reg.exterior.coords)
    if interserction_vertices2d.shape[0] > 0:
        KIN_and_FRIC_REG.append(np.hstack([interserction_vertices2d, np.ones([interserction_vertices2d.shape[0], 1])*kin_reg[0, -1] ]))



from scipy.io import savemat

data = {'kin_regions': KIN_REG, 'kin_and_fric_reg': KIN_and_FRIC_REG}
savemat("go1_kin_region.mat", data)

# from scipy.io import savemat
# data = {'kin_regions': KIN_REG, 'kin_and_fric_reg': KIN_and_FRIC_REG}
# savemat("go1_kin_region_l0.mat", data)
