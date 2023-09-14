# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.feasible_wrench_polytope import FeasibleWrenchPolytope
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
from scipy.io import loadmat

from base_controllers.utils.common_functions import getRobotModel
import base_controllers.params as conf

import pinocchio as pin
from scipy.optimize import linprog


plt.close('all')
math = Math()

def plot_FWP(FWP, max_force, title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.scatter(FWP[0, :], FWP[1, :], FWP[2, :], marker='o', color='royalblue', alpha=0.2)
    ax.scatter(max_force[0], max_force[1], max_force[2], marker='o', color='r', s=1000)
    plt.title(title)
    points = FWP[:3, :].T
    FWP3d_hull = ConvexHull(points)
    for i in FWP3d_hull.simplices:
        plt.plot(points[i, 0], points[i, 1], points[i, 2], color='b')

    vertices = points[FWP3d_hull.vertices]
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], marker='o', color='b')

    plt.show()

def suboptimal_force(lookup_table, theta):
    idx = (np.abs(lookup_table[:, 0] - theta)).argmin()
    return lookup_table[idx, 1]


''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'go1'

robot = getRobotModel(robot_name, generate_urdf=True)

q = pin.neutral(robot.model)
q[7:12 + 7] = conf.robot_params[robot_name]['q_0']

legs = ['lf', 'rf','lh', 'rh']
feet_id = [robot.model.getFrameId(leg + '_foot') for leg in legs]

pin.forwardKinematics(robot.model, robot.data, q)
pin.updateFramePlacements(robot.model, robot.data)

height = 0
for foot in feet_id:
    height -=  robot.data.oMf[foot].translation[2]/4

q[2] = height

pin.forwardKinematics(robot.model, robot.data, q)
pin.updateFramePlacements(robot.model, robot.data)

''' number of generators, i.e. rays/edges used to linearize the friction cone '''
ng = 4

'''
possible constraints for each foot:
 ONLY_ACTUATION = only joint-torque limits are enforces
 ONLY_FRICTION = only friction cone constraints are enforced
 FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
'''
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']

# number of decision variables of the problem
# n = nc*6
# comWF = robot.robotComW(q)      # pos of COM in world frame w. trunk controller
comBF = robot.robotComB(q[7:])  # pos of COM in body frame w. trunk controller
rpy = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix()).T
# the effect of g acceleration in default conf is < 5% of the actuation limits, so we don't consider it in the max
# force computation
comWF_lin_acc = np.array([.0, .0, 10.0])
comWF_ang_acc = np.array([.0, .0, .0])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForce = np.array([0., .0, .0]) # units are N
extCentroidalTorque = np.array([.0, .0, .0]) # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

""" contact points in the World Frame"""
LF_foot =  robot.data.oMf[robot.model.getFrameId('lf_foot')].translation #+ np.array([0.1, 0, 0])
RF_foot =  robot.data.oMf[robot.model.getFrameId('rf_foot')].translation #+ np.array([0.1, 0, 0])
LH_foot =  robot.data.oMf[robot.model.getFrameId('lh_foot')].translation #+ np.array([0.1, 0, 0])
RH_foot =  robot.data.oMf[robot.model.getFrameId('rh_foot')].translation #+ np.array([0.1, 0, 0])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
mu = 0.8

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

randomSwingLeg = random.randint(0, 3)
tripleStance = False  # if you want you can define a swing leg using this variable
if tripleStance:
    print('Swing leg', randomSwingLeg)
    stanceFeet[randomSwingLeg] = 0
print('stanceLegs ', stanceFeet)

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ = array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([-000.0, 0.0, -000.0])  # units are Nm
extTorqueW = np.array([0.0, -00000.0, -00000.0])  # units are Nm

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()
params.pointContacts = True
params.setContactsPosWF(contactsWF)
params.externalCentroidalWrench = extCentroidalWrench

#params.setCoMLinAcc(comWF_lin_acc)
params.setTorqueLims(comp_dyn.robotModel.robotModel.joint_torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)

rbd = RigidBodyDynamics()

data = loadmat("go1_kin_region_l0.mat")
kin_fric_reg = data['kin_and_fric_reg'][0]

lookup_table = np.zeros([kin_fric_reg.shape[0], 2])

for i, comWF in enumerate(kin_fric_reg):

    tic = time.time()
    params.setCoMPosWF(comWF)
    '''I now check whether the given CoM configuration is stable or not'''
    C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)

    if not isIKoutOfWorkSpace:
        fwp = FeasibleWrenchPolytope(params)
        FWP = fwp.computeFeasibleWrenchPolytopeVRep(params, forcePolytopes)
        FWP_hull = ConvexHull(FWP.T,  qhull_options="QJ")
        c = np.array([0, 0, -1, 0, 0, 0])
        A_ub = FWP_hull.equations[:, :-1]
        b_ub = -FWP_hull.equations[:, -1]
        A_eq = np.array([[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 1]])
        b_eq = np.zeros([5, 1])
        res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=None, method='highs-ds',
                  callback=None, options=None, x0=None, integrality=None)
        toc = time.time()
        # if we disard the angular part of the wrench, the result is different! use full wrench polytope
        # FWP_hull3d = ConvexHull(FWP[:3, :].T, qhull_options="QJ")
        # c = np.array([0, 0, -1])
        # A_ub = FWP_hull3d.equations[:, :-1]
        # b_ub = -FWP_hull3d.equations[:, -1]
        # res3d = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=None, b_eq=None, bounds=None, method='highs-ds',
        #                 callback=None, options=None, x0=None, integrality=None)

        print("FWP + linprog took", toc-tic)

        theta = np.arctan2(comWF[1], comWF[0])
        Fz = res.x[2]

        lookup_table[i, 0] = theta
        lookup_table[i, 1] = Fz


        title = str(theta)
        plot_FWP(FWP, res.x, title)


        # w_gi = rbd.computeCentroidalWrench(params.getTotalMass(), comWF, params.externalCentroidalWrench, comWF_lin_acc)
        # '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
        # start = time.time()
        # isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
        # print("dynamic stability check time", time.time() - start)
        #
        # '''I now check whether the given CoM configuration is statically stable or not (see "Feasible Region")'''
        # start = time.time()
        # isStaticallyStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)
        # print("static stability check time", time.time() - start)
    else:
        isFWPStable = False
        isStaticallyStable = False

# '''Plotting the contact points in the 3D figure'''
# nc = np.sum(stanceFeet)
# stanceID = params.getStanceIndex(stanceFeet)
# force_scaling_factor = 1500
#
# ''' 2D figure '''
# plotter = Plotter()
# plt.figure()
# for j in range(0,
#                nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
#     idx = int(stanceID[j])
#     ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
#     h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
#
# '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
# inertialForce = comWF_lin_acc * params.getTotalMass() / force_scaling_factor
# extForce = extForce / 100
# if isFWPStable:
#     plt.plot(comWF[0], comWF[1], 'go', markersize=15, label='CoM (dynamic check)')
# else:
#     plt.plot(comWF[0], comWF[1], 'ro', markersize=15, label='CoM (dynamic check)')
#
# plt.arrow(comWF[0], comWF[1], inertialForce[0], inertialForce[1], head_width=0.01, head_length=0.01, fc='k',
#           ec='orange', label='inertial acceleration')
# plt.arrow(comWF[0] + inertialForce[0], comWF[1] + inertialForce[1], extForce[0] / force_scaling_factor,
#           extForce[1] / force_scaling_factor, head_width=0.01,
#           head_length=0.01, fc='blue', ec='blue', label='external force')
#
# plt.scatter([comWF[0], comWF[0] + inertialForce[0]], [comWF[1], comWF[1] + inertialForce[1]], color='k', marker='^',
#             label='inertial acceleration')
# plt.scatter([comWF[0], comWF[0] + inertialForce[0]], [comWF[1], comWF[1] + inertialForce[1]], color='b', marker='^',
#             label='external force')
#
# if isStaticallyStable:
#     plt.plot(comWF[0], comWF[1], 'g', markersize=20, marker='^', label='CoM (static check)')
# else:
#     plt.plot(comWF[0], comWF[1], 'r', markersize=20, marker='^', label='CoM (static check)')
#
# plt.grid()
# plt.xlabel("X [m]")
# plt.ylabel("Y [m]")
# plt.legend()
# plt.show()
#




######################################
# Find maximum wrench in z direction #
######################################
# max lambda
# s.t. w = lambda * [ 0 0 1 0 0 0]^T in FWP
import time

from scipy.io import savemat
data = {'lookup_table_forces': lookup_table}
savemat('lookup_table_forces.mat', data)








