# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 16:08:43 2018

@author: romeoorsolino
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization import nonlinear_projection
from jet_leg.plotting.plotting_tools import Plotter

import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, PathPatch
import mpl_toolkits.mplot3d.art3d as art3d

from base_controllers.utils.common_functions import getRobotModel
import  base_controllers.params as conf

import pinocchio as pin


def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)


plt.close('all')

robot_name = 'go1'
projection = nonlinear_projection.NonlinearProjectionBretl(robot_name)

math = Math()


robot = getRobotModel(robot_name, generate_urdf=False)

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

# ONLY_ACTUATION or ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = ['ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION']
useVariableJacobian = False

# number of decision variables of the problem
# n = nc*6
comWF = robot.robotComW(q) # pos of COM in world frame w. trunk controller
comBF = robot.robotComB(q[7:])  # pos of COM in body frame w. trunk controller
rpy = pin.rpy.matrixToRpy(pin.Quaternion(q[3:7]).toRotationMatrix()).T
comWF_lin_acc = np.array([.0, .0, .0])
comWF_ang_acc = np.array([.0, .0, .0])

""" contact points in the World Frame"""
LF_foot =  robot.data.oMf[robot.model.getFrameId('lf_foot')].translation
RF_foot =  robot.data.oMf[robot.model.getFrameId('rf_foot')].translation
LH_foot =  robot.data.oMf[robot.model.getFrameId('lh_foot')].translation
RH_foot =  robot.data.oMf[robot.model.getFrameId('rh_foot')].translation

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
stanceLegs = [1, 1, 1, 1]
nc = np.sum(stanceLegs)
stanceIndex = []
swingIndex = []
print ('stance', stanceLegs)
for iter in range(0, 4):
    if stanceLegs[iter] == 1:
        #               print 'new poly', stanceIndex, iter
        stanceIndex = np.hstack([stanceIndex, iter])
    else:
        swingIndex = iter

''' parameters to be tuned'''
g = 9.81
mu = 0.8
axisZ = np.array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
# %% Cell 2

''' joint position limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
joint_limits_max = robot.model.upperPositionLimit[7:].reshape(4,3)
joint_limits_min = robot.model.lowerPositionLimit[7:].reshape(4,3)

normals = np.vstack([n1, n2, n3, n4])

''' Add 2D figure '''
mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.unicode'] = True
fig = plt.figure(1)

scale = np.linspace(50, 150, 6)
jet = cm = plt.get_cmap('RdBu')
cNorm = colors.Normalize(vmin=55, vmax=145)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 5

params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setCoMPosBF(comBF)
params.setOrientation(rpy)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setJointLimsMax(joint_limits_max)
params.setJointLimsMin(joint_limits_min)



ax = fig.add_subplot(111)
P = []
for height in range(25, 13, -2):
    comWF[2] = height/100.
    params.setCoMPosWF(comWF)
    polygon, computation_time = projection.project_polytope(params, None, 20. * np.pi / 180, 0.02)
    P.append(polygon)
    point = np.vstack([polygon])
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)' % (colorVal[0], colorVal[1], colorVal[2]))
    idx -= 1
    # plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')
    x = np.hstack([point[:, 0], point[0, 0]])
    y = np.hstack([point[:, 1], point[0, 1]])
    h1 = ax.plot(x, y, color=colorVal, linewidth=5., label=r'${}$'.format(height/100.) + r' $\mathrm{m}$')
ax.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label=r'$\mathrm{Feet}$')

box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.95, box.height])
plt.xlim([-0.5,0.5])
plt.rc('font', family='serif', size=20)
plt.grid()
plt.xlabel("x [m]")
plt.ylabel("y [m]")
ax.legend()

plt.show()

#
# ###########
# '''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
idx = 5
for polygon in P:
    colorVal = scalarMap.to_rgba(scale[idx])
    idx -= 1
    p = Polygon(polygon[:, :2], label=str(polygon[0,2]), edgecolor='k', facecolor=colorVal)
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=polygon[0,2], zdir="z")
ax.set_xlim(-0.4, 0.4)
ax.set_ylim(-0.4, 0.4)
ax.set_zlim(-0.0, 0.3)

ax.legend(title='height [m]')
plt.show()

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import time
polygonS = Polygon(polygon[:, :2])
time2check = np.zeros([10000,1])
for i in range(time2check.shape[0]):
    p =(np.random.rand(2) - 0.5) * 3
    tic = time.time()
    check =  Point(p).within(polygonS)
    toc = time.time()
    time2check[i] = toc-tic
    if time2check[i] > 0.001:
        print(time2check[i], check, p)

plt.figure()
plt.plot(time2check)

plt.hlines(time2check.mean()+3*time2check.std()**2, 0, time2check.shape[0])
plt.hlines(time2check.mean()-3*time2check.std()**2, 0, time2check.shape[0])
plt.hlines(time2check.mean(), 0, time2check.shape[0])