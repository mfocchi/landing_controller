import casadi as ca
import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
from landing_controller.controller.vhisip_dynamics import VHSIP
from landing_controller.controller.feasibility import *
import os
import time as clock
np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

flight_time = 0.1
### INPUTS ###
m = 12.
L = 0.25
max_spring_compression = 0.1
max_settling_time = 1.2
apex_height = 3
g_mag = 9.81
pmin=0.1
pmax=0.3
Tfmax=2
FZmax = 800

state_x0 = np.array([[1],
                     [0.]])

state_y0 = np.array([[2],
                     [0.]])

state_z0 = np.array([[- g_mag*flight_time],
                     [0.]])


vhsip = VHSIP(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)
vhsip.set_init(state_x0, state_y0, state_z0)
# msd dynamics
k_limit = m * (8 / max_settling_time)**2
eig_z_limit = -np.sqrt(k_limit/m)

vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

if state_z0[0,0] < vz_limit:
    k = m * ( state_z0[0, 0] / (np.exp(1) * max_spring_compression) )**2
else:
    k = k_limit
d = 2 * np.sqrt(m * k)

time, posz, velz, accz = vhsip.MSD_z_dynamics(m, k, d)
vhsip.set_z_dynamics(time, posz, velz, accz)

vhsip.propagation_matrices()

vhsip.solve_ocp(vx_f=0, vy_f=0)

solved = vhsip.is_ZMPfeasible()
if not solved:
    vhsip.projected_zmp, res = vhsip.projectPointFR(vhsip.zmp_xy)
else:
    vhsip.projected_zmp = vhsip.zmp_xy.copy()

vhsip.compute_xy_dynamics(vhsip.zmp_xy[0], vhsip.zmp_xy[1])

counter = 0
vhsip.plot_ref(title='vhsip'+str(counter))
X0 = None
while not solved:
    i = vhsip.T_p_com_ref.shape[1]-1
    while i >= 0 :
        if np.all(vhsip.T_p_com_ref[:2, i] >= vhsip.projected_zmp):
            i -= 1
        else:
            break

    t_star = i * vhsip.dt

    vhsip.set_init(state_x0, state_y0, state_z0)
    time, posz, velz, accz, X0 = vhsip.bezier_z_dynamicsN(p0=L, v0=state_z0[0], amax=FZmax/m, pmin=pmin, pmax=pmax, Tfmax=t_star, Tf=None, pf=None, vf=None, Y0=X0, N=5)

    vhsip.set_z_dynamics(time, posz, velz, accz)
    vhsip.propagation_matrices()
    vhsip.zmp_xy = vhsip.zmp_xy.copy()
    vhsip.compute_xy_dynamics()

    counter += 1

    vhsip.plot_ref(title='vhsip'+str(counter)+' t* = '+ str(t_star))

    solved = vhsip.is_COMtrajFeasible()



    if counter == 5:
        print('Limit loops reached')
        break



vhsip.plot_3D(title='vhsip'+str(counter))



