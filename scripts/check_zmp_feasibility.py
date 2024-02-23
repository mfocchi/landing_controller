# this is an example file
# given the initial com horizontal velocity and apex height, the script computes the optimal cop location and it checks
# whether is possible to move the feet in order to put the support polygon centroid on the optimal cop

import numpy as np
from landing_controller.controller.extended.ext_landingController import ExtendedLandingController
from landing_controller.controller.extended.utils import  plot_3D


np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

### INPUTS ###
m = 12.
L = 0.25
max_spring_compression = 0.1
max_settling_time = 1.2
g_mag = 9.81
pmin = 0.1
pmax = 0.3
Tfmax = 2
# msd dynamics
k_limit = m * (8 / max_settling_time)**2
eig_z_limit = -np.sqrt(k_limit/m)
vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

### aerial motion inputs ###
flight_time = 0.1
apex_height = .6

# feet location
LF_footWF = np.array([  0.20776,   0.18087, 0.])
RF_footWF = np.array([  0.20776,  -0.18087, 0.])
LH_footWF = np.array([ -0.16844,   0.18087, 0.])
RH_footWF = np.array([ -0.16844,   -0.18087, 0.])
feet = np.vstack([LF_footWF, RF_footWF, LH_footWF, RH_footWF])

for v in [ 1.5,  2.5,  3.5]:
    state_x0 = np.array([[v],
                         [0.]])

    state_y0 = np.array([[0],
                         [0.]])

    state_z0 = np.array([[- np.sqrt(2*apex_height*g_mag)],
                         [0.]])

    ELC = ExtendedLandingController(dt=0.002, L=L, g_mag=g_mag, w_v=1., w_p=1., w_u=1., feet=feet)
    ELC.set_init(state_x0, state_y0, state_z0)


    if state_z0[0,0] < vz_limit:
        k = m * ( state_z0[0, 0] / (np.exp(1) * max_spring_compression) )**2
    else:
        k = k_limit
    d = 2 * np.sqrt(m * k)

    time, posz, velz, accz = ELC.MSD_z_dynamics(m, k, d)
    ELC.set_z_dynamics(time, posz, velz, accz)

    ELC.propagation_matrices()

    ELC.solve_ocp(vx_f=0, vy_f=0)

    solved = ELC.is_ZMPfeasible(ELC.zmp_xy)

    if not solved:
        feet_centroid_T = -ELC.projectPointFR(-ELC.zmp_xy)
    else:
        feet_centroid_T = ELC.zmp_xy.copy()

    ELC.compute_xy_dynamics(feet_centroid_T[0], feet_centroid_T[1])
    # solved, t_star = ELC.is_COMtrajFeasible()
    plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, feet_centroid_T, ELC.kinematic_region, ELC.kinematic_slice_l0, ELC.ctrl_indexes, feet,
            title=str(v)+str(solved), i=1)