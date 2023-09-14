import numpy as np
from landing_controller.controller.extended.ext_landing_controller import ExtendedLandingController
from landing_controller.controller.extended.utils import plot_ref, plot_3D

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

flight_time = 0.1
### INPUTS ###
m = 12.
L = 0.25
max_spring_compression = 0.1
max_settling_time = 1.2
apex_height = 1
g_mag = 9.81
pmin = 0.1
pmax = 0.3
Tfmax = 2

state_x0 = np.array([[3],
                     [0.]])

state_y0 = np.array([[0],
                     [0.]])

state_z0 = np.array([[- np.sqrt(2*apex_height*g_mag)],
                     [0.]])


ELC = ExtendedLandingController(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)
ELC.set_init(state_x0, state_y0, state_z0)
#Fzmax = ELC.suboptimal_force(state_x0, state_y0)
FZmax = 800
# msd dynamics
k_limit = m * (8 / max_settling_time)**2
eig_z_limit = -np.sqrt(k_limit/m)

vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

if state_z0[0,0] < vz_limit:
    k = m * ( state_z0[0, 0] / (np.exp(1) * max_spring_compression) )**2
else:
    k = k_limit
d = 2 * np.sqrt(m * k)

time, posz, velz, accz = ELC.MSD_z_dynamics(m, k, d)
ELC.set_z_dynamics(time, posz, velz, accz)

ELC.propagation_matrices()

ELC.solve_ocp(vx_f=0, vy_f=0)

solved = ELC.is_ZMPfeasible()

if not solved:
    ELC.projected_zmp= -ELC.projectPointFR(-ELC.zmp_xy)
else:
    ELC.projected_zmp = ELC.zmp_xy.copy()

ELC.compute_xy_dynamics(ELC.projected_zmp[0], ELC.projected_zmp[1])
# solved, t_star = ELC.is_COMtrajFeasible()

#
ctrl_counter = 0
title = 'ELC: ' + str(ctrl_counter) + " solved:" + str(solved)
plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title)
plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes, title)
X0 = None
while not solved:
    ctrl_counter += 1

    t_star = ELC.index_ref_over_zmp(-1) * ELC.dt

    ELC.set_init(state_x0, state_y0, state_z0)
    time, posz, velz, accz, X0 = ELC.bezier_z_dynamicsN(p0=L, v0=state_z0[0], amax=Fzmax/m, pmin=pmin, pmax=pmax, Tfmax=t_star, Tf=None, pf=None, vf=None, Y0=X0, N=5)

    ELC.set_z_dynamics(time, posz, velz, accz)
    ELC.propagation_matrices()
    #ELC.zmp_xy = ELC.zmp_xy.copy()
    ELC.compute_xy_dynamics(ELC.projected_zmp[0], ELC.projected_zmp[1])

    solved = ELC.is_COMtrajFeasible()

    title = 'ELC: ' + str(ctrl_counter) + " solved:" + str(solved)
    plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title)
    plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes,
            title)

    if ctrl_counter == 5:
        print('Limit loops reached')
        break


#ELC.plot_3D(title='ELC'+str(counter)+" solved:"+str(solved))



