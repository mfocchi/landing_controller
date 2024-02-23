import numpy as np
from landing_controller.controller.extended.ext_landingController import ExtendedLandingController
from landing_controller.controller.extended.utils import plot_ref, plot_3D
import time as t_os

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)

flight_time = 0.1
### INPUTS ###
m = 12.
L = 0.25
max_spring_compression = 0.1
max_settling_time = 1.2
apex_height = .6
g_mag = 9.81
pmin = 0.1
pmax = 0.3
Tfmax = 2

state_x0 = np.array([[5],
                     [0.]])

state_y0 = np.array([[0],
                     [0.]])

state_z0 = np.array([[- np.sqrt(2*apex_height*g_mag)],
                     [0.]])

LF_footWF = np.array([  0.20776,   0.18087, 0.])
RF_footWF = np.array([  0.20776,  -0.18087, 0.])
LH_footWF = np.array([ -0.16844,   0.18087, 0.])
RH_footWF = np.array([ -0.16844,   -0.18087, 0.])
feet = np.vstack([LF_footWF, RF_footWF, LH_footWF, RH_footWF])


ELC = ExtendedLandingController(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1., feet=feet)
ELC.set_init(state_x0, state_y0, state_z0)

#Fzmax = ELC.suboptimal_force(state_x0, state_y0)

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


solved = ELC.is_ZMPfeasible(ELC.zmp_xy)

if not solved:
    ELC.projected_zmp = -ELC.projectPointFR(-ELC.zmp_xy)
else:
    ELC.projected_zmp = ELC.zmp_xy.copy()

ELC.compute_xy_dynamics(ELC.zmp_xy[0], ELC.zmp_xy[1])
# solved, t_star = ELC.is_COMtrajFeasible()
# plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, feet_centroid_T, ELC.kinematic_region, ELC.kinematic_slice_l0,
#         ELC.ctrl_indexes, feet, title= str(solved), i=0)
ctrl_counter = 0


i = ELC.ctrl_horz-1
while i >= 0:
    if np.all(ELC.T_v_com_ref[:2, i] >= 0.8*ELC.init_velH):
        i -= 1
    else:
        break
t_star = i * ELC.dt
title = 'ELC: ' + str(ctrl_counter) + " solved:" + str(solved) + " t_star:" + str(t_star)
plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title, t_star)
plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.kinematic_region, ELC.kinematic_slice_l0, ELC.ctrl_indexes, feet, title)

X0 = np.zeros(6)
X0[0] = t_star
X0v = X0.copy()

while not solved:
    ctrl_counter += 1

    ELC.set_init(state_x0, state_y0, state_z0)

    # Fzmax = ELC.suboptimal_force(state_xtd, state_ytd)
    Fzmax = 600

    tic = t_os.time()
    time, posz, velz, accz, X0 = ELC.bezier_z_dynamicsN(p0=L, v0=state_z0[0], amax=Fzmax/m, pmin=pmin, pmax=pmax, Tfmax=t_star, Tf=None, pf=None, vf=2, Y0=X0, N=5)

    #time, posz, velz, accz, X0 = ELC.bezier_z_dynamics_cpp(p0=L, v0=float(state_z0[0]), amax=Fzmax / m, pmin=pmin, pmax=pmax, Tfmax=t_star, vf=2, X0=np.array(X0))
    toc = t_os.time()
    print('optimize bezier:', toc - tic)

    tic = t_os.time()
    ELC.set_z_dynamics(time, posz, velz, accz)
    ELC.propagation_matrices()
    #ELC.zmp_xy = ELC.zmp_xy.copy()
    ELC.compute_xy_dynamics(ELC.zmp_xy[0], ELC.zmp_xy[1])
    toc = t_os.time()
    print('horz dynamics:', toc - tic)

    i = ELC.ctrl_horz - 1
    # check the whole com traj is feasible
    check_com = ELC.is_COMtrajFeasible(i)

    # check velocity reduction
    check_fin_vel = np.all(ELC.T_v_com_ref[:2, i] <=  0.8 * ELC.T_v_com_ref[:2, 0])

    if check_com and check_fin_vel:
        solved = True

    # is it possible to solve the problem simply by cutting the refecence?
    else:
        while i > 0:
            if np.all(ELC.T_v_com_ref[:2, i] >= 0.8* ELC.init_velH):
                i -= 1
            else:
                break
        t_star = i * ELC.dt

        solved = ELC.is_COMtrajFeasible(i)

        if not solved:
            while i > 0:
                if not ELC.is_COMFeasible(i):
                    i -= 1
                else:
                    break
            t_star = i * ELC.dt


    print('feasibility and t intersection:', toc - tic)

    if ctrl_counter == 5:
        print('Limit loops reached')
        break

    # title = 'ELC: ' + str(ctrl_counter) + " solved:" + str(solved) + " t_star:" + str(t_star)
    # plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title, t_star)
    # plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes, feet,
    #         title)

    print(ctrl_counter)

    # if ctrl_counter == 5:
    #     print('Limit loops reached')
    #     break


#ELC.plot_3D(title='ELC'+str(counter)+" solved:"+str(solved))

# plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes, feet,
#             title)



    title = 'ELC: ' + str(ctrl_counter) + " solved:" + str(solved) + " t_star:" + str(t_star)
    plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title, t_star)
    plot_3D(ELC.T_p_com_ref, ELC.zmp_xy,  ELC.projected_zmp, ELC.kinematic_region, ELC.kinematic_slice_l0, ELC.ctrl_indexes, feet,
            title, i)


