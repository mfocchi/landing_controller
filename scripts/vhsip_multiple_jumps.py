import numpy as np
from landing_controller.controller.extended.ext_landing_controller import ExtendedLandingController, ballisticTraj
from landing_controller.controller.extended.utils import plot_ref, plot_3D

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

### INPUTS ###
m = 12.
g_mag = 9.81
L = 0.25
dt=0.002
max_spring_compression = 0.1
max_settling_time = 1.2
apex_height = 1
flight_time = np.sqrt(2*(apex_height-L)/g_mag)
pmin = 0.1
pmax = 0.3
Tfmax = 2


sim_time = np.empty([0])
time_offset = 0.
sim_pos = np.empty([3, 0])
sim_vel = np.empty([3, 0])
sim_acc = np.empty([3, 0])

td_counter = 0

state_xapex = np.array([[4.5],
                        [0.]])
state_yapex = np.array([[0.],
                        [0.]])
state_zapex = np.array([[0.],
                        [apex_height]])

# compute and save ballistic trajectory
vz_lo = state_zapex[0]
pz_lo = state_zapex[1]
flight_time = (-vz_lo-np.sqrt(vz_lo**2+2*g_mag*(pz_lo-L)))/(-g_mag)
ball_time, ball_pos, ball_vel, ball_acc = ballisticTraj(state_xapex, state_yapex, state_zapex, flight_time, dt, g_mag)

sim_time = np.hstack([sim_time, ball_time + time_offset])
time_offset = sim_time[-1]
sim_pos = np.hstack([sim_pos, ball_pos])
sim_vel = np.hstack([sim_vel, ball_vel])
sim_acc = np.hstack([sim_acc, ball_acc])


# state at new touch down
state_xtd = np.array([[sim_vel[0, -1]],
                      [sim_pos[0, -1]]])
state_ytd = np.array([[sim_vel[1, -1]],
                      [sim_pos[1, -1]]])
state_ztd = np.array([[sim_vel[2, -1]],
                      [0]])
for i in range(0,2):
    td_counter += 1
    ELC = ExtendedLandingController(dt=dt, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)

    ELC.set_init(state_xtd, state_ytd, state_ztd)
    # Fzmax = ELC.suboptimal_force(state_xtd, state_ytd)
    FZmax = 800.

    offset = np.array([[state_xtd[1]],
                       [state_ytd[1]],
                       [0]])
    # msd dynamics
    k_limit = m * (8 / max_settling_time)**2
    eig_z_limit = -np.sqrt(k_limit/m)

    vz_limit = np.exp(1) * max_spring_compression * eig_z_limit

    if state_ztd[0,0] < vz_limit:
        k = m * ( state_ztd[0, 0] / (np.exp(1) * max_spring_compression) )**2
    else:
        k = k_limit
    d = 2 * np.sqrt(m * k)

    time, posz, velz, accz = ELC.MSD_z_dynamics(m, k, d)
    ELC.set_z_dynamics(time, posz, velz, accz)

    ELC.propagation_matrices()

    ELC.solve_ocp(vx_f=0, vy_f=0)

    if not ELC.is_ZMPfeasible():
        ELC.projected_zmp = -ELC.projectPointFR(-ELC.zmp_xy)
    else:
        ELC.projected_zmp = ELC.zmp_xy.copy()

    ELC.compute_xy_dynamics(ELC.projected_zmp[0], ELC.projected_zmp[1])

    solved = ELC.is_COMtrajFeasible()
    ctrl_counter = 0
    title = "TD number: " + str(td_counter) + ' ELC: ' + str(ctrl_counter) + " solved:" + str(solved)
    plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title)
    plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes, title)

    X0 = None
    while not solved:
        ctrl_counter += 1

        t_star = ELC.index_ref_over_zmp(1) * ELC.dt

        FZmax = ELC.suboptimal_force(state_xtd, state_ytd)
        time, posz, velz, accz, X0= ELC.bezier_z_dynamicsN(p0=L, v0=state_ztd[0], amax=800/m, pmin=pmin, pmax=pmax, Tfmax=t_star, Tf=None, pf=None, vf=None, Y0=X0, N=5)
        ELC.set_z_dynamics(time, posz, velz, accz)
        ELC.propagation_matrices()
        ELC.zmp_xy = ELC.zmp_xy.copy()
        ELC.compute_xy_dynamics()

        solved = ELC.is_COMtrajFeasible()

        title = "TD number: " + str(td_counter) + ' ELC: ' + str(ctrl_counter) + " solved:" + str(solved)
        plot_ref(time, ELC.T_p_com_ref, ELC.T_v_com_ref, ELC.T_a_com_ref, ELC.zmp_xy, ELC.projected_zmp, title)
        plot_3D(ELC.T_p_com_ref, ELC.zmp_xy, ELC.projected_zmp, ELC.feasibility, ELC.feasibility_l0, ELC.ctrl_indexes,
                title)
        if ctrl_counter == 5:
            print('Limit loops reached')
            break


    # save the stabilizing trajectory
    sim_time = np.hstack([sim_time, time+time_offset])
    time_offset = sim_time[-1]
    sim_pos = np.hstack([sim_pos, ELC.T_p_com_ref+offset])
    sim_vel = np.hstack([sim_vel, ELC.T_v_com_ref])
    sim_acc = np.hstack([sim_acc, ELC.T_a_com_ref])


    # state at new lift off
    state_xlo = np.array([[sim_vel[0, -1]],
                          [sim_pos[0, -1]]])
    state_ylo = np.array([[sim_vel[1, -1]],
                          [sim_pos[1, -1]]])
    state_zlo = np.array([[sim_vel[2, -1]],
                          [sim_pos[2, -1]]])


    if state_zlo[1]-L < 0.01:
        break

    # compute and save ballistic trajectory
    vz_lo = state_zlo[0]
    pz_lo = state_zlo[1]
    flight_time = (-vz_lo-np.sqrt(vz_lo**2+2*g_mag*(pz_lo-L)))/(-g_mag)
    ball_time, ball_pos, ball_vel, ball_acc = ballisticTraj(state_xlo, state_ylo, state_zlo, flight_time, dt, g_mag)

    sim_time = np.hstack([sim_time, ball_time + time_offset])
    time_offset = sim_time[-1]
    sim_pos = np.hstack([sim_pos, ball_pos])
    sim_vel = np.hstack([sim_vel, ball_vel])
    sim_acc = np.hstack([sim_acc, ball_acc])

    # state at new touch down
    state_xtd = np.array([[sim_vel[0, -1]],
                          [sim_pos[0, -1]]])
    state_ytd = np.array([[sim_vel[1, -1]],
                          [sim_pos[1, -1]]])
    state_ztd = np.array([[sim_vel[2, -1]],
                          [0]])


plot_ref(sim_time, sim_pos, sim_vel, sim_acc, title='Complete reference')



#############
# ANIMATION #
#############

from matplotlib import animation
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')


def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])

N = sim_pos.shape[1]
data = sim_pos
line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

# Setting the axes properties
xymin = sim_pos[0:2, :].min()-0.05
xymax = sim_pos[0:2, :].max()-0.05

ax.set_xlim3d([xymin, xymax])
ax.set_xlabel('X')

ax.set_xlim3d([xymin, xymax])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, sim_pos[2, :].max()+0.05])
ax.set_zlabel('Z')

ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=1, blit=False)
#ani.save('matplot003.gif', writer='imagemagick')
plt.show()