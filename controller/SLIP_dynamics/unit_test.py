from SLIP_dynamics_lib import SLIP_dynamics
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import time

dt = 0.004
L = 0.200
max_spring_compression = 1/2 * L
m = 2.5
g_mag = 9.81
w_v = 100.
w_p = 1.
w_u = 1.
max_settling_time = 1.

s = SLIP_dynamics(dt, L, max_spring_compression, m, g_mag, w_v, w_p, w_u, max_settling_time)


fig_p, ax_p = plt.subplots(3,1)
fig_v, ax_v = plt.subplots(3,1)
fig_a, ax_a = plt.subplots(3,1)

fig_pz, ax_pz = plt.subplots()


legend = []

zmp_x_list = []
zmp_y_list = []

zmp_x_ocp_list = []
zmp_y_ocp_list = []
zmp_x_noocp_list = []
zmp_y_noocp_list = []
zmp_colour_list = []
v0_list = np.arange(0.1,2.1, 0.2)


for v0 in v0_list:
	init_pos = np.array([0., 0., L])
	init_vel = np.array([0.53963049,  0.,  -v0])
	init_state_x = np.array([[init_vel[0]], [init_pos[0]]])
	init_state_y = np.array([[init_vel[1]], [init_pos[1]]])
	#s.runVerbose(init_pos, init_vel)
	s.def_and_solveOCP(init_pos, init_vel)
	zmp_x_ocp_list.append(s.zmp_xy[0])
	zmp_y_ocp_list.append(s.zmp_xy[1])
	s.noOCP(init_pos, init_vel)
	zmp_x_noocp_list.append(s.zmp_xy[0])
	zmp_y_noocp_list.append(s.zmp_xy[1])
	s.xy_dynamics()
	print('zmp:', s.zmp_xy)
ax_p[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[0,:s.ctrl_horz].T)
ax_p[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[0, :s.ctrl_horz].T)
ax_p[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[1,:s.ctrl_horz].T)
ax_p[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[2,:s.ctrl_horz].T)
ax_pz.plot(s.ctrl_time[:s.ctrl_horz], s.T_p_com_ref[2,:s.ctrl_horz].T)
ax_v[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[0, :s.ctrl_horz].T)
ax_v[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[1, :s.ctrl_horz].T)
ax_v[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_v_com_ref[2, :s.ctrl_horz].T)
ax_a[0].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[0, :s.ctrl_horz].T)
ax_a[1].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[1, :s.ctrl_horz].T)
ax_a[2].plot(s.ctrl_time[:s.ctrl_horz], s.T_a_com_ref[2, :s.ctrl_horz].T)

zmp_x_list.append(s.zmp_xy[0])
zmp_y_list.append(s.zmp_xy[1])
#legend.append(init_vel[2])
ax_p[2].axhline(y=s.L-s.max_spring_compression, color='r', linestyle='--')
ax_v[0].axhline(y=0., color='r', linestyle='--')
ax_p[0].set_ylabel('pos com x [m]')
ax_p[1].set_ylabel('pos com y [m]')
ax_p[2].set_ylabel('pos com z [m]')
ax_v[0].set_ylabel('vel com x [m/s]')
ax_v[1].set_ylabel('vel com y [m/s]')
ax_v[2].set_ylabel('vel com z [m/s]')
ax_a[0].set_ylabel('acc com x [m/s^2]')
ax_a[1].set_ylabel('acc com y [m/s^2]')
ax_a[2].set_ylabel('acc com z [m/s^2]')
ax_pz.set_ylabel('pos com z [m]')
ax_p[2].set_xlabel('t [s]')
ax_v[2].set_xlabel('t [s]')
ax_a[2].set_xlabel('t [s]')
ax_pz.set_xlabel('t [s]')
ax_pz.legend(legend, title='V0 [m/s]')
l = len(zmp_x_ocp_list)
plt.figure()
for i in range(0, l):
	plt.scatter(zmp_x_ocp_list[i], zmp_y_ocp_list[i], color=[(0., 0., 1. )], alpha=(i+1.)/l)
	plt.scatter(zmp_x_noocp_list[i], zmp_y_noocp_list[i], color=[(1., 0., 0. )], alpha=(i+1.)/l)
plt.title('zmp')
plt.ylabel('zmp y [m]')
plt.xlabel('zmp x [m]')
plt.show()


