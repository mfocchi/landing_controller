import casadi as ca
import numpy as np
from  scipy.special import comb
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from landing_controller.controller.vhisip_dynamics import VHSIP

def bezier(w, tau):
    b = 0.
    deg = w.shape[0] - 1
    for k in range(0, deg+1):
        b += comb(deg, k, exact=True) * w[k] * (1 - tau) ** (deg - k) * tau ** k
    return b


def bezierTraj(w, T0=0, Tf=1, step=0.002):
    T = np.arange(T0, Tf, step)
    deltaT = T[-1] - T[0]
    B = np.zeros_like(T)
    for i in range(T.shape[0]):
        tau = (T[i] - T[0]) / deltaT
        B[i] = bezier(w, tau)

    return T, B

###################################################
# COMPUTE COM Z REFERENCE AS A BEZIER OF DEGREE 4 #
###################################################

#### INPUT
# initial position
p0 = 0.25
# final position
pf = None
# initial velocity
v0 = -3.0
# final velocity
vf = None
# max time (I'm not sure it is actually needed)
Tmax = 2
# boundary on position
pmin = 0.1
pmax = 0.3
# boundary on acceleration
m = 12.5
amax = 800/m


#### NLP formulation
opti = ca.Opti()

# optimization variables: bezier coefficient, time
T = opti.variable(1)

# wp0 = opti.variable(1) no need of this
wp0 = p0
wp1 = opti.variable(1)
wp2 = opti.variable(1)
wp3 = opti.variable(1)
if pf is None or not (pmin < pf < pmax):  # if pf is given, set w4=pf
    wp4 = opti.variable(1)
else:
    wp4 = pf

wp = ca.vcat([wp0, wp1, wp2, wp3, wp4])

# for simplify computations, let define coeffs of bezier derivatives (do not consider time)
wv0 = 4 * (wp1 - wp0)
wv1 = 4 * (wp2 - wp1)
wv2 = 4 * (wp3 - wp2)
wv3 = 4 * (wp4 - wp3)
wv = ca.vcat([wv0, wv1, wv2, wv3])

wa0 = 3 * (wv1 - wv0)
wa1 = 3 * (wv2 - wv1)
wa2 = 3 * (wv3 - wv2)
wa = ca.vcat([wa0, wa1, wa2])

# COST
opti.minimize(-(wv[-1]/T)**2) #'max vf^2'

# CONSTRAINTS
# initial position
# opti.subject_to(wp0==p0)
#
# final position
if type(wp4) == ca.casadi.MX: # if pf is given, set w4=pf
    opti.subject_to(wp4 <= pmax)
#
# initial velocity
opti.subject_to(wv0/T == v0)

# causality, limited time
opti.subject_to(T > 0.1)
opti.subject_to(T <= Tmax)

# positive snap
s = wa2 - 2*wa1 + wa0
snap = 2 * s / T**4
opti.subject_to(s > 0)

# the minimum of the acceleration is postive
a = s/(T**4)
b = 2*(wa1-wa0)/(T**3)
c = wa0/(T**2)

Delta = b**2-4*a*c
av = -Delta/(4*a)
opti.subject_to(Delta<= 0.)
# opti.subject_to(av >= 20.)


# and in (0, T)
tv = - b/(2*a)
tauv = tv / T
opti.subject_to(tv < T)

# initial and final acceleration are upper-bounded
opti.subject_to(wa0/(T**2) <= amax)
opti.subject_to(wa2/(T**2) <= amax)

av_alt = bezier(wa/T**2, tauv)
# zero velocity at minimum of acceleration
vv = bezier(wv/T, tauv)
opti.subject_to(vv == 0)

# lower bound of position
pv = bezier(wp, tauv)
opti.subject_to(pv >= pmin)

if vf is None:
    # positive final velocity
    opti.subject_to(wv3 >= 0)
else:
    # if vf is given, equality constaint
    opti.subject_to(wv3/T==vf) # (not tested)
    
# INITIAL GUESS
opti.set_initial(wp4, 0.98*pmax)
opti.set_initial(T, Tmax/2)


# SOLVER
p_opts = {"expand":True}
s_opts = {"max_iter": 1000}
opti.solver("ipopt",p_opts,
                    s_opts)

sol = opti.solve()

# evaluate
Tsol = sol.value(T)
wpsol = sol.value(wp)
wvsol = sol.value(wv)
wasol = sol.value(wa)

tauvsol = sol.value(tauv)
tvsol = sol.value(tv)
av_altsol = sol.value(av_alt)
avsol = sol.value(av)
vvsol = sol.value(vv)
pvsol = sol.value(pv)


print('Tsol: ', Tsol)
print('wpsol: ', wpsol)
print('wvsol: ', wvsol)
print('wasol: ', wasol)


# plot bezier
fig, axs = plt.subplots(3, 1, figsize=(10, 5))

time, posz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
velz = bezierTraj(wvsol/Tsol, T0=0, Tf=Tsol, step=0.002)[1]
accz = bezierTraj(wasol/(Tsol**2), T0=0, Tf=Tsol, step=0.002)[1]

axs[0].plot(time, posz)
axs[0].set_ylabel('position [m]')
axs[0].hlines(pmin, 0, time[-1], linestyles='dashed')
axs[0].hlines(pmax, 0, time[-1], linestyles='dashed')
axs[0].plot(0, p0, marker="o", markersize=5)
color0 = axs[0].lines[0].get_color()
axs[0].vlines(tvsol, pmin, pmax, linestyles='dashed', colors=color0)
if pf is not None and (pmin < pf < pmax):
    axs[0].plot(time[-1], pf, marker="o", markersize=5)
axs[0].grid()


axs[1].plot(time, velz)
axs[1].set_ylabel('velocity [m/s]')
axs[1].plot(0, v0, marker="o", markersize=5)
axs[1].vlines(tvsol, velz[0], velz[-1], linestyles='dashed', colors=color0)
axs[1].grid()


axs[2].plot(time, accz)
axs[2].set_xlabel('time [s]')
axs[2].set_ylabel('acceleration [m/s^2]')
axs[2].hlines(0, 0, time[-1], linestyles='dashed')
axs[2].hlines(amax, 0, time[-1], linestyles='dashed')
axs[2].vlines(tauvsol*Tsol, 0, amax, linestyles='dashed', colors=color0)
axs[2].grid()

plt.subplots_adjust(right=0.85)

####################
# VHSIP TRAJECTORY #
####################
L = p0
vhsip = VHSIP(dt=0.002, L=L, g_mag=9.81, w_v=1., w_p=1., w_u=1.)

state_x0 = np.array([[-30],
                     [0.]])

state_y0 = np.array([[0.],
                     [0.]])

state_z0 = np.array([[v0],
                     [0.]])

vhsip.set_init(state_x0, state_y0, state_z0)
vhsip.set_z_dynamics(time, posz, velz, accz)
vhsip.solve_ocp()
vhsip.compute_xy_dynamics(vhsip.zmp_xy[0], vhsip.zmp_xy[1])

fig, axs = plt.subplots(3, 1,  figsize=(10,5))

axs[0].set_ylabel("c [$m$]")
axs[0].plot(vhsip.time, vhsip.T_p_com_ref.T)
axs[0].grid()

axs[1].set_ylabel("$\dot{c}$ [$m/s$]")
axs[1].plot(vhsip.time, vhsip.T_v_com_ref.T)
axs[1].grid()


axs[2].set_ylabel("$\ddot{c}$ [$m/s^2$]")
axs[2].plot(vhsip.time, vhsip.T_a_com_ref.T)
axs[2].grid()

plt.show()

legend = [Line2D([0], [0], color=axs[0].lines[0].get_color(), lw=4, label="x"),
          Line2D([0], [0], color=axs[0].lines[1].get_color(), lw=4, label="y"),
          Line2D([0], [0], color=axs[0].lines[2].get_color(), lw=4, label="z")]

fig.legend(handles=legend,
           loc="center right",  # Position of legend
           borderaxespad=1,  # Small spacing around legend box
           )

plt.subplots_adjust(right=0.85)




