import casadi as ca
import numpy as np
from  scipy.special import comb
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from termcolor import colored
import time as t_os


def bezier4(w, tau):
    b = w[0] * (1 - tau) ** 4 + \
        4 * w[1] * (1 - tau) ** 3 * tau + \
        6 * w[2] * (1 - tau) ** 2 * tau ** 2 + \
        4 * w[3] * (1 - tau) ** 1 * tau ** 3 + \
        w[4] * tau ** 4
    return b

def bezier3(w, tau):
    b = w[0] * (1 - tau) ** 3 + \
        3 * w[1] * (1 - tau) ** 2 * tau + \
        3 * w[2] * (1 - tau) ** 1 * tau ** 2 + \
        w[3]* tau ** 3
    return b

def bezier2(w, tau):
    b = w[0] * (1 - tau) ** 2 + \
        2 * w[1] * (1 - tau)  * tau + \
        w[2] * tau ** 2
    return b

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


#### INPUT
# initial position
p0 = 0.25
# final position
# pf = 0.29999
# initial velocity
v0 = -3.0
# final velocity
vf = 2
# max time (I'm not sure it is actually needed)
Tmax = 2
# boundary on position
pmin = 0.1
pmax = 0.3# np.max([p0, pf])
# boundary on acceleration
m = 12.5
amax = 500/m

ctrl_knots = np.linspace(0, 1, num=6)

# plot bezier
fig, axs = plt.subplots(3,1, figsize=(10,5))

tic0 = t_os.time()
#### NLP formulation
opti = ca.Opti()

# optimization variables: bezier coefficient, time

T = opti.variable(1)

# wp0 = opti.variable(1) no need of this
wp0 = p0
wp1 = opti.variable(1)
wp2 = opti.variable(1)
wp3 = opti.variable(1)
if 'pf' in locals() and pmin < pf < pmax:  # if pf is given, set w4=pf
    wp4 = pf
else:
    wp4 = opti.variable(1)

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

toc0 = t_os.time()

# CONSTRAINTS
# initial position
# opti.subject_to(wp0==p0)
#

tic1 = t_os.time()
#
# initial velocity
opti.subject_to(wv0/T == v0)

# causality, limited time
opti.subject_to(T > 0)
opti.subject_to(T <= Tmax)


for tau in ctrl_knots:
    # the acceleration is postive and upper bounded
    opti.subject_to(bezier2(wa/(T**2), tau) <= amax)
    opti.subject_to(bezier2(wa/(T**2), tau) >= 0.0)

    # bounds on position
    if tau == ctrl_knots[0]:
        continue
    ticc = t_os.time()
    opti.subject_to(bezier4(wp, tau) <= pmax)
    tocc = t_os.time()
    print('bezier', tocc-ticc)
    if tau == ctrl_knots[-1]:
        opti.subject_to(bezier4(wp, tau) >= (pmax + p0) / 2)  # does not allow for squatty jumps
    else:
        opti.subject_to(bezier4(wp, tau) >= pmin)

if vf is None:
    # positive final velocity
    opti.subject_to(bezier3(wv/T, 1) >= 0.)
else:
    # if vf is given, equality constaint
    opti.subject_to(bezier3(wv/T, 1) == vf)

# solver
p_opts = {'expand': False}# this speeds up ~x10
s_opts = {'linsolver':4,  #4 works well
        'outlev': 2, #0
        'strat_warm_start':1,
        'algorithm':0,
        'bar_murule':2, # 5 works well
        'feastol':1e-4,
        'tuner':0,
        'bar_feasible':0, #0 works well
        'bar_directinterval':10,
        'maxit':1500}
opti.solver('knitro', p_opts, s_opts);

# initial variable, not set are 0
opti.set_initial(wp4, 0.98*pmax)
opti.set_initial(T, Tmax/2)

# cost
cost = 1
opti.minimize(cost)
toc1 = t_os.time()
tic2 = t_os.time()
sol = opti.solve()
toc2 = t_os.time()

# evaluate
Tsol = sol.value(T)
wpsol = sol.value(wp)
wvsol = sol.value(wv)
wasol = sol.value(wa)


time, pos = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
vel = bezierTraj(wvsol/Tsol, T0=0, Tf=Tsol, step=0.002)[1]
acc = bezierTraj(wasol/(Tsol**2), T0=0, Tf=Tsol, step=0.002)[1]

print(toc0-tic0)
print(toc1-tic1)
print(toc2-tic2)

axs[0].plot(time, pos)


axs[1].plot(time, vel)


axs[2].plot(time, acc)

axs[0].set_ylabel('position [m]')
axs[0].grid()



axs[1].set_ylabel('velocity [m/s]')
axs[1].grid()

axs[2].set_xlabel('time [s]')
axs[2].set_ylabel('acceleration [m/s^2]')
axs[2].grid()

plt.subplots_adjust(right=0.85)