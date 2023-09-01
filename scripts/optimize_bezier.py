import casadi as ca
import numpy as np
from  scipy.special import comb
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from termcolor import colored

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
#vf = 2.8
# max time (I'm not sure it is actually needed)
Tmax = 2
# boundary on position
pmin = 0.1
pmax = 0.3# np.max([p0, pf])
# boundary on acceleration
m = 12.5
amax = 800/m


# plot bezier
fig, axs = plt.subplots(3,1, figsize=(10,5))



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

if 'vf' in locals():
    # if vf is given, equality constaint
    opti.subject_to(wv3/T==vf) # (not tested)
else:
    # positive final velocity
    opti.subject_to(wv3 >= 0)

# solver
p_opts = {}#{"expand":True}
s_opts = {"max_iter": 1000}
opti.solver("ipopt",p_opts,
                    s_opts)

# initial variable, not set are 0
opti.set_initial(wp4, 0.98*pmax)
opti.set_initial(T, Tmax/2)

# cost
costs = []
# costs.append({'f':  wv[-1]/T, 'name': 'min vf'})
# costs.append({'f': -wv[-1]/T, 'name': 'max vf'})
costs.append({'f': -(wv[-1]/T)**2, 'name': 'max vf^2'})
# # costs.append({'f': -wa[-1]/(T**2), 'name': 'max af'})
# # costs.append({'f': -(wa[-1]/(T**2))**2, 'name': 'max af^2'})
# costs.append({'f': T, 'name': 'min T'})
# costs.append({'f': T**2, 'name': 'min T^2'})
# costs.append({'f': -T, 'name': 'max T'})
# costs.append({'f': -T**2, 'name': 'max T^2'})


legend = []
Tsolmax = 0.0
for cost in costs:
    print(colored(cost['name'], 'blue'), flush=True)
    opti.minimize(cost['f'])
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

    asol = sol.value(a)
    bsol = sol.value(b)
    csol = sol.value(c)


    print('Tsol: ', Tsol)
    Tsolmax = max(Tsol, Tsolmax)
    print('wpsol: ', wpsol)
    print('wvsol: ', wvsol)
    print('wasol: ', wasol)

    print('tauvsol: ', tauvsol)
    print('tvsol: ', tvsol)
    print('av_altsol: ', av_altsol)
    print('avsol: ', avsol)
    print('vvsol: ', vvsol)
    print('pvsol: ', pvsol)

    print('asol: ', asol)
    print('bsol: ', bsol)
    print('csol: ', csol)





    time, pos = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
    vel = bezierTraj(wvsol/Tsol, T0=0, Tf=Tsol, step=0.002)[1]
    acc = bezierTraj(wasol/(Tsol**2), T0=0, Tf=Tsol, step=0.002)[1]

    axs[0].plot(time, pos)
    axs[0].vlines(tvsol, pmin, pmax, linestyles='dashed', colors=axs[0].lines[-1].get_color())


    axs[1].plot(time, vel)
    axs[1].vlines(tvsol, vel[0], vel[-1], linestyles='dashed', colors=axs[0].lines[-1].get_color())


    axs[2].plot(time, acc)
    axs[2].vlines(tauvsol*Tsol, 0, amax, linestyles='dashed', colors=axs[0].lines[-1].get_color())

    legend.append(Line2D([0], [0], color=axs[0].lines[-1].get_color(), lw=4, label=cost['name'] + " (" + str(opti.stats()['iter_count']) +")"))

axs[0].set_ylabel('position [m]')
axs[0].hlines(pmin, 0, Tsolmax, linestyles='dashed')
axs[0].hlines(pmax, 0, Tsolmax, linestyles='dashed')
axs[0].plot(0, p0, marker="o", markersize=5)
if 'pf' in locals() and pmin < pf < pmax:
    axs[0].plot(Tsolmax, pf, marker="o", markersize=5)
axs[0].grid()



axs[1].set_ylabel('velocity [m/s]')
axs[1].plot(0, v0, marker="o", markersize=5)
axs[1].grid()

axs[2].set_xlabel('time [s]')
axs[2].set_ylabel('acceleration [m/s^2]')
axs[2].hlines(0, 0, Tsolmax, linestyles='dashed')
axs[2].hlines(amax, 0, Tsolmax, linestyles='dashed')
axs[2].grid()

fig.legend(handles=legend,
              loc="center right",  # Position of legend
              borderaxespad=1,  # Small spacing around legend box
              title="cost func")

plt.subplots_adjust(right=0.85)