import casadi as ca
import numpy as np
from landing_controller.controller.extended.utils import *
import os

p0=0.25
v0=1
amax=200
pmin=0.1
pmax=0.3
Tfmax=0.3
Tf=None
pf=None
vf=2
Y0=None
N=5
ctrl_points = np.linspace(0, 1, num=6, endpoint=True)




if N is None:
    N = 3

X = []
X0 = []

wp_list = []

# optimization variables: bezier coefficient, time
if Tf is None:
    T = ca.SX.sym('T', 1)
    X.append(T)
    if Tfmax is not None:
        X0.append(Tfmax)
    else:
        X0.append(0.5)
else:
    T = Tf

# wp0 = opti.variable(1) no need of this
wp0 = p0
wp_list.append(wp0)
for i in range(1, N):
    X.append(ca.SX.sym('wp' + str(i), 1))
    wp_list.append(X[-1])
    X0.append(0.0)

if pf is None or not (pmin < pf < pmax):  # if pf is given, set w4=pf
    wpN = ca.SX.sym('wp' + str(N), 1)
    X.append(wpN)
    wp_list.append(X[-1])
    X0.append(0.98 * pmax)
else:
    wpN = pf
    wp_list.append(wpN)

# opt variables
X = ca.vcat(X)

# for simplify computations, let's define coeffs of bezier derivatives (do not consider time)
wp = ca.Function('wp', [X], [ca.vcat(wp_list)])

wv_list = []
for i in range(N):
    wv_list.append(N * (wp_list[i + 1] - wp_list[i]))
wv = ca.Function('wv', [X], [ca.vcat(wv_list)])

wa_list = []
for i in range(N - 1):
    wa_list.append((N - 1) * (wv_list[i + 1] - wv_list[i]))
wa = ca.Function('wa', [X], [ca.vcat(wa_list)])

# COST
cost = 1  # -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

# CONSTRAINTS
constrs = []
ubconstrs = []
lbconstrs = []
# initial position
# opti.subject_to(wp0==p0)
# #
#
# # initial velocity
# init_vel = wv_list[0] / T
# constrs.append(init_vel - v0)
# ubconstrs.append(0.0)
# lbconstrs.append(0.0)
#
# # causality, limited time
# if Tf is None:
#     constrs.append(T)
#     if Tfmax is None:
#         ubconstrs.append(+np.inf)
#         lbconstrs.append(0.)
#     else:
#         ubconstrs.append(1. * Tfmax)
#         lbconstrs.append(0. * Tfmax)
#
# for tau in ctrl_points:
#     # the acceleration is postive and upper bounded
#     constrs.append(bezier(wa(X) / T ** 2, tau))
#     ubconstrs.append(amax)
#     # lbconstrs.append(-self.g_mag)
#     lbconstrs.append(0.0)
#
#     # bounds on position
#     constrs.append(bezier(wp(X), tau))
#     ubconstrs.append(pmax)
#     if tau == ctrl_points[-1]:
#         lbconstrs.append((pmax + p0) / 2)  # does not allow for squatty jumps
#     else:
#         lbconstrs.append(pmin)
#     #
#     # # bounds on velocity
#     # if vf is not None:
#     #     constrs.append(bezier(wv(X), tau))
#     #     ubconstrs.append(vf)
#     #     if tau == self.ctrl_points[-1]:
#     #         lbconstrs.append(vf)
#     #     else:
#     #         lbconstrs.append(-10)
#
# if vf is None:
#     # positive final velocity
#     constrs.append(wv_list[N - 1])
#     ubconstrs.append(np.inf)
#     lbconstrs.append(0.0)
# else:
#     # if vf is given, equality constaint
#     constrs.append(wv_list[N - 1] / T - vf)
#     ubconstrs.append(0.0)
#     lbconstrs.append(0.0)
#
# # SOLVER
nlp = {'x': X, 'f': cost, 'g': ca.vcat(constrs)}
#opts = {'ipopt.print_level': 0, 'print_time': 0}
opts = {}

# opts['expand'] = True
# opts['verbose'] = False
# opts['print_time'] = 0
# opts['options_file'] = os.environ['HOME']+'/knitro-13.2.0-Linux-64/examples/Python/examples/knitro.opt'
# opts['jit'] = False
# opts['compiler'] = 'shell'

S = ca.nlpsol('S', 'knitro', nlp, opts)

if Y0 is not None:
    X0 = Y0

r = S(x0=ca.vcat(X0), lbg=ca.vcat(lbconstrs), ubg=ca.vcat(ubconstrs))

# evaluate
# X = [T, wp1, wp2, wp3, wp4]
Tsol = r['x'][0]
wpsol = wp(r['x'])
wvsol = wv(r['x'])
wasol = wa(r['x'])

time, pz = bezierTraj(wpsol, T0=0, Tf=Tsol, step=0.002)
vz = bezierTraj(wvsol / Tsol, T0=0, Tf=Tsol, step=0.002)[1]
az = bezierTraj(wasol / (Tsol ** 2), T0=0, Tf=Tsol, step=0.002)[1]