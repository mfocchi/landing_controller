import casadi as ca
import numpy as np
from landing_controller.controller.extended.utils import *

p0    = ca.MX.sym('p0')
v0    = ca.MX.sym('v0')
amax  = ca.MX.sym('amax')
pmin  = ca.MX.sym('pmin')
pmax  = ca.MX.sym('pmax')
Tfmax = ca.MX.sym('Tfmax')
vf    = ca.MX.sym('vf')
#X0    = ca.SX.sym('X0', 6)
N = 5

params = ca.vcat([p0, v0, amax, pmin, pmax, Tfmax, vf])#, X0])

ctrl_knots = np.linspace(0, 1, num=6, endpoint=True)
X = []
wp_list = []

T = ca.MX.sym('T', 1)
X.append(T)

wp0 = p0
wp_list.append(wp0)
for i in range(1, N):
    X.append(ca.MX.sym('wp' + str(i), 1))
    wp_list.append(X[-1])

wpN = ca.MX.sym('wp' + str(N), 1)
X.append(wpN)
wp_list.append(X[-1])

# opt variables
X = ca.vcat(X)

# for simplify computations, let's define coeffs of bezier derivatives (do not consider time)
wp = ca.Function('wp', [p0, X], [ca.vcat(wp_list)])

wv_list = []
for i in range(N):
    wv_list.append(N * (wp_list[i + 1] - wp_list[i]))
wv = ca.Function('wv', [p0, X], [ca.vcat(wv_list)])

wa_list = []
for i in range(N - 1):
    wa_list.append((N - 1) * (wv_list[i + 1] - wv_list[i]))
wa = ca.Function('wa', [p0, X], [ca.vcat(wa_list)])


# COST
cost = 1 # -(bezier(wv(X) / T, 1)) ** 2  # 'max vf^2'

# CONSTRAINTS
constrs = []
ubconstrs = []
lbconstrs = []
# initial position
# opti.subject_to(wp0==p0)
#

# initial velocity
init_vel = wv_list[0] / T
constrs.append(init_vel - v0)
ubconstrs.append(0.0)
lbconstrs.append(0.0)

# causality, limited time
constrs.append(T)
if Tfmax is None:
    ubconstrs.append(+np.inf)
    lbconstrs.append(0.)
else:
    ubconstrs.append(3. * Tfmax)
    lbconstrs.append(0.)

for tau in ctrl_knots:
    # the acceleration is postive and upper bounded
    constrs.append(bezier(wa(p0, X) / T ** 2, tau))
    ubconstrs.append(amax)
    lbconstrs.append(0.0)

    # bounds on position
    constrs.append(bezier(wp(p0, X), tau))
    ubconstrs.append(pmax)
    if tau == ctrl_knots[-1]:
        lbconstrs.append((pmax + p0) / 2)  # does not allow for squatty jumps
    else:
        lbconstrs.append(pmin)

if vf is None:
    # positive final velocity
    constrs.append(wv_list[N - 1])
    ubconstrs.append(np.inf)
    lbconstrs.append(0.0)
else:
    # if vf is given, equality constaint
    constrs.append(wv_list[N - 1] / T - vf)
    ubconstrs.append(0.0)
    lbconstrs.append(0.0)

ubconstrsFunction = ca.Function('ubconstrs', [params], [ca.vcat(ubconstrs)])
lbconstrsFunction = ca.Function('lbconstrs', [params], [ca.vcat(lbconstrs)])

# SOLVER
nlp = {'x': X, 'f': cost, 'g': ca.vcat(constrs), 'p':params}
# Pick a compiler
compiler = "gcc"  # Linux
# compiler = "clang"  # OSX
# compiler = "cl.exe" # Windows

# Run this script in an environment that recognised the compiler as command.
# On Windows, the suggested way is to run this script from a "x64 Native Tools Command Promt for VS" (Requires Visual C++ components or Build Tools for Visual Studio, available from Visual Studio installer. You also need SDK libraries in order to access stdio and math.)

flags = ["-O3"]  # Linux/OSX

# Create an NLP solver instance
opts = {
            "ipopt.print_level": 8,
            "ipopt.linear_solver": "ma57",
            "ipopt.ma57_pivtol": 1e-6,
            "ipopt.nlp_scaling_max_gradient": 100.0,
            "ipopt.nlp_scaling_min_value": 1e-6,
            "ipopt.tol": 1e-3,
            "ipopt.dual_inf_tol": 1000.0,
            "ipopt.compl_inf_tol": 1e-2,
            "ipopt.constr_viol_tol": 1e-3,
            "ipopt.acceptable_tol": 1e0,
            "ipopt.acceptable_iter": 1,
            "ipopt.acceptable_compl_inf_tol": 1,
            "ipopt.alpha_for_y": "dual-and-full",
            "ipopt.max_iter": 4000,
            "ipopt.warm_start_bound_frac": 1e-2,
            "ipopt.warm_start_bound_push": 1e-2,
            "ipopt.warm_start_mult_bound_push": 1e-2,
            "ipopt.warm_start_slack_bound_frac": 1e-2,
            "ipopt.warm_start_slack_bound_push": 1e-2,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.required_infeasibility_reduction": 0.8,
            "ipopt.perturb_dec_fact": 0.1,
            "ipopt.max_hessian_perturbation": 10000,
            "ipopt.fast_step_computation": "yes",
            "ipopt.hessian_approximation": "exact",
        }
solver = ca.nlpsol("solver", "ipopt", nlp, opts)
#
# # Generate C code for the NLP functions
solver.generate_dependencies("solver.c")

lbconstrsFunction.generate("lbconstrs.c")
ubconstrsFunction.generate("ubconstrs.c")

wp.generate("wp.c")
wv.generate("wv.c")
wa.generate("wa.c")


#
import subprocess
#
# # On Windows, use other flags
cmd_args = [compiler, "-fPIC", "-shared"] + flags + ["solver.c", "lbconstrs.c", "ubconstrs.c", "wp.c", "wv.c", "wa.c", "-o", "nlp.so"]
subprocess.run(cmd_args)

#
# solver = ca.nlpsol("solver", "ipopt", "./nlp.so")
# # ubconstrsFunction = ca.Function("lbconstrs", "./nlp.so")
# # lbconstrsFunction = ca.Function("ubconstrs", "./nlp.so")
# arg = {}
#
# p0val    = 0.25
# v0val    = -3
# amaxval  = 600/12.5
# pminval  = 0.1
# pmaxval  = 0.3
# Tfmaxval = 0.2
# vfval    = 2
# X0val    = np.zeros(6)
# X0val[0] = 0.1
#
#
#
# pval = np.array([p0val   ,
#                        v0val   ,
#                        amaxval ,
#                        pminval ,
#                        pmaxval ,
#                        Tfmaxval,
#                        vfval   ])
# # Solve the NLP
# res = solver(
#             ubg = ubconstrsFunction(pval),
#             lbg = lbconstrsFunction(pval),
#             x0 =  X0val,
#             p = pval)
#
#
# # Print solution
# print("-----")
# print("objective at solution =", res["f"])
# print("primal solution =", res["x"])
# print("dual solution (x) =", res["lam_x"])
# print("dual solution (g) =", res["lam_g"])