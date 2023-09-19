import casadi as ca
import numpy as np
import time

# Create a new NLP solver instance from the compiled code
opts = {"ipopt.linear_solver": "ma57"}
solver = ca.nlpsol("solver", "ipopt", "./nlp.so", opts)
ubconstrsFunction = ca.external("ubconstrs", "./nlp.so")
lbconstrsFunction = ca.external("lbconstrs", "./nlp.so")

times = []
X0val    = np.zeros(6)
X0val[0] = 0.1
for i in range(-10, 10):
    p0val    = 0.25
    v0val    = -3+0.1*i
    amaxval  = 600/12.5
    pminval  = 0.1
    pmaxval  = 0.3
    Tfmaxval = 0.2
    vfval    = 2


    pval = np.array([p0val   ,
                     v0val   ,
                     amaxval ,
                     pminval ,
                     pmaxval ,
                     Tfmaxval,
                     vfval   ])


    tic = time.time()
    # Solve the NLP
    res = solver(
                ubg = ubconstrsFunction(pval),
                lbg = lbconstrsFunction(pval),
                x0 = X0val,
                p = pval)
    toc = time.time()
    times.append(toc-tic)

    # Print solution
    print("-----")
    print("objective at solution =", res["f"])
    print("primal solution =", res["x"])
    print("dual solution (x) =", res["lam_x"])
    print("dual solution (g) =", res["lam_g"])

    X0val = res['x']