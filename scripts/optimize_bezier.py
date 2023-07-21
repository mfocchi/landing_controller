import numpy as np
from scipy.optimize import *
from scipy.sparse import *

p0 = 0.25
pf = 0.25
v0 = -1
amax = 800/15
p_min = 0.12

# x = [w0, w1, w2, w3, w4, T]

def cost(x):
    # max final velocity
    w0 = x[0]
    w1 = x[1]
    w2 = x[2]
    w3 = x[3]
    w4 = x[4]
    T  = x[5]
    vf = 4*(w4-w3)/T
    return vf

def cost_jac(x):
    dc_dx3 = -4 / x[5]
    dc_dx4 = 4 / x[5]
    dc_dx5 = -4 * (x[4] - x[3]) / x[5]
    row = np.array([0, 0, 0])
    col = np.array([3, 4, 5])
    data = np.array([dc_dx3, dc_dx4, dc_dx5])
    J = csc_array((data, (row, col)), shape=(1, 6))
    return J

def cost_hess(x):
    d2c_dx3x5 =  4 / x[5]**2
    d2c_dx4x5 = -4 / x[5]**2
    d2c_dx5x5 = 8 /  x[5]**3

    row = np.array([3, 4, 5, 5, 5])
    col = np.array([5, 5, 3, 4, 5])
    data = np.array([d2c_dx3x5, d2c_dx4x5, d2c_dx3x5, d2c_dx4x5, d2c_dx5x5])
    H = csc_array((data, (row, col)), shape=(6, 6))
    return H

def const(x):
    pass

### linear constraints ###
# initial position: w0 = p0
# final position: w4 = pf
# time positive: T > 0
A0 = np.array([[1., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 1., 0.],
               [0., 0., 0., 0., 0., 1.]])
b0_lb = np.array([p0, pf, 0.])
b0_ub = np.array([p0, pf, 2.]) # potentially time < +inf, actually this is impossible
# b0_lb <= A0 * x <= b0_ub
initialPosition = LinearConstraint(A0, b0_lb, b0_ub)


### nonlinear constraints ###
# initial velocity: w0' = 4*T * (w1-w0) = v0
# bounded acceleration
# bounded position
