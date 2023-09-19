import casadi as ca
import numpy as np

f = ca.external('f', './gen.so')
x = np.array([0, 1])
y = 4

out = f(x, y)

print(out[0] + out[1])