import casadi as cs
import os
print(os.environ['LD_PRELOAD'])
optimizer = 'knitro'
x = cs.MX.sym('x')
nlp = {'x': x, 'f': (x - 1) ** 2}
nlp_solver = cs.nlpsol('nlp_solver', optimizer, nlp)
sol = nlp_solver(x0=[3], lbx=[2], ubx=[10])
print(sol)