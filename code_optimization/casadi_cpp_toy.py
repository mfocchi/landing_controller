import casadi as ca

x = ca.MX.sym('x')
y = ca.MX.sym('y')
p = ca.MX.sym('p')

f = ca.sin(y)*x
g = x**2-p
nlp = {'x': ca.vcat([x, y]), 'f': f, 'g': g, 'p':p}
S = ca.nlpsol('S', 'knitro', nlp)

r = S()

opt_func = ca.Function('opt_func', [p], [S])
#
# C = ca.CodeGenerator('gen.c')
# C.add(opt_func)
# C.generate()
