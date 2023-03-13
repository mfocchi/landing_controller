import numpy as np
import pinocchio as pin
from base_controllers.utils.math_tools import *
import time
mathJet = Math()

w_euler_e = np.array([0, -0.1, np.pi/2])
w_euler_des = np.array([0.1, 0.1, np.pi/2])

start = time.time()
# using Michele's functions
w_R_e = mathJet.eul2Rot(w_euler_e)
w_R_des = mathJet.eul2Rot(w_euler_des)
# w_err = computeOrientationError(w_R_e, w_R_des)

# here, I pasted the content of compute Orientation Error, for making comparisons
e_R_des = w_R_e.T.dot(w_R_des)
# compute the angle-axis representation of the associated orientation error
# compute the angle: method 2) with atan2
delta_theta = math.atan2(np.sqrt(
    pow(e_R_des[2, 1] - e_R_des[1, 2], 2) + pow(e_R_des[0, 2] - e_R_des[2, 0], 2) + pow(
        e_R_des[1, 0] - e_R_des[0, 1], 2)), e_R_des[0, 0] + e_R_des[1, 1] + e_R_des[2, 2] - 1)
# compute the axis (deal with singularity)
if delta_theta == 0.0:
    e_error_o = np.zeros(3)
else:
    r_hat = 1 / (2 * np.sin(delta_theta)) * np.array(
        [e_R_des[2, 1] - e_R_des[1, 2], e_R_des[0, 2] - e_R_des[2, 0], e_R_des[1, 0] - e_R_des[0, 1]])
    # compute the orientation error
    e_error_o = delta_theta * r_hat
#    # the error is expressed in the end-effector frame
#    # we need to map it in the world frame to compute the moment because the jacobian is in the WF
w_error_o = w_R_e.dot(e_error_o)
end = time.time()

# using Pinocchio (they are faster)
start_pin = time.time()
w_R_e_pin = pin.rpy.rpyToMatrix(w_euler_e)
w_R_des_pin = pin.rpy.rpyToMatrix(w_euler_des)
e_R_des_pin = w_R_e.T.dot(w_R_des)
aa = pin.AngleAxis(e_R_des)
e_error_o_pin = aa.angle * aa.axis
w_error_o_pin = w_R_e.dot(e_error_o_pin)
end_pin = time.time()

print('*********')
print('* w_R_e *')
print('*********')
print('Michele')
print(w_R_e)
print('Pinocchio')
print(w_R_e_pin)

print('***********')
print('* w_R_des *')
print('***********')
print('Michele')
print(w_R_des)
print('Pinocchio')
print(w_R_des_pin)

print('***********')
print('* e_R_des *')
print('***********')
print('Michele')
print(e_R_des)
print('Pinocchio')
print(e_R_des_pin)

print('*************')
print('* e_error_o *')
print('*************')
print('Michele')
print(e_error_o)
print('Pinocchio')
print(e_error_o_pin)

print('*************')
print('* w_error_o *')
print('*************')
print('Michele')
print(w_error_o)
print('Pinocchio')
print(w_error_o_pin)

print('****************')
print('* elapsed time *')
print('****************')
print('Michele')
print(end-start)
print('Pinocchio')
print(end_pin-start_pin)




Kp = np.array([100, 200, 300])
Kp_mat = np.diag(Kp)
w_R_hf = mathJet.eul2Rot((np.array([0, 0, np.pi/4])))

Kp_w_mat = np.zeros((3,3))
Kp_w_mat[2,2] = Kp_mat[2,2]

c_pow2 = w_R_hf[0, 0] ** 2
s_pow2 = w_R_hf[0, 1] ** 2
sc = w_R_hf[0, 0] * w_R_hf[0, 1]
start = time.time()
for i in range(4):
    Kp_w_mat[0, 0] = c_pow2 * Kp_mat[0,0] + s_pow2 * Kp_mat[1,1]
    Kp_w_mat[1, 1] = s_pow2 * Kp_mat[0, 0] + c_pow2 * Kp_mat[1, 1]
    Kp_w_mat[1, 0] = Kp_w_mat[0,1] = sc * (Kp_mat[1,1]-Kp_mat[0,0])
end = time.time()
print('[Method 1] elapsed time = ', (end-start))
print('Kp_w_mat\n', Kp_w_mat)


start = time.time()
for i in range(4):
    Kp_w_mat = w_R_hf.T@Kp_mat@w_R_hf
end = time.time()
print('[Method 2] elapsed time = ', (end-start))
print('Kp_w_mat\n', Kp_w_mat)
