import numpy as np
from lib.calcJacobian import calcJacobian
import math


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    J = calcJacobian(q_in)


    # dq = np.zeros((1, 7))
    # J =  calcJacobian(q_in)
    # v_in = v_in.reshape(-1,1)
    # omega_in = omega_in.reshape(-1,1)
    #
    # v= []
    # omega = []
    # # x = float(np.nan)
    #
    # for i in range(len(v_in)):
    #     if math.isnan(v_in[i]):
    #
    #         v_in[i] = []
    #         v.append(v_in[i])
    #     else:
    #         v.append(v_in[i])
    #
    # for i in range(len(omega_in)):
    #     if math.isnan(omega_in[i]):
    #         # print('hello')
    #         omega_in[i] = []
    #         omega.append(omega_in[i])
    #     else:
    #         omega.append(omega_in[i])
    #
    #
    velocity = np.hstack((v_in, omega_in)).reshape([6,1])

    indexList = [np.any(i) for i in np.isnan(velocity)]
    # delete all the rows with any NaN value
    velocity = np.delete(velocity, indexList, axis =0)
    J = np.delete(J, indexList, axis = 0)


    # zero_vel = list(map(tuple, np.where(np.isnan(velocity))))[0]
    #
    # J = np.delete(J, zero_vel, axis = 0)
    #
    # velocity = np.delete(velocity, zero_vel, axis = 0)

    # #
    # # J_inv = np.linalg.pinv(J)
    # # #
    # # dq= J_inv@velocity
    #
    f = np.linalg.lstsq(J, velocity, rcond=None)[0]
    # dq=dq.reshape(7)
    # print(dq)


    # f = np.linalg.lstsq(J, velocity ,rcond=None)[0]
    #
    # # dq = f.T @ velocity
    dq = f.reshape(7)

    return dq
