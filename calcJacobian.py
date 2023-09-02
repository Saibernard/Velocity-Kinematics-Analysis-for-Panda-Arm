import numpy as np
# from lib.calculateFK import FK
from math import pi

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    # J = np.zeros((6, 7))
    alpha = [0, -pi / 2, pi / 2, pi / 2, pi / 2, pi / 2, -pi / 2, 0]
    d = [0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.210]
    a = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
    # q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = q_in

    t1_w = np.array([[np.cos(0), -np.sin(0) * (np.cos(alpha[0])), np.sin(0) * (np.sin(alpha[0])),
                      a[0] * (np.cos(0))], [np.sin(0), np.cos(0) * (np.cos(alpha[0])),
                                               -np.cos(0) * (np.sin(alpha[0])), a[0] * (np.sin(0))],
                     [0, np.sin(alpha[0]), np.cos(alpha[0]), d[0]], [0, 0, 0, 1]])
    t2_1 = np.array([[np.cos(q[0]), -np.sin(q[0]) * (np.cos(alpha[1])), np.sin(q[0]) * (np.sin(alpha[1])),
                      a[1] * (np.cos(q[0]))], [np.sin(q[0]), np.cos(q[0]) * (np.cos(alpha[1])),
                                               -np.cos(q[0]) * (np.sin(alpha[1])), a[1] * (np.sin(q[0]))],
                     [0, np.sin(alpha[1]), np.cos(alpha[1]), d[1]], [0, 0, 0, 1]])
    t3_2 = np.array([[np.cos(q[1]), -np.sin(q[1]) * (np.cos(alpha[2])), np.sin(q[1]) * (np.sin(alpha[2])),
                      a[2] * (np.cos(q[1]))], [np.sin(q[1]), np.cos(q[1]) * (np.cos(alpha[2])),
                                               -np.cos(q[1]) * (np.sin(alpha[2])), a[2] * (np.sin(q[1]))],
                     [0, np.sin(alpha[2]), np.cos(alpha[2]), d[2]], [0, 0, 0, 1]])
    t4_3 = np.array([[np.cos(q[2]), -np.sin(q[2]) * (np.cos(alpha[3])), np.sin(q[2]) * (np.sin(alpha[3])),
                      a[3] * (np.cos(q[2]))], [np.sin(q[2]), np.cos(q[2]) * (np.cos(alpha[3])),
                                               -np.cos(q[2]) * (np.sin(alpha[3])), a[3] * (np.sin(q[2]))],
                     [0, np.sin(alpha[3]), np.cos(alpha[3]), d[3]], [0, 0, 0, 1]])
    t5_4 = np.array([[np.cos(q[3]-pi), -np.sin(q[3]-pi) * (np.cos(alpha[4])), np.sin(q[3]-pi) * (np.sin(alpha[4])),
                      a[4] * (np.cos(q[3]-pi))], [np.sin(q[3]-pi), np.cos(q[3]-pi) * (np.cos(alpha[4])),
                                                -np.cos(q[3]-pi) * (np.sin(alpha[4])), a[4] * (np.sin(q[3]-pi))],
                     [0, np.sin(alpha[4]), np.cos(alpha[4]), d[4]], [0, 0, 0, 1]])
    t6_5 = np.array([[np.cos(q[4]), -np.sin(q[4]) * (np.cos(alpha[5])), np.sin(q[4]) * (np.sin(alpha[5])),
                      a[5] * (np.cos(q[4]))], [np.sin(q[4]), np.cos(q[4]) * (np.cos(alpha[5])),
                                               -np.cos(q[4]) * (np.sin(alpha[5])), a[5] * (np.sin(q[4]))],
                     [0, np.sin(alpha[5]), np.cos(alpha[5]), d[5]], [0, 0, 0, 1]])
    t7_6 = np.array([[np.cos(pi-q[5]), -np.sin(pi-q[5]) * (np.cos(alpha[6])), np.sin(pi-q[5]) * (np.sin(alpha[6])),
                      a[6] * (np.cos(pi-q[5]))], [np.sin(pi-q[5]), np.cos(pi-q[5]) * (np.cos(alpha[6])),
                                               -np.cos(pi-q[5]) * (np.sin(alpha[6])), a[6] * (np.sin(pi-q[5]))],
                     [0, np.sin(alpha[6]), np.cos(alpha[6]), d[6]], [0, 0, 0, 1]])
    te_7 = np.array([[np.cos(q[6]-(pi/4)), -np.sin(q[6]-(pi/4)) * (np.cos(alpha[7])), np.sin(q[6]-(pi/4)) * (np.sin(alpha[7])),
                      a[7] * (np.cos(q[6]-(pi/4)))], [np.sin(q[6]-(pi/4)), np.cos(q[6]-(pi/4)) * (np.cos(alpha[7])),
                                                    -np.cos(q[6]-(pi/4)) * (np.sin(alpha[7])),
                                                    a[7] * (np.sin(q[6]-(pi/4)))],
                     [0, np.sin(alpha[7]), np.cos(alpha[7]), d[7]], [0, 0, 0, 1]])
    joint_1_transformation = t1_w
    joint_2_transformation = joint_1_transformation @ t2_1
    joint_3_transformation = joint_2_transformation @ t3_2
    joint_4_transformation = joint_3_transformation @ t4_3
    joint_5_transformation = joint_4_transformation @ t5_4
    joint_6_transformation = joint_5_transformation @ t6_5
    joint_7_transformation = joint_6_transformation @ t7_6
    joint_e_transformation = joint_7_transformation @ te_7

    RM_1 = joint_1_transformation[:-1, :-1]
    RM_2 = joint_2_transformation[:-1, :-1]
    RM_3 = joint_3_transformation[:-1, :-1]
    RM_4 = joint_4_transformation[:-1, :-1]
    RM_5 = joint_5_transformation[:-1, :-1]
    RM_6 = joint_6_transformation[:-1, :-1]
    RM_7 = joint_7_transformation[:-1, :-1]
    RM_e = joint_e_transformation[:-1, :-1]

    R_0 = np.eye(3)[:, 2:]
    R_1 = (RM_1[:,2:])
    # print(np.shape(R_1))
    R_2 = (RM_2[:,2:])
    R_3 = (RM_3[:,2:])
    R_4 = (RM_4[:,2:])
    R_5 = (RM_5[:,2:])
    R_6 = (RM_6[:,2:])
    R_7 = (RM_7[:,2:])
    R_8 = (RM_e[:,2:])

    TM_1 = joint_1_transformation[:-1, -1:]
    TM_2 = joint_2_transformation[:-1, -1:]
    # TM_3 = joint_3_transformation[:-1, -1:]
    TM_4 = joint_4_transformation[:-1, -1:]
    # TM_5 = joint_5_transformation[:-1, -1:]
    # TM_6 = joint_6_transformation[:-1, -1:]
    # TM_7 = joint_7_transformation[:-1, -1:]
    TM_e = joint_e_transformation[:-1, -1:]

    third_joint_position = np.array([[0], [0], [0.195], [1]])
    thirdjoint_pos = joint_3_transformation @ third_joint_position
    TM_3 = thirdjoint_pos[:-1, -1:]

    fifth_joint_position = np.array([[0], [0], [0.125], [1]])
    fifthjoint_pos = joint_5_transformation @ fifth_joint_position
    TM_5 = fifthjoint_pos[:-1, -1:]
    sixth_joint_position = np.array([[0], [0], [0.015], [1]])
    sixthjoint_pos = joint_6_transformation @ sixth_joint_position
    TM_6 = sixthjoint_pos[:-1, -1:]
    seventh_joint_position = np.array([[0], [0], [0.051], [1]])
    seventhjoint_pos = joint_7_transformation @ seventh_joint_position
    TM_7 = seventhjoint_pos[:-1, -1:]



    # print(np.shape(TM_7))
    O0 = np.array([0,0,0]).reshape(-1,1)

    # print(np.shape(O0))



    Linear_Jacobian_1 = np.cross(R_0.T, (TM_e - O0 ).T).T
    Linear_Jacobian_2 = np.cross(R_1.T, (TM_e - TM_1 ).T).T
    Linear_Jacobian_3 = np.cross(R_2.T, (TM_e - TM_2 ).T).T
    Linear_Jacobian_4 = np.cross(R_3.T, (TM_e - TM_3 ).T).T
    Linear_Jacobian_5 = np.cross(R_4.T, (TM_e - TM_4 ).T).T
    Linear_Jacobian_6 = np.cross(R_5.T, (TM_e - TM_5 ).T).T
    Linear_Jacobian_7 = np.cross(R_6.T, (TM_e - TM_6 ).T).T
    Linear_Jacobian_e = np.cross(R_7.T, (TM_e - TM_7 ).T).T

    # print(np.shape(Linear_Jacobian_7))

    Angular_Jacobian_1 = R_0
    Angular_Jacobian_2 = R_1
    Angular_Jacobian_3 = R_2
    Angular_Jacobian_4 = R_3
    Angular_Jacobian_5 = R_4
    Angular_Jacobian_6 = R_5
    Angular_Jacobian_7 = R_6
    Angular_Jacobian_e = R_7

    # print(np.shape(Angular_Jacobian_1))

    L_Jacobian_Matrix = np.hstack([ Linear_Jacobian_2, Linear_Jacobian_3, Linear_Jacobian_4, Linear_Jacobian_5
                                , Linear_Jacobian_6, -Linear_Jacobian_7, Linear_Jacobian_e])

    A_Jacobian_Matrix = np.hstack([ Angular_Jacobian_2, Angular_Jacobian_3, Angular_Jacobian_4, Angular_Jacobian_5
                                   , Angular_Jacobian_6, -Angular_Jacobian_7, Angular_Jacobian_e])

    J = np.vstack([L_Jacobian_Matrix, A_Jacobian_Matrix])




    ## STUDENT CODE GOES HERE

    return J

if __name__ == '__main__':
    q_in= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q_in),3))
