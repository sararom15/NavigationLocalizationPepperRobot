#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
This file contains the two functions needed for the computation of the obstacle positions with respect to the world frame.
This because from the laser we get the position of the obstacle wrt the robot frame.
Knowing the position of the robot in the world, and the position of the obstacles wrt robot, then we can compute the position
of the obstacles wrt the world frame.
NOTA:
the Robot frame is composed as follows:
--> the center (0,0) is the robot
--> X increasing from bottom to top
--> Y increasing from right to left
The world frame is composed as well as the robot frame (x and y axes).
"""

import numpy as np


def RotatMatrix(yam):
    """
    :param yam: rotation about z-axis of the robot frame with respect to the world frame (expressed in radians)
    :return: the rotation matrix [3x3]
    """

    # First row of the rotation matrix
    r00 = np.cos(yam)
    r01 = np.sin(yam)
    r02 = 0.0

    # Second row of the rotation matrix
    r10 = -np.sin(yam)
    r11 = np.cos(yam)
    r12 = 0.0

    # Third row of the rotation matrix
    r20 = 0.0
    r21 = 0.0
    r22 = 1.0

    # 3x3 rotation Matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def Compute_Position_Obs(x_robot_w, y_robot_w, theta_robot_w, x_obs_r, y_obs_r):
    """
    :param x_robot_w: x value of the robot position in world frame
    :param y_robot_w: y value of the robot position in world frame
    :param theta_robot_w: theta value of the robot position in world frame
    :param x_obs_r: x value of obstacle position in robot frame
    :param y_obs_r: y value of obstacle position in robot frame
    :return: Python list with the position of the obstacle with respect to the world frame
    """

    # 1: Create vector [1x3] of the position of the robot wrt world frame
    p_robot_w = np.array([[y_robot_w], [x_robot_w], [0]])

    # 2: Compute position of the obstacle wrt robot frame [1x3] vector
    p_obs_r = np.array([[y_obs_r], [x_obs_r], [0]])

    # 3: Compute the rotation matrix, calling the function 'RotatMatrix'. [3x3] matrix
    rot_mat = RotatMatrix(theta_robot_w)

    # 4: Compute the obstacle position wrt world frame [1x3] vector
    p_obs_w = [[0 for x in range(1)] for y in range(3)]

    for i in range(len(rot_mat)):
        for j in range(len(p_obs_r[0])):
            for k in range(len(p_obs_r)):
                p_obs_w[i][j] += rot_mat[i][k] * p_obs_r[k][j] # vector
    if p_robot_w[1][0] > 0:
        y_obs_w = p_obs_w[1][0] + p_robot_w[1][0]
    else:
        y_obs_w = p_obs_w[1][0] + p_robot_w[1][0]

    if p_robot_w[0][0] > 0:
        x_obs_w = p_obs_w[0][0] + p_robot_w[0][0]
    else:
        x_obs_w = p_obs_w[0][0] + p_robot_w[0][0]
    p_obs_w_ = [y_obs_w, x_obs_w, 0]
    return p_obs_w_
