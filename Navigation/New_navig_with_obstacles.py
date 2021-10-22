"""
Node responsable of move Pepper in the environemnt by avoiding obstacles.
"""

import math
import os
from ComputeWorldPositionOfObstacles import *

my_path = os.path.abspath(os.getcwd())


def ComputeRealPositionsFromMap(data, rows, cols):
    """
    :param data: matrix with the position of the obstacles, robot and goal
    :param rows: numb of rows of the matrix
    :param cols: num of columns of the matrix
    :return goal: real goal position on the world frame [m]
    :return obstacle: real obstacles position on the world frame [m]

    Description: The function is able to compute the position of the goal and the obstacles in the world frame [m].
    (i,j) of the matrix represent the position of them in the real world by taking into account the resolution:
    As stated in "grid.py" file, 1 cell on the matrix is 5 cm in the reality.
    In addition, the zero is in the center of the map.
    """

    position = [[], []]
    obstacle = [[], []]

    for i in range(rows):
        for j in range(cols):
            # data[i,j] == 2 = ROBOT POSITION
            if data[i, j] == 2:
                position[0].append(-float(i)/20 + float(rows-1)/40)
                position[1].append(-float(j)/20 + float(cols-1)/40)
            # data[i,j] == 1 = OBSTACLE POSITION
            elif data[i, j] == 1:
                obstacle[0].append(-float(i)/20 + float(rows-1)/40)
                obstacle[1].append(-float(j)/20 + float(cols-1)/40)
    return obstacle, position


def ComputeAttractiveVelocity(x_goal, y_goal, x_robot, y_robot, theta_robot):
    distance_goal = math.sqrt((x_robot - x_goal) * (x_robot - x_goal) + (y_robot - y_goal) * (y_robot - y_goal))
    velocity_vector = np.array([[(x_goal - x_robot)/distance_goal], [(y_goal - y_robot)/distance_goal]])
    matrix = np.array([[math.cos(theta_robot), math.sin(theta_robot)],
                  [-math.sin(theta_robot), math.cos(theta_robot)]])
    vel_att = np.dot(matrix, velocity_vector)
    return vel_att, distance_goal


def ReachGoal(motion_service, rows, cols, gain, x_goal, y_goal, output_q, updatePosition):

    """
    Artificial potential field application.
    In addition, when the robot get in local minimum, a tangential velocity is applied.
    """

    # get data from the queue
    data = output_q.get()
    Position = updatePosition.get()

    print "angle", Position[2][0]

    # get position of the goal, obstacle and robot wrt world frame
    # OBSTACLE AND GOAL (to modify yet)
    obstacle, p_robot_w_ = ComputeRealPositionsFromMap(data, rows, cols)

    # p_robot_w_ = np.array([[position[0]], [position[1]]])

    # VELOCITY COMPUTED TAKING INTO ACCOUNT ALL THE OBSTACLE WITHIN 1.5 METERS
    # 1) Compute repulsive component
    # Compute the distance between the obstacles and the current position of the robot
    dr = [0 for i in obstacle[0]]
    vel_rep_x = 0
    vel_rep_y = 0

    for i in range(len(obstacle[0])):
        dr[i] = math.sqrt(((-obstacle[0][i] + p_robot_w_[0][0]) * (-obstacle[0][i] + p_robot_w_[0][0])) +
                          ((-obstacle[1][i] + p_robot_w_[1][0]) * (-obstacle[1][i] + p_robot_w_[1][0])))

        if dr[i] < 1.5:
            vel_rep_x += (- obstacle[0][i] + p_robot_w_[0][0])/(dr[i] * dr[i])
            vel_rep_y += (- obstacle[1][i] + p_robot_w_[1][0])/(dr[i] * dr[i])

    repulsiveVelocity_vector = np.array([[vel_rep_x], [vel_rep_y]])
    print "REPUVEL", repulsiveVelocity_vector

    matrix = np.array([[math.cos(Position[2][0]), math.sin(Position[2][0])],
                        [-math.sin(Position[2][0]), math.cos(Position[2][0])]])

    vel_rep = np.dot(matrix, repulsiveVelocity_vector)

    t_x = vel_rep[0][0]
    t_y = -vel_rep[1][0]

    # 2) Compute attractive component
    # Compute the distance between the goal and the current position of the robot
    vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, p_robot_w_[0][0], p_robot_w_[1][0], Position[2][0])

    # 3) Compute the velocity taking into account the 2 components
    vel_lin_x = vel_att[0][0] + gain * vel_rep[0][0] + gain * t_x
    vel_lin_y = vel_att[1][0] + gain * vel_rep[1][0] + gain * t_y

    vel_ang_theta = 0.0

    # normalize the total velocity
    norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y + vel_ang_theta * vel_ang_theta)
    vel_lin_x = float(vel_lin_x) / norm
    vel_lin_y = float(vel_lin_y) / norm
    vel_ang_theta = float(vel_ang_theta) / norm

    print "====================================================================="
    print "Forward Movement Started"

    threshold = 0.3
    # the robot will be stopped when the distance to the goal is less than 0.3 meters

    obstacle_x = []
    obstacle_y = []

    while abs(distance_goal) > threshold:
        # get always data from the queue
        data = output_q.get()
        # set the velocity
        motion_service.moveToward(0.3*vel_lin_x, 0.3*vel_lin_y, vel_ang_theta)
        # update the position of the goal, the obstacle and the robot
        obstacle, p_robot_w_ = ComputeRealPositionsFromMap(data, rows, cols)
        # p_robot_w_ = np.array([[position[0]], [position[1]]])
        # update the distances between the position and the obstacles
        dr = [0 for i in obstacle[0]]
        vel_rep_x = 0
        vel_rep_y = 0
        for i in range(len(obstacle[0])):
            dr[i] = math.sqrt(((-obstacle[0][i] + p_robot_w_[0][0]) * (-obstacle[0][i] + p_robot_w_[0][0])) + ((-obstacle[1][i] + p_robot_w_[1][0]) * (-obstacle[1][i] + p_robot_w_[1][0])))

            if dr[i] < 1.5:
                # print "distance", dr
                obstacle_x.append(obstacle[0])
                obstacle_y.append(obstacle[1])
                vel_rep_x += (- obstacle[0][i] + p_robot_w_[0][0]) / (dr[i] * dr[i])
                vel_rep_y += (- obstacle[1][i] + p_robot_w_[1][0]) / (dr[i] * dr[i])

        repulsiveVelocity_vector = np.array([[vel_rep_x], [vel_rep_y]])
        matrix = np.array([[math.cos(Position[2][0]), math.sin(Position[2][0])],
                           [-math.sin(Position[2][0]), math.cos(Position[2][0])]])
        vel_rep = np.dot(matrix, repulsiveVelocity_vector)

        t_x = vel_rep[0][0]
        t_y = -vel_rep[1][0]

        """
        # normalization repulsive component
        intensity_rep = math.sqrt(vel_rep_x * vel_rep_x + vel_rep_y * vel_rep_y)
        vel_rep_x /= intensity_rep
        vel_rep_y /= intensity_rep
        """
        # 2) Compute attractive component
        # Compute the distance between the goal and the current position of the robot
        vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, p_robot_w_[0][0], p_robot_w_[1][0],
                                                           Position[2][0])
        # 3) Compute the velocity taking into account the 2 components
        vel_lin_x = vel_att[0][0] + gain * vel_rep[0][0] + gain * t_x
        vel_lin_y = vel_att[1][0] + gain * vel_rep[1][0] + gain * t_y

        vel_ang_theta = 0.0

        print "rep_x, attr_x, tot_x", vel_rep[0][0], vel_att[0][0], vel_lin_x
        print "rep_y, attr_y, tot_y", vel_rep[1][0], vel_att[1][0], vel_lin_y

        # normalize
        norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y + vel_ang_theta * vel_ang_theta)
        vel_lin_x = float(vel_lin_x) / norm
        vel_lin_y = float(vel_lin_y) / norm
        vel_ang_theta = float(vel_ang_theta) / norm

    np.savetxt(my_path + "/csv_files/obstacle_position.csv", np.column_stack((obstacle_x, obstacle_y)), delimiter="|",
               fmt='%s')
    motion_service.moveToward(0, 0, 0)
    print "Forward Movement Complete"
    print "====================================================================="

