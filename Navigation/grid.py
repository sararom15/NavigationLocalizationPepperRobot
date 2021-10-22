import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np


def map_obstacle(data, x_obs, y_obs, rows, cols):

    """
    A numpy array is typically referenced by array[row, col], and the origin (0,0) is the TOP LEFT of
    the matrix. Additionally this means the array is indexed by the [y, x] value, not [x, y].
    Since in the world frame the initial position is (0,0), we want that it is posed on the center of the grid.
    We need to do a re-mapping in the grid-map.

    :param x_obs: x_obs position in the world frame [meters]
    :param y_obs: y_obs position in the world frame [meters]
    :param rows: num of cells for rows ( NUM OF ROWS OF THE MATRIX)
    :param cols: num of cells for columns (NUM OF COLS OF THE MATRIX)
    :return: the newest matrix with the position of the obstacle.

    ------> %%%%%%%% RESOLUTION %%%%%%%% <------
    NOTA: since each values from the sensor is multiplied for 10, then
    EACH CELL IN THE MAP IS 10 CM IN REALITY.

    NOTA 2: since each values from the sensor is multiplied for 20, then
    EACH CELL IN THE MAP IS 5 CM IN REALITY.

    """

    # OBSTACLES POSITION
    data[int((rows-1)/2 - np.array(round(x_obs, 1) * 20)), int(-np.array(round(y_obs, 1) * 20) + (cols - 1)/2)] = OBSTACLE_CELL


def map_robot(data, x_robot, y_robot, rows, cols):
    """
    As the function before
    :param x_robot: x_robot position in the world frame [meters]
    :param y_robot: y_robot position in the world frame [meters]
    :param rows: num of cells for rows ( NUM OF ROWS OF THE MATRIX)
    :param cols: num of cells for columns (NUM OF COLS OF THE MATRIX)
    :return: the newest matrix with the position of the robot.

    """
    # ROBOT POSITION
    for i in range(2):
        data[int(-(round(x_robot, 1) * 20) + (rows - 1)/2), int(-(round(y_robot, 1) * 20) + (cols-1)/2)] = PEPPER_CELL
        data[int(-(round(x_robot, 1) * 20 + i) + (rows - 1)/2), int(-(round(y_robot, 1) * 20) + (cols-1)/2)] = PEPPER_CELL
        data[int(-(round(x_robot, 1) * 20) + (rows - 1)/2), int(-(round(y_robot, 1) * 20 + i) + (cols-1)/2)] = PEPPER_CELL
        data[int(-(round(x_robot, 1) * 20 + i) + (rows - 1)/2), int(-(round(y_robot, 1) * 20 + i) + (cols-1)/2)] = PEPPER_CELL


def map_goal(data, x_goal, y_goal, rows, cols):
    """
    As the function before.
    :param x_goal: x_robot position in the world frame [meters]
    :param y_goal: y_robot position in the world frame [meters]
    :param rows: num of cells for rows ( NUM OF ROWS OF THE MATRIX)
    :param cols: num of cells for columns (NUM OF COLS OF THE MATRIX)
    :return: the newest matrix with the position of the goal.
    """
    # GOAL POSITION
    for i in range(2):
        data[int(-(round(x_goal, 1) * 20) + (rows - 1) / 2), int(-(round(y_goal, 1) * 20) + (cols - 1) / 2)] = GOAL_CELL
        data[int(-(round(x_goal, 1) * 20 + i) + (rows - 1) / 2), int(-(round(y_goal, 1) * 20) + (cols - 1) / 2)] = GOAL_CELL
        data[int(-(round(x_goal, 1) * 20) + (rows - 1) / 2), int(-(round(y_goal, 1) * 20 + i) + (cols - 1) / 2)] = GOAL_CELL
        data[int(-(round(x_goal, 1) * 20 + i) + (rows - 1) / 2), int(-(round(y_goal, 1) * 20 + i) + (cols - 1) / 2)] = GOAL_CELL


# GLOBAL VARIABLES
EMPTY_CELL = 0
OBSTACLE_CELL = 1
PEPPER_CELL = 2
GOAL_CELL = 3

# create discrete colormap
cmap = colors.ListedColormap(['white', 'black', 'red', 'green', ])
bounds = [EMPTY_CELL, OBSTACLE_CELL, PEPPER_CELL, GOAL_CELL, GOAL_CELL + 1]
norm = colors.BoundaryNorm(bounds, cmap.N)

"""
def printMap(rows, cols, x_goal, y_goal, position_robot, MapforPrinting):

    # create a plot
    fig, ax = plt.subplots()
    # interaction turns on
    plt.ion()

    #get the matrix 301x301 with the position of the obstacles
    data_t = MapforPrinting.get()

    #get the actual position of the robot
    robot_p = position_robot.get()

    #print the position of the robot on the matrix
    map_robot(data_t, robot_p[0][0], robot_p[1][0], rows, cols)

    #print the position of the goal on the matrix
    map_goal(data_t, x_goal, y_goal, rows, cols)

    # show the results
    ax.imshow(data_t, cmap=cmap, norm=norm)
    ax.set_xticks(np.arange(0.5, rows, 1))
    ax.set_yticks(np.arange(0.5, cols, 1))
    plt.tick_params(axis='both', labelbottom=False, labelleft=False)
    plt.pause(0.001)
"""

