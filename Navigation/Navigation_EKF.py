#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
It is the main node responsible of letting Pepper navigate in the environment, by avoiding obstacles. The robot
estimation is provided by the EKF nodes. The obstacles are sensed by laser scanner and their positions are printed
in a dynamic map.
In the main function, 4 threads are called when the node is running:
1) Thread that is responsible of detecting aruco in the environment;
2) Thread that estimates the Robot position thru EKF;
3) Thead that generate the velocity according to the input data;
4) Thread that monitors the obstacles positions and prints them into a dynamic map.

The communication is queue-based.
"""

import qi
from grid import *
from New_navig_with_obstacles import *
from EKF import *
from PoseEstimation_Pepper_Aruco import *


# ----- In case the GUI for Navigation does not work ------
"""
# Initial position
init_position = np.array([[float(0.0)], [float(0.0)], [float(0.0)]])
# Define position of the goal (x,y)
x_goal = 2.0
y_goal = 1
# Robot IP
ip = "130.251.13.188"
# ---------------------------------------------------------
"""

# initialization coefficients
gain = 0.1

# MAPPING
rows = 301
cols = 301

# GLOBAL VARIABLES
EMPTY_CELL = 0
OBSTACLE_CELL = 1
PEPPER_CELL = 2
GOAL_CELL = 3

# create discrete colormap
cmap = colors.ListedColormap(['white', 'black', 'red', 'green', ])
bounds = [EMPTY_CELL, OBSTACLE_CELL, PEPPER_CELL, GOAL_CELL, GOAL_CELL + 1, ]
norm = colors.BoundaryNorm(bounds, cmap.N)

laserValueList = [
    # RIGHT LASER
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg02/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg06/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg10/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/Y/Sensor/Value",
    # FRONT LASER
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value",
    # LEFT LASER
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg02/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg04/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg06/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg08/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg10/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg12/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/Y/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value",
    "Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/Y/Sensor/Value"
]


# function to get data from the lasers
def LaserSensor(memory_service, x_goal, y_goal, matrix_obstacles, position_robot, updatePosition):

    t = threading.currentThread()

    # initialization matrices
    data1 = np.zeros(rows * cols).reshape(rows, cols)
    data2 = np.zeros(rows * cols).reshape(rows, cols)
    data3 = np.zeros(rows * cols).reshape(rows, cols)
    data4 = np.zeros(rows * cols).reshape(rows, cols)
    data5 = np.zeros(rows * cols).reshape(rows, cols)

    useSensorValues = True
    counter = 0

    # create a plot
    # fig, ax = plt.subplots()
    # interaction turns on
    # plt.ion()

    while getattr(t, "do_run", True):
        a = time.time()

        # get Robot position estimated by EKF by Queue
        p_robot_w = position_robot.get()

        right_values = [[], []]
        front_values = [[], []]
        left_values = [[], []]

        # get data from sensors: distance obstacle-robot
        laserValue = memory_service.getListData(laserValueList)
        for i in range(0, 29, 2):
            right_values[0].append(-laserValue[i])
        for i in range(1, 30, 2):
            right_values[1].append(laserValue[i])
        for i in range(31, 60, 2):
            front_values[0].append(laserValue[i])
        for i in range(30, 59, 2):
            front_values[1].append(laserValue[i])
        for i in range(60, 90, 2):
            left_values[0].append(laserValue[i])
        for i in range(61, 90, 2):
            left_values[1].append(-laserValue[i])

        obs_front1 = [[], []]
        obs_left1 = [[], []]
        obs_right1 = [[], []]

        # Compute the position of the obstacles WITHIN 1.5 METERS around Pepper wrt the world frame
        for i in range(1, 15):
            # get only data within 3 meters
            if abs(math.sqrt(front_values[0][i] * front_values[0][i] + front_values[1][i] * front_values[1][i])) < 1.5:
                obs_world = Compute_Position_Obs(p_robot_w[0][0], p_robot_w[1][0], p_robot_w[2][0], front_values[1][i],
                                                 front_values[0][i])
                obs_front1[0].append(obs_world[0])
                obs_front1[1].append(obs_world[1])
            if abs(math.sqrt(left_values[0][i] * left_values[0][i] + left_values[1][i] * left_values[1][i])) < 1.5:
                obs_world1 = Compute_Position_Obs(p_robot_w[0][0], p_robot_w[1][0], p_robot_w[2][0], left_values[1][i],
                                                  left_values[0][i])
                obs_left1[0].append(obs_world1[0])
                obs_left1[1].append(obs_world1[1])
            if abs(math.sqrt(right_values[0][i] * right_values[0][i] + right_values[1][i] * right_values[1][i])) < 1.5:
                obs_world2 = Compute_Position_Obs(p_robot_w[0][0], p_robot_w[1][0], p_robot_w[2][0], right_values[1][i],
                                                  right_values[0][i])
                obs_right1[0].append(obs_world2[0])
                obs_right1[1].append(obs_world2[1])

        s = time.time() - a
        print "ACQUISITION TIME", s

        l = time.time()

        # map the first matrix
        if counter == 0:
            # zeros matrix data1
            data1 = np.zeros(rows * cols).reshape(rows, cols)

            # put the obstacles position in the matrix data1
            for s in range(len(obs_front1[0]) - 1):
                if not obs_front1 == []:
                    map_obstacle(data1, obs_front1[0][s], obs_front1[1][s], rows, cols)

            for a in range(len(obs_right1[0]) - 1):
                if not obs_right1 == []:
                    map_obstacle(data1, obs_right1[0][a], obs_right1[1][a], rows, cols)

            for r in range(len(obs_left1[0]) - 1):
                if not obs_left1 == []:
                    map_obstacle(data1, obs_left1[0][r], obs_left1[1][r], rows, cols)

            p = time.time() - l
            print("Time to map the matrix data1", p)
            # change the counter value
            counter = 1
            print "ok1"

        # map the second matrix
        elif counter == 1:
            data2 = np.zeros(rows * cols).reshape(rows, cols)
            for s in range(len(obs_front1[0]) - 1):
                if not obs_front1 == []:
                    map_obstacle(data2, obs_front1[0][s], obs_front1[1][s], rows, cols)

            for m in range(len(obs_right1[0]) - 1):
                if not obs_right1 == []:
                    map_obstacle(data2, obs_right1[0][m], obs_right1[1][m], rows, cols)

            for n in range(len(obs_left1[0]) - 1):
                if not obs_left1 == []:
                    map_obstacle(data2, obs_left1[0][n], obs_left1[1][n], rows, cols)

            counter = 2
            print "ok2"

        # map the third matrix
        elif counter == 2:
            data3 = np.zeros(rows * cols).reshape(rows, cols)

            for s in range(len(obs_front1[0]) - 1):
                if not obs_front1 == []:
                    map_obstacle(data3, obs_front1[0][s], obs_front1[1][s], rows, cols)

            for t in range(len(obs_right1[0])-1):
                if not obs_right1 == []:
                    map_obstacle(data3, obs_right1[0][t], obs_right1[1][t], rows, cols)

            for l in range(len(obs_left1[0])-1):
                if not obs_left1 == []:
                    map_obstacle(data3, obs_left1[0][l], obs_left1[1][l], rows, cols)

            counter = 3
            print "ok3"

        # map the fourth matrix
        elif counter == 3:
            data4 = np.zeros(rows * cols).reshape(rows, cols)
            for s in range(len(obs_front1[0]) - 1):
                if not obs_front1 == []:
                    map_obstacle(data4, obs_front1[0][s], obs_front1[1][s], rows, cols)

            for p in range(len(obs_right1[0])-1):
                if not obs_right1 == []:
                    map_obstacle(data4, obs_right1[0][p], obs_right1[1][p], rows, cols)

            for q in range(len(obs_left1[0])-1):
                if not obs_left1 == []:
                    map_obstacle(data4, obs_left1[0][q], obs_left1[1][q], rows, cols)

            counter = 4
            print "ok4"

        # map the fifth matrix
        elif counter == 4:
            data5 = np.zeros(rows * cols).reshape(rows, cols)
            for s in range(len(obs_front1[0]) - 1):
                if not obs_front1 == []:
                    map_obstacle(data5, obs_front1[0][s], obs_front1[1][s], rows, cols)

            for d in range(len(obs_right1[0]) - 1):
                if not obs_right1 == []:
                    map_obstacle(data5, obs_right1[0][d], obs_right1[1][d], rows, cols)

            for k in range(len(obs_left1[0]) - 1):
                if not obs_left1 == []:
                    map_obstacle(data5, obs_left1[0][k], obs_left1[1][k], rows, cols)

            # put data5 into a queue
            # output_q.put(data5)
            counter = 0
            print "ok5"

        s = time.time()
        # merge the matrices:
        # data_t = data1 or data2 or data3 or data4 or data5 = [((data1 or data2) or (data3 or data4)) or data5]

        data1_2 = np.logical_or(data1, data2)
        data3_4 = np.logical_or(data3, data4)
        data1_4 = np.logical_or(data1_2, data3_4)
        data_t = np.logical_or(data1_4, data5).astype(int)

        # fill the final matrix (data_t) in the queue for the thread which will be responsable of computing the velocity
        matrix_obstacles.put(data_t)
        updatePosition.put(p_robot_w)

        # map also the robot position anche goal position
        map_robot(data_t, p_robot_w[0][0], p_robot_w[1][0], rows, cols)
        map_goal(data_t, x_goal, y_goal, rows, cols)

        r = time.time() - s
        print"merge data time", r

        # show the results
        # ax.imshow(data_t, cmap=cmap, norm=norm)
        # ax.set_xticks(np.arange(0.5, rows, 1))
        # ax.set_yticks(np.arange(0.5, cols, 1))
        # plt.tick_params(axis='both', labelbottom=False, labelleft=False)
        # plt.pause(0.000000001)

        time.sleep(0.1)


def main(ip, init_pos_x,init_pos_y, init_pos_theta, x_goal, y_goal):
    ip = str(ip.get())
    # print "ip", str(ip.get())
    # print "x_goal", float(x_goal.get())
    x_goal = float(x_goal.get())
    # print "y_goal", float(y_goal.get())
    y_goal = float(y_goal.get())
    init_position = np.array([[float(init_pos_x.get())], [float(init_pos_y.get())], [float(init_pos_theta.get())]])
    # print "init position", init_position

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=ip,
                        help="Robot IP address. On robot or local Naoqi: use \
                        '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()

    # Starting application
    try:
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["laserReader", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    session = app.session

    # get camera matrix and dist coeff for the aruco detection phase
    mtx = np.load('CameraMatrix.npy')
    dist = np.load('DistCoeff.npy')
    print "matrix", mtx
    print "coeff", dist

    # Starting services
    memory_service = session.service("ALMemory")
    motion_service = session.service("ALMotion")
    motion_service.wakeUp()
    motion_service.setOrthogonalSecurityDistance(0.0005)
    motion_service.setTangentialSecurityDistance(0.0005)

    # define the queues
    matrix_obstacles = Queue.Queue()
    position_robot = Queue.Queue()
    robotPose_w = Queue.Queue()
    updatePosition = Queue.Queue()

    # Create a threads
    # --- thread for pose estimation using Aruco Markers  --- #
    ArucoThread = threading.Thread(target=PoseEstimation, args=(session, mtx, dist, robotPose_w,))
    ArucoThread.start()

    # --- EKF thread: to have and estimation of the Pepper position --- #
    EKFThread = threading.Thread(target=EKFmain, args=(session, init_position, robotPose_w, position_robot,))
    EKFThread.start()

    # --- thread to monitor the laser data --- #
    monitorThread = threading.Thread(target=LaserSensor, args=(memory_service, x_goal, y_goal, matrix_obstacles,
                                                               position_robot, updatePosition,))
    monitorThread.start()

    # --- thread to compute the velocity using APF algorithm and move Pepper --- #
    apfThread = threading.Thread(target=ReachGoal, args=(motion_service, rows, cols, gain, x_goal, y_goal,
                                                         matrix_obstacles, updatePosition, ))
    apfThread.start()

    # Program stays at this point until we stop it
    app.run()

    sys.exit(0)


if __name__ == "__main__":
    main(ip, init_position, x_goal, y_goal)









