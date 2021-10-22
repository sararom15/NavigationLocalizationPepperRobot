"""
This node allows Pepper to move in a triangular path, achieving the three goal in front the 3 markers in the world frame ('picture of the environment setup save in the folder'), and stay there for a while (20 secs).
It is received the robot position thru TCP/IP connection by the 'EKF_Socket.py'.
"""

import qi
import socket
from PoseEstimation_Pepper_Aruco import *

# For the motion control
linear_threshold = 0.3
angular_threshold = 0.05


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def ComputeErrorTheta(x_goal, y_goal, x_robot, y_robot, theta_robot):
    desired_Angulation = math.atan2(y_goal - y_robot, x_goal - x_robot)
    angle_to_move = desired_Angulation - theta_robot
    angle_to_move = normalize_angle(angle_to_move)
    return angle_to_move


def ComputeAttractiveVelocity(x_goal, y_goal, x_robot, y_robot, theta_robot):
    distance_goal = math.sqrt((x_robot - x_goal) * (x_robot - x_goal) + (y_robot - y_goal) * (y_robot - y_goal))
    velocity_vector = np.array([[(x_goal - x_robot)/distance_goal], [(y_goal - y_robot)/distance_goal]])
    matrix = np.array([[math.cos(theta_robot), math.sin(theta_robot)],
                  [-math.sin(theta_robot), math.cos(theta_robot)]])
    vel_att = np.dot(matrix, velocity_vector)

    return vel_att, distance_goal


def MovePepper(motion_service):
    # CLIENT DEFINITION
    s = socket.socket()
    host = "localhost"
    port = 3030
    s.bind((host, port))
    s.listen(5)
    new_soc, addr = s.accept()

    # Initialization variables
    # for initialization velocities
    counter = 0
    ID_15 = True  # if set to True, the robot reaches the marker ID:15
    ID_13 = False  # if set to True, the robot reaches the marker ID:13
    ID_17 = False  # if set to True, the robot reaches the marker ID:17

    # variables for counting the time when the robot achieved the goal: initialized at 0
    t = 0
    a = 0
    b = 0

    # --- LOOP --- ]
    while True:

        # read information about the robot position
        data = new_soc.recv(1012)
        arr_strs = (data.split(","))
        position = []
        for i in range(len(arr_strs)):
            position.append(float(arr_strs[i]))

        if ID_15:
            # --------- reach the marker ID: 15 and stay there for a while -------- #
            x_goal = markers[0].x - 1
            y_goal = markers[0].y
            theta_goal = markers[0].theta

            # INITIALIZATION VELOCITY DEFINED ACCORDING TO THE INITIAL POSITION OF THE ROBOT
            if counter == 0:
                print "goal position", x_goal, y_goal, theta_goal
                counter = counter + 1
                # ----- linear velocity ------
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1],
                                                                   position[2])
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ----- angular velocity  ------
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = -1

                print "====================================================================="
                print "Forward Movement Started"

            # --- start the motion --- #
            if abs(distance_goal) > linear_threshold:
                print "Go close the marker 15"
                if abs(error_theta) > angular_threshold:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.1 * vel_ang_theta)
                else:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.0)

                # ---- update linear velocity -----
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1], position[2])
                print "distance goal", distance_goal
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ---- update angular velocity -----
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = - 1
            else:
                print "marker 15 has been achieved, stay here for a while"
                print "====================================================================="

                # stay there for a while (20 secs)
                if t < 20:
                    motion_service.moveToward(0, 0, 0)
                    t = t+1
                    print t
                # switch to True the MARKER ID I wanna visit right now, and switch to False the marker 15, because
                # it has already visited
                elif t == 20:
                    ID_13 = True
                    ID_15 = False
                # set the counter to  0 to initialize the new mission
                    counter = 0
                    t = t+1
                    print "Go to marker 13"
                    print "====================================================================="

        if ID_13:
            # --------- reach the marker ID: 13 and stay there for a while -------- #
            x_goal = markers[2].x - 0.2
            y_goal = markers[2].y + 1
            theta_goal = markers[2].theta

            # INITIALIZATION VELOCITY DEFINED ACCORDING TO THE INITIAL POSITION OF THE ROBOT
            if counter == 0:
                counter = counter + 1
                print "goal position ID 13", x_goal, y_goal, theta_goal
                # ----- linear velocity ------
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1],
                                                                   position[2])
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ----- angular velocity  ------
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = -1

                print "====================================================================="
                print "Forward Movement Started"

            # --- start the motion --- #
            if abs(distance_goal) > linear_threshold:
                print "Going close to the marker 13"
                if abs(error_theta) > angular_threshold:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.1 * vel_ang_theta)
                else:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.0)

                # ---- update linear velocity -----
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1], position[2])
                print "distance goal", distance_goal
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ---- update angular velocity -----
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = - 1
            else:
                print "marker 13 has been achieved, stay here for a while"
                print "====================================================================="
                # --- stay there for a while --- #
                if a < 20:
                    motion_service.moveToward(0, 0, 0)
                    a = a+1
                    print a
                elif a == 20:
                    motion_service.moveToward(0, 0, 0)
                    ID_13 = False
                    ID_17 = True
                    # set the counter to  0 to initialize the new mission
                    counter = 0
                    print "Go to marker 17"
                    print "====================================================================="

        # --------- reach the marker ID: 17 and stay there for a while -------- #
        if ID_17:
            x_goal = markers[1].x + 0.60
            y_goal = markers[1].y + 1
            theta_goal = markers[1].theta

            # INITIALIZATION VELOCITY DEFINED ACCORDING TO THE INITIAL POSITION OF THE ROBOT
            if counter == 0:
                counter = counter + 1
                print "goal position ID 17", x_goal, y_goal, theta_goal
                # ----- linear velocity ------
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1],
                                                                   position[2])
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ----- angular velocity  ------
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = -1

                print "====================================================================="
                print "Forward Movement Started"

            # --- start the motion --- #
            if abs(distance_goal) > linear_threshold:
                print "Going close to the marker 17"
                if abs(error_theta) > angular_threshold:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.1 * vel_ang_theta)
                else:
                    motion_service.moveToward(0.3 * vel_lin_x, 0.3 * vel_lin_y, 0.0)

                # ---- update linear velocity -----
                vel_att, distance_goal = ComputeAttractiveVelocity(x_goal, y_goal, position[0], position[1], position[2])
                print "distance goal", distance_goal
                vel_lin_x = vel_att[0][0]
                vel_lin_y = vel_att[1][0]
                # normalize the total velocity
                norm = math.sqrt(vel_lin_x * vel_lin_x + vel_lin_y * vel_lin_y)
                vel_lin_x = float(vel_lin_x) / norm
                vel_lin_y = float(vel_lin_y) / norm

                # ---- update angular velocity -----
                error_theta = theta_goal - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = - 1
            else:
                print "marker 17 has been achieved, stay here for a while"
                print "====================================================================="

                if b < 20:
                    motion_service.moveToward(0, 0, 0)
                    b = b+1
                    print b

                elif b == 20:
                    motion_service.moveToward(0, 0, 0)
                    ID_17 = False
                # set the counter to  0 to initialize the new mission
                    counter = 0
                    # print "Go again to the marker 15"
                    # print "====================================================================="

        # --- one round has been performed: re-start the loop ---
        if ID_13 == False and ID_15 == False and ID_17 == False:

            print "Go back to the initial position"
            print "====================================================================="

            # ----- angular velocity  ------
            error_theta = 0.0 - position[2]
            print "delta theta", error_theta
            if error_theta > 0:
                vel_ang_theta = 1
            else:
                vel_ang_theta = -1

            # Alignment of the robot with the initial orientation (0.0 degree)
            if abs(error_theta) > 0.3:
                motion_service.moveToward(0.0, 0.0, 0.1 * vel_ang_theta)
                error_theta = 0.0 - position[2]
                print "delta theta", error_theta
                if error_theta > 0:
                    vel_ang_theta = 1
                else:
                    vel_ang_theta = -1

            # stop it when the robot is aligned
            else:
                motion_service.moveToward(0, 0, 0)
                t = 0
                a = 0
                b = 0
                ID_15 = True

            # time.sleep(4)
            print "Forward Movement Complete and start again"
            print "====================================================================="

        # else:
        #    motion_service.moveToward(0.0, 0.0, 0.0)
        #    print "Forward Movement Complete"
        #    print "====================================================================="
    new_soc.close()
    time.sleep(3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="130.251.13.188",
                        help="Robot IP address. On robot or local Naoqi: use \
                        '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
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
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    # motion_service.wakeUp()
    posture_service.goToPosture("StandInit", 0.5)

    MovePepper(motion_service)


if __name__ == "__main__":
    main()

