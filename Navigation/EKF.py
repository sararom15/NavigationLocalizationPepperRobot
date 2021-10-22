"""
The code performs the localization of the robot with EKF and ArUCO marker.
What we have:
1) velocity of the robot as control input (u,v,w);
2) position of the robot when it detects a marker in the environment (provided by the 'PoseEstimation_Pepper_Aruco.py' thru a queue communication);

The information about the robot position are then used by the Navigation node and are shared by queue-based communication.
"""

import qi
import os
import matplotlib.pyplot as plt
from PoseEstimation_Pepper_Aruco import *


my_path = os.path.abspath(os.getcwd())


# -- GLOBAL VARIABLES FOR PLOTTING
error_x = []
error_y = []
error_theta = []
x_measurement = []
y_measurement = []
x_value = []
y_value = []
meas_x = []
meas_y = []
meas_z = []
time_ = []


# Suppress scientific notation when printing NumPy arrays
np.set_printoptions(precision=3, suppress=True)

# INITIALIZATION
# -- Q matrix initialization: state model noise covariance matrix -> (3x3) square matrix -> 3 states
#     Q represents how much the actual motion deviates from your assumed state space model.
Q_t = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.05]])

# first trial it was set to 1: too big since the state estimated was completely equal to the measurement sensor.
# now it works quite well with this value.

# -- R matrix initialization: sensor measurement noise covariance matrix -> (3x3 square matrix -> same as the sensor
# measurement matrix H): sensor measurement noise covariance
R_t = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.4]])


def calc_input(session):
    """
    3 control input:
    [u,v,w] -> linear velocities along x and y axes of the robot frame + angular velocity about z axis of the robot
    """
    motion_service = session.service("ALMotion")
    # get a numpy array of the control input
    U = np.c_[motion_service.getRobotVelocity()]
    return U


def motion_model(x_k_minus_1, u, deltat):
    """
    compute the state model -> non linear
    three states, and 3 control input
    x_k = x_(k-1) + u * cos(theta) * deltat - v * sin(theta) * deltat
    y_k = y_(k-1) + u * sin(theta) * deltat + v * cos(theta) * deltat
    theta_k = theta(k-1) + w * deltat
    """
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[math.cos(x_k_minus_1[2]) * deltat, -math.sin(x_k_minus_1[2]) * deltat, 0],
                  [math.sin(x_k_minus_1[2]) * deltat, math.cos(x_k_minus_1[2]) * deltat, 0],
                  [0, 0, deltat]])

    # the predicted next state
    xPred = np.dot(F, x_k_minus_1) + np.dot(B, u)

    return xPred


def observation_model(x):
    """
    the predicted measurement from the sensor is equal to the predicted state since the output from the camera is just the position of the robot in the world frame
    -> H is an IDENTITY MATRIX (3X3) because we have three states
    """
    H = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    zPred = np.dot(H, x)
    return zPred


def Jacobian_f(x, u, deltat):
    """
    COMPUTE JACOBIAN OF THE STATE MODEL
    it is a 3x3 matrix such that:
    Jacobian_f = [[dx/dx, dx/dy, dx/dtheta], [dy/d, dy/dv, dy/dw], [dtheta/du, dtheta/dv, dtheta/dw]]
    where
    dx/dtheta = -u * sin(theta) * dt + v * cos(theta) * dt
    dy/dtheta = u * cos(theta) * dt - v * sin(theta) * dt

    """
    dx_dtheta = float((-u[0] * math.sin(x[2]) * deltat) - (u[1] * math.cos(x[2]) * deltat))
    dy_dtheta = float((u[0] * math.cos(x[2]) * deltat) - (u[1] * math.sin(x[2]) * deltat))
    Jacobian_f = np.array([[1, 0, dx_dtheta],
                            [0, 1, dy_dtheta],
                            [0, 0, 1]])
    return Jacobian_f


def Jacobian_H():
    """
    as already stated in the observation model, the sensor otoput is the position fo the robto in the world frame, then
    Jacobian_H = 3x3 identity matrix
    """
    Jacobian_H = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    return Jacobian_H


def EKF(session, realMeasurement, x, p, deltat, position_robot,):

    # -- PREDICT -- #
    # state estimate (X_t) at time t based on the state estimate at time t-1 and the control input applied at time t-1

    # get the control input at time t-1
    u = calc_input(session)
    print "Control Input", u

    # get the state space model : PREDICTED STATE
    x_Pred = motion_model(x, u, deltat)

    # Compute the jacobian of the state space model
    J_f = Jacobian_f(x, u, deltat)
    # print "Jacobian f", J_f

    # Predict the covariance of the state
    p_Pred = np.dot(J_f, np.dot(p, J_f.T)) + Q_t
    # print "Covariance State before EKF: ", p

    print "State estimate before EKF (Odometry): ", x_Pred

    # -- UPDATE -- #
    """
    NOTA: we have data measured by the camera sensor only if a marker is detected. 
    """

    # predict the observation of the sensor based on the latter estimated state
    zPred = observation_model(x)
    print "predicted observation", zPred

    # take real data from the camera sensor
    z = realMeasurement.get()
    print "Real Observation", z

    # Compute the Jacobian of the observation model
    J_h = Jacobian_H()

    # Calculate the innovation covariance
    S_t = np.dot(J_h, np.dot(p_Pred, J_h.T)) + R_t
    print "S", S_t

    # Calculate Kalman gain
    K_t = np.dot(p_Pred, np.dot(J_h, np.linalg.pinv(S_t)))
    # print "K", K_t

    # OUTLINE
    # design a circle which defines the outline for the measurement sensor, with:
    # [center = state estimated at time k]
    # [radius = 0.5]
    # if the sensor measurement is inside the outline -> it will take into consideration for computing the innovation,
    # otherwise no
    center_x = zPred[0]
    center_y = zPred[1]
    radius = 0.3
    distance = math.sqrt((center_x - z[0]) ** 2 + (center_y - z[1]) ** 2)

    # Compute the innovation
    # if there is no detection of the marker -> don't have the sensor measurement, then the innovation is 0
    if all([v == 0 for v in z]) or distance <= radius:
        print "NO MEASUREMENT!!!"
        inn_t = np.array([[0], [0], [0]])
    # if the maker has been detected -> compute innovation such that it is = realMeasurement - estimated one
    else:
        inn_t = z - zPred

    # Compute the update state estimate at time t
    x = x_Pred + np.dot(K_t, inn_t)

    # Update the state covariance at time t
    p = p_Pred - (np.dot(np.dot(K_t, J_h), p_Pred))

    print "Covariance P", p
    print "state estimate after EKF: ", x
    position_robot.put(x)
    return x, p, z


def savedata(x, t, z):
    error_x.append(x[0])
    error_y.append(x[1])
    error_theta.append(x[2])
    meas_x.append(z[0])
    meas_y.append(z[1])
    meas_z.append(z[2])
    time_.append(t)
    np.savetxt(my_path + "/csv_files/data_positions.csv", np.column_stack((error_x, error_y, error_theta, meas_x, meas_y,
                                                                           meas_z, time_)), delimiter=",", fmt='%s')

    x_value.append(x[0])
    y_value.append(x[1])
    x_measurement.append(z[0])
    y_measurement.append(z[1])
    np.savetxt(my_path + "/csv_files/plotdata.csv", np.column_stack((x_value, y_value, x_measurement, y_measurement)),
               delimiter=",", fmt='%s')


def plot_error_measurement(x, t, z):
    """
    Plot in real-time the state estimate (x,y,theta) and the sensor measurements (x,y,theta) in the world frame,
    over the time.
    """
    error_x.append(x[0])
    error_y.append(x[1])
    error_theta.append(x[2])
    meas_x.append(z[0])
    meas_y.append(z[1])
    meas_z.append(z[2])
    time_.append(t)

    # First subplot: X values
    plt.subplot(3, 1, 1)
    plt.plot(time_, meas_x, marker='*', color='darkmagenta',linestyle='', markersize='2.5')
    plt.plot(time_, error_x, color='orangered', linewidth='0.8')
    plt.xlabel("time [s]")
    plt.ylabel("position X [m]")
    plt.pause(0.05)

    # Second subplot: Y values
    plt.subplot(3, 1, 2)

    plt.plot(time_, meas_y, marker='*', color='darkmagenta', linestyle='', markersize='2.5')
    plt.plot(time_, error_y, color='chocolate', linewidth='0.8')

    plt.ylabel("position Y [m]")
    plt.pause(0.05)

    # Third subplot: theta values
    plt.subplot(3, 1, 3)

    plt.plot(time_, meas_z, marker='*', color='darkmagenta', linestyle='', markersize='2.5')
    plt.plot(time_, error_theta, color='darkred', linewidth='0.8')

    plt.xlabel("time [s]")
    plt.ylabel("position theta [rad]")
    plt.pause(0.05)

    # --- SINGLE PLOT: ERROR ALONG X, OR Y, OR THETA ---
    plt.axhline(y=0, color='r', linestyle='-', linewidth=1)
    plt.plot(time_, error_x, color='b')
    # plt.axhline(y=1.27, color='r', linestyle='-')
    # plt.plot(time_, x_measurement, color='g')
    plt.plot(time_, error_y, color='purple',linewidth=1)
    plt.plot(time_, error_theta, color='g',linewidth=1)

    plt.xlabel("time [s]")
    plt.ylabel("error")

    plt.pause(0.05)
    plt.gca().legend(('y=0','x [m]', 'y [m]', 'theta [rad]'))


def plot_path(x,z):
    """
    plot in real-time the path and the position measurement in X-Y plane
    """

    x_value.append(x[0])
    y_value.append(x[1])
    x_measurement.append(z[0])
    y_measurement.append(z[1])

    # plot
    plt.plot(y_value, x_value, color='g', linewidth=1)
    plt.scatter(y_measurement, x_measurement,s=3, color='purple')
    plt.xlabel("y")
    plt.ylabel("x")
    plt.xlim(-2, 4)
    plt.ylim(-2, 4)
    plt.axis('equal')
    plt.pause(0.05)
    plt.gca().legend(('State Estimation', 'Sensor Measurement'))


def EKFmain(session, init_position, realMeasurement, position_robot):
    # -- INITIALIZATION -- #
    deltat = 1
    # -- Define the initial position of the x status
    # x_t_minus_1 = np.array([[1.8], [0.0], [0.0]])
    x_t_minus_1 = init_position

    p_t_minus_1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    _time = 0.0

    while True:
        _time += deltat
        # -- loop -- #
        x, p, z = EKF(session, realMeasurement, x_t_minus_1, p_t_minus_1, deltat, position_robot,)
        savedata(x, _time, z)
        # plot_error_measurement(x, _time, z)
        # plot_path(x, z)
        x_t_minus_1 = x
        p_t_minus_1 = p
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="130.251.13.123",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()

    # get camera matrix and dist coeff for the aruco detection phase
    mtx = np.load('CameraMatrix.npy')
    dist = np.load('DistCoeff.npy')
    print "matrix", mtx
    print "coeff", dist

    # initialization of the queues
    robotPose_w = Queue.Queue()
    position_robot = Queue.Queue()

    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        motion_service = session.service("ALMotion")
        posture_service = session.service("ALRobotPosture")
        # motion_service.wakeUp()
        posture_service.goToPosture("StandInit", 0.5)

        # create a threads for check aruco and for estimate the state of the robot

        # -- detection aruco thread -- #
        ArucoThread = threading.Thread(target=PoseEstimation, args=(session, mtx, dist, robotPose_w,))
        ArucoThread.start()

        # -- EKF thread -- #
        EKFThread = threading.Thread(target=EKFmain, args=(session, robotPose_w, position_robot,))
        EKFThread.start()

    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

