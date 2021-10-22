"""
Second GUI: it is responsible of the calibration: it allows to capture pictures for 6 times (10 pictures each time changing orientation of the chessboard).

The path for the directory where the images for calibration are saved is : 'C:/Users/sarar/OneDrive/Documenti/calibration_images/'
need to be change in case of downloading.

"""

import Tkinter
from Tkinter import *
import qi
import argparse
import sys
import time
import numpy as np
import cv2
import vision_definitions
import os
from functools import partial
import shutil
import glob

my_path = os.path.abspath(os.getcwd())


def CalibrationCamera():
    """
    function for calibrating the camera
    """
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (6, 9)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None

    # Extracting path of individual image stored in a given directory
    # images = glob.glob('./images2/*.jpg')
    images = glob.glob(my_path + '/calibration_images/*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                                 flags=cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                       cv2.CALIB_CB_FILTER_QUADS)

        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """

        if ret:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        cv2.imshow('img', img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    h, w = img.shape[:2]

    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    # -- Save the values into a files
    np.save('CameraMatrix', mtx)
    np.save('DistCoeff', dist)
    return mtx, dist


def TakePicture(session, i, cameraNumber):
    video_service = session.service("ALVideoDevice")
    motion_service = session.service("ALMotion")
    life_service = session.service("ALAutonomousLife")
    life_service.setAutonomousAbilityEnabled("BasicAwareness", False)
    motion_service.wakeUp()

    # cameraID = 0 # Top Camera
    # cameraID = 1  # Bottom Camera
    # cameraID = 2 # Depth Camera
    # cameraID = 3 # Stereo Camera
    cameraID = cameraNumber

    resolution = vision_definitions.kQVGA  # 320 * 240
    colorSpace = vision_definitions.kBGRColorSpace
    videoClient = video_service.subscribe("python_client", resolution, colorSpace, 5)
    print "Client name: ", videoClient
    video_service.setParam(vision_definitions.kCameraSelectID, cameraID)
    t0 = time.time()
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    naoImage = video_service.getImageRemote(videoClient)
    t1 = time.time()
    # Time the image transfer.
    print "acquisition delay ", t1 - t0
    # Now we work with the image returned and save it as a PNG  using ImageDraw
    # package.
    # Get the image size and pixel array.
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    img_str = str(bytearray(array))
    video_service.releaseImage(videoClient)
    nparr = np.fromstring(img_str, np.uint8)
    img_np = nparr.reshape((imageHeight, imageWidth, 3)).astype(np.uint8)
    cv2.imshow("pepper camera", img_np)
    path = my_path + '/temp'
    status = cv2.imwrite(os.path.join(path, 'image_%d.jpg' % (i)), img_np)
    print "ok?", status
    key = cv2.waitKey(1)
    video_service.unsubscribe(videoClient)


def Capture(session, i, entryCamera):
    print "prova", entryCamera.get()
    if int(entryCamera.get()) == 0:
        cameraNumber = 0
    else:
        cameraNumber = 1
    print "cameraNumber", cameraNumber
    for j in range(0, 10):
        print "i", i
        print "j", j
        TakePicture(session, i, cameraNumber)
        i = i + 1


def SavePictures():
    img_folder_path = my_path + '/calibration_images/'
    dirListing = os.listdir(img_folder_path)
    print "num of elements", (len(dirListing))

    for i in range(0, 10):
        path1 = my_path + '/image_%d.jpg' % (i)
        path2 = my_path + '/calibration_images/image_%d.jpg' % (i + len(dirListing))
        shutil.copyfile(path1, path2)


def Window2(win2, session):
    i = 0
    win2.title('Camera Calibration')
    win2.geometry("1000x700+10+20")

    titlelabel = Tkinter.Label(win2, text="Take pictures for Calibration using Pepper Camera", fg='red',
                               font=("Helvetica", 16))
    titlelabel.place(x=20, y=20)

    Camera = Tkinter.Label(win2, text='Choose Camera', fg='black', font=("Helvetica", 12))
    Camera.place(x=20, y=50)

    TypeCamera0 = Tkinter.Label(win2, text='Type -> 0 <- to calibrate the TOP CAMERA')
    TypeCamera0.place(x=20, y=80)

    TypeCamera1 = Tkinter.Label(win2, text='Type -> 1 <- to calibrate the BOTTOM CAMERA')
    TypeCamera1.place(x=20, y=110)

    """
    choices = ['Top Camera', 'Bottom Camera']
    camera = StringVar(win2)
    camera.set('Top Camera')
    CameraChoice = OptionMenu(win2, camera, *choices)
    CameraChoice.place(x =250, y=480)
    camera.trace("w", Capture)
    """
    entryCamera = Entry(win2)
    entryCamera.place(x=600, y=95)

    pictures = Tkinter.Label(win2, text='Take a pictures', fg='black', font=("Helvetica", 12))
    pictures.place(x=20, y=140)

    steps1 = Tkinter.Label(win2, text="STEP 1: Take 10 pictures keeping the chessboard within approximately 0.5 mt in "
                                      "front of Pepper")
    steps1.place(x=20, y=170)
    # Do1 = Button(win2, text='Capture', command=partial(Capture, i))

    Do1 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do1.place(x=200, y=200)

    Save1 = Button(win2, text='Save them', command=SavePictures)
    Save1.place(x=400, y=200)

    steps2 = Tkinter.Label(win2, text="STEP 2: Take 10 pictures with the chessboard titled up to approximately +45 "
                                      "degrees in horizontal direction")
    steps2.place(x=20, y=230)

    Do2 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do2.place(x=200, y=260)

    Save2 = Button(win2, text='Save them', command=SavePictures)
    Save2.place(x=400, y=260)

    steps3 = Tkinter.Label(win2, text="STEP 3: Take 10 pictures with the chessboard titled up to approximately -45 "
                                      "degrees in horizontal direction")
    steps3.place(x=20, y=290)
    Do3 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do3.place(x=200, y=320)

    Save3 = Button(win2, text='Save them', command=SavePictures)
    Save3.place(x=400, y=320)

    steps4 = Tkinter.Label(win2, text="STEP 4: Take 10 pictures with the chessboard titled up to approximately +45 "
                                      "degrees in vertical direction")
    steps4.place(x=20, y=350)

    Do4 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do4.place(x=200, y=380)

    Save4 = Button(win2, text='Save them', command=SavePictures)
    Save4.place(x=400, y=380)

    steps5 = Tkinter.Label(win2, text="STEP 5: Take 10 pictures with the chessboard titled up to approximately -45 "
                                      "degrees in vertical direction")
    steps5.place(x=20, y=410)

    Do5 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do5.place(x=200, y=440)

    Save5 = Button(win2, text='Save them', command=SavePictures)
    Save5.place(x=400, y=440)

    steps6 = Tkinter.Label(win2, text="STEP 6: Take 10 pictures keeping the chessboard parallel to Pepper")
    steps6.place(x=20, y=470)

    Do6 = Button(win2, text='Capture', command=partial(Capture, session, i, entryCamera))
    Do6.place(x=200, y=500)

    Save6 = Button(win2, text='Save them', command=SavePictures)
    Save6.place(x=400, y=500)

    calibrate = Tkinter.Label(win2, text='Calibration', fg='black', font=("Helvetica", 12))
    calibrate.place(x=20, y=530)

    Calibration = Tkinter.Label(win2, text='STEP 7: perform Calibration of the camera')
    Calibration.place(x=20, y=550)

    Btn_Cal = Button(win2, text='Calibrate Camera', command=CalibrationCamera)
    Btn_Cal.place(x=250, y=580)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="130.251.13.188",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()

    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    root = Tkinter.Tk()
    Window2(root)
    root.mainloop()
    entryCamera = Entry(root)
    entryCamera.place(x=250, y=480)
