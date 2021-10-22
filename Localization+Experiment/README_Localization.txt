Remember to set set the environment variables as explained in the installation guide:
http://doc.aldebaran.com/2-5/dev/python/install_guide.html


-----------------------------------------------------------------------------------------------

LOCALISATION CODE

[NOTA: the world frame is defined in the Laboratorium. Look the environment setup in the picture.] 

1) "EKF_Socket.py": responsible of the estimation of the robot state.
The sensor measurement are received by queue. 

What to do before to run: 
- Pepper's IP; 
- Specify the initial position of the robot in the world frame(by default it is set to [0,0,0]). 

2) "PoseEstimation_Pepper_Aruco.py" : responsible of the detection and estimation of the robot position in the world frame. These data are send to the 'EKF_Socket.py' thru Queue-based communication. 

What to do before: 
- Pepper's IP; 
- need to have the Camera Matrix and Distortion Coefficient as .npy format file in the same folder; 
- create inside the code a marker's class, specifying the world position of the involved markers. 


3) "Test_motion_3markers.py": let the robot move in the world frame. With the configuration shown in the environment setup picture, the code allows the robot to move achieving the markers and staying there for a while (20 sec). 
It receives the info about the robot position by tcp/ip Connection from the EKF_Socket.py code. 

What to do before: 
-Pepper's IP. 

--------------------------------------------------

HOW TO RUN THE CODE: 
Open 2 terminals: 
in the first one, run 'python EKF_Socket.py'; 
in the second one, run 'python Test_motion_3markers.py'. 


