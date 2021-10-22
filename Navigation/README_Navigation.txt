Remember to set the environment variables as explained in the installation guide:
http://doc.aldebaran.com/2-5/dev/python/install_guide.html
--------------------------------------------------------------------------------------
NAVIGATION CODES 

[NOTA: the world frame is defined in the Laboratorium. Look the environment setup in the picture.] 


It is the main node responsable of letting Pepper navigate in the environment, by avoiding obstacles. The robot estimation is provided by the EKF nodes. The obstacles are sensed by laser scanner and tehir positions are printed
in a dynamic map.

HOW TO RUN: 
To run the software, open a single terminal e run: "GUI_Navigation.py". 
Type all the required information and the application. 

-------------------------------------------------------------------------------------
Running the application, the main function of the "Navigation_EKF.py" node is called. 
It creates 4 threads: 

1) Thread that is responsable of detecting aruco in the environemnt (PoseEstimation_Pepper_Aruco.py);
2) Thread that estimates the Robot position thru EKF (EKF.py);
3) Thead that generate the velocity according to the input data (New_navig_with_obstacles.py);
4) Thread that monitors the obstacles positions and prints them into a dynamic map (Navigation_EKF.py).

The exchange of information happens thru queue-based communication. 
