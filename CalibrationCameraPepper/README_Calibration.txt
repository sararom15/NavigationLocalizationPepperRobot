Download Python SDK for Naoqi and set the environment variables as explained in the installation guide:
http://doc.aldebaran.com/2-5/dev/python/install_guide.html

-----------------------------------------------------------------------------------------------
These nodes are responsable of the calibration of the Pepper's cameras. 

1) "GUI_Calibration_Introduction.py" : the first node to run, it opens a GUI with all the instructions to have a good calibration; 

2) The previous node triggers the second GUI and runs the second code: "GUI_Calibration.py", responsable of the calibration itself. The GUI allows you to select the Pepper camera (bottom ot top), take a pictures to the chessboard and save them. 

__________________________________________________________________________________________
HOW TO RUN

-BEFORE TO RUN:
1) open the 'GUI_Calibration.py' and defines the folder in your own computer, where the image will be save by default, and where the images must be saved in order to perform calibration. 

2) Be careful to insert the Pepper IP. 


-RUN THE APPLICATION: 
Run in the terminal: 'python GUI_Calibration_Introduction.py' 