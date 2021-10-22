"""
This file launches the GUI that takes initial and goal position of the robot and IP
The parameters are then used by the Navigation_EKF
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
from Navigation_EKF import *


def printer(param):
    print "ciao", int(param.get())


def Window1(win1):
    win1.title('Parameter definition')
    win1.geometry("700x600")

    title = Label(win1, text="Type the IP address of Pepper", fg='black', font=("Helvetica", 16), anchor='center')
    title.place(x=100, y=20)

    EntryIP = Entry(win1)
    EntryIP.place(x= 500, y=28)

    steps1 = Label(win1, text="By considering the frame defined in the lab, \n" "type the initial position and "
                              "orientation of the chosen Pepper in the environment", fg='black', font=("Helvetica", 12))
    steps1.place(x=30, y=120)

    xpos = Label(win1, text="x value", fg = 'black', font=("Helvetica", 10))
    xpos.place(x=70, y=170)

    EntryXPosition = Entry(win1)
    EntryXPosition.place(x=40, y=190)

    ypos = Label(win1, text="y value", fg = 'black', font=("Helvetica", 10))
    ypos.place(x=270, y=170)

    EntryYPosition = Entry(win1)
    EntryYPosition.place(x=240, y=190)

    thetapos = Label(win1, text="theta value", fg = 'black', font=("Helvetica", 10))
    thetapos.place(x=470, y=170)

    EntryThetaPosition = Entry(win1)
    EntryThetaPosition.place(x=440, y=190)

    steps2 = Label(win1, text="Type the Goal position that the robot has to reach", fg='black', font=("Helvetica", 12))
    steps2.place(x=30, y=300)

    xgoal = Label(win1, text="x value", fg='black', font=("Helvetica", 10))
    xgoal.place(x=150, y=350)

    EntryXGoal = Entry(win1)
    EntryXGoal.place(x=120, y=370)

    ygoal = Label(win1, text="y value", fg = 'black', font=("Helvetica", 10))
    ygoal.place(x=350, y=350)

    EntryYGoal = Entry(win1)
    EntryYGoal.place(x=320, y=370)

    Run = Button(win1, text="Run Application", command=partial(main, EntryIP, EntryXPosition, EntryYPosition,
                                                               EntryThetaPosition, EntryXGoal, EntryYGoal))
    Run.place(x=350, y=450)


if __name__ == "__main__":
    # Solve NSInvalidArgumentException problem on Mac
    root = Tk()
    Window1(root)
    root.mainloop()


