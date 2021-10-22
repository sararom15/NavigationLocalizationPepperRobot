"""
Main GUI: presentation of the steps to follow to have a good accurate Calibration of the camera.

It triggers the second GUI which will be responsable of the calibration itself.
"""


from Tkinter import *
import qi
import argparse
import sys
import GUI_Calibration


def call_win2():
    win2 = Toplevel(root)
    GUI_Calibration.Window2(win2, session)
    return


def Window1(win1):
    win1.title('Overview Camera Calibration')
    win1.geometry("1300x700+10+20")

    title = Label(win1, text="Steps to follow for an accurate Camera Calibration", fg='red', font=("Helvetica", 16),
                  anchor='center')
    title.place(x=300, y=20)
    overview = Label(win1, text="Accurate calibration is of key importance for performance in most machine and "
                                "computer vision tasks. \n"
                                "The following lists our best practices which we have found through extensive "
                                "experimentation and theoretical considerations.",
                     anchor='center', font=("Helvetica", 12))
    overview.place(x=120, y=50)

    steps1 = Label(win1, text="1. Choose the right size calibration target. Large enough to properly constrain "
                              "parameters. Preferably it should cover approx. half of the total area when seen "
                              "fronto-parallel in the camera images. ", fg='black', font=("Helvetica", 8))
    steps1.place(x=30, y=100)

    steps2 = Label(win1, text="2. Perform calibration at the approximate working distance (WD) of your final "
                              "application. The camera should be focused at this distance as low as 0.5 meter to as "
                              "far as 10 meters from the camera, and lens focus should be unchanged after calibration.",
                   fg='black', font=("Helvetica", 8))
    steps2.place(x=30, y=120)

    steps3 = Label(win1, text="3. The target should have a high feature count. Using fine patterns is preferable. "
                              "However, at some point detection robustness suffers. Our recommendation is to use fine "
                              "pattern counts for cameras above 3MPx and if the lighting is controlled and good.",
                   fg='black', font=("Helvetica", 8))
    steps3.place(x=30, y=140)

    steps4 = Label(win1, text="4. Collect images from different areas and tilts. Move the target to fully cover the "
                              "image area and aim for even coverage. Lens distortion can be properly determined from "
                              "fronto-parallel images, but focal length estimation is dependent on observing "
                              "foreshortening. \n Include both frontoparallel images, and images taken with the board "
                              "tilted up to +/- 45 degrees in both horizontal an vertical directions. Tilting more is "
                              "usually not a good idea as feature localization accuracy suffers and can become biased.",
                   fg='black', font=("Helvetica", 8))
    steps4.place(x=30, y=160)

    steps5 = Label(win1, text="5. Use good lighting. This is often overlooked, but hugely important. The calibration "
                              "target should preferably be diffusely lit by means of controlled photography lighting. "
                              "Strong point sources give rise to uneven illumination, possibly making detection fail, "
                              "\n and not utilizing the camera's dynamic range very well.  Shadows can do the same.",
                   fg='black', font=("Helvetica", 8))
    steps5.place(x=30, y=200)

    steps6 = Label(win1, text="6.Have enough observations. Usually, calibration should be performed on at least 6 "
                              "observations (images) of a calibration target. If a higher order camera or distortion "
                              "model is used, more observations are beneficial.", fg='black', font=("Helvetica", 8))
    steps6.place(x=30, y=240)

    steps7 = Label(win1, text="7. Consider using uniquely coded targets such as CharuCo boards or Chessboards. These "
                              "allow you to gather observations from the very edges of the camera sensor and lens, "
                              "and hence constrain the distortion parameters very well. \n Also, they allow you to "
                              "collect data even when some of the feature points do not fulfil the other requirements.",
                   fg='black', font=("Helvetica", 8))
    steps7.place(x=30, y=260)

    steps8 = Label(win1, text="8. Calibration is only as accurate as the calibration target used. Use laser or inkjet "
                              "printed targets only to validate and test.", fg='black', font=("Helvetica", 8))
    steps8.place(x=30, y=300)

    steps9 = Label(win1, text="9.Proper mounting of calibration target and camera. In order to minimize distortion and "
                              "bow in larger targets, mount them either vertically, or laying flat on a rigid support. "
                              "Consider moving the camera instead of the target in these cases instead. \n Use a "
                              "quality tripod, and avoid touching the camera during acquisitions.  ", fg='black',
                   font=("Helvetica", 8))
    steps9.place(x=30, y=320)

    steps10 = Label(win1, text="10. Remove bad observations. Carefully inspect reprojection errors. Both per-view and "
                               "per-feature. If any of these appear as outliers, exclude them and recalibrate. ",
                    fg='black', font=("Helvetica", 8))
    steps10.place(x=30, y=360)

    steps11 = Label(win1, text="11. Obtaining a low reproduction error does not equal a good camera calibration, but "
                               "merely indicates that the provided data/evidence can be described with the used model. "
                               "This could be due to overfitting. \n Parameter uncertainties are indications of how "
                               "well the chosen camera model was constrained. ", fg='black', font=("Helvetica", 8))
    steps11.place(x=30, y=380)

    steps12 = Label(win1, text="12. Analyse the individual reprojection errors. Their direction and magnitude should "
                               "not correlate with position, i.e. they should point chaotically in all directions. \n "
                               "Calib.io's Camera Calibrator software provides powerfull visualizations to investigate "
                               "the reprojected errors.", fg='black', font=("Helvetica", 8))
    steps12.place(x=30, y=420)

    btn = Button(win1, text='I understand', command=call_win2)
    btn.place(x=500, y=600)


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
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    root = Tk()
    Window1(root)
    root.mainloop()




