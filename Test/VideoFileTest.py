# -*- coding: utf-8 -*-
#!/usr/bin/env python3

######################################################
#                                                    #
#                 Video File Test                    #
#                                                    #
#  This program tests reading frames from a web cam  #
#  writing those frames to a video file.             #
#                                                    #
#  @Version: 1.0                                     #
#  @Created: 2020-1-8                                #
#  @Author: Team 4121                                #
#                                                    #
######################################################

'''FRC 4121 - Test of writing video files'''

# System imports
import sys
import os

#Module imports
import cv2 as cv
import numpy as np
import datetime
import time

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')

#Team 4121 module imports
from FRCCameraLibrary import FRCWebCam


# Define main method
def main():

    # Declare global variables
    videoDirectory = '/home/pi/Team4121/Videos'

    # Create an instance of FRC Webcam
    camSettings = {}
    camSettings['Width'] = 320
    camSettings['Height'] = 240
    camSettings['Brightness'] = 50
    camSettings['Exposure'] = 50
    camSettings['FPS'] = 15
    camera = FRCWebCam('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0', "FieldCam", camSettings, 'VideoTest001')

    # Create blank vision image
    imgCam = np.zeros(shape=(320, 240, 3), dtype=np.uint8)

    # Main loop of program
    while (True):

        # Read frame from camera
        imgCam = camera.read_frame()

        # Show image
        cv.imshow("Camera", imgCam)

        # Write frame to file
        camera.write_video(imgCam)

        # Check for stopping
        if cv.waitKey(1) == 27:
            break

    # Close open windows
    cv.destroyAllWindows()

    # Release the camera
    camera.release_cam()


#define main function
if __name__ == '__main__':
    main()



