#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#####################################################################
#                                                                   #
#                  FRC Edge Detection Test App                      #
#                                                                   #
#  This file contains a program for testing edge detection for FRC  #
#  robot applications.  This program uses methods out of the        #
#  FRCCameraLibrary for defining cameras and grabbing frames from   #
#  the web cam.                                                     #
#                                                                   #
#  @Version: 1.0                                                    #
#  @Created: 2021-01-18                                             #
#  @Author: Team 4121                                               #
#                                                                   #
#####################################################################

"""Edge detection test application"""

# System imports
import sys
import os
import imp

# Setup paths for PI use
sys.path.append('/usr/local/lib/python3.7/dist-packages')
sys.path.append('/home/pi/.local/lib/python3.7/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')

# Module imports
import cv2 as cv
import numpy as np
#from matplotlib import pyplot as plt

# Team 4121 module imports
from FRCVisionLibrary import VisionLibrary
from FRCCameraLibrary import FRCWebCam


#Define main processing function
def main():

    #Create web camera stream
    camSettings = {}
    camSettings['Width'] = 640
    camSettings['Height'] = 480
    camSettings['Brightness'] = 0.3
    camSettings['Exposure'] = 30
    camSettings['FPS'] = 15
    webCamera = FRCWebCam(0, "WebCam", camSettings)
    webCamera.start_camera()

    #Create blank image
    imgRaw = np.zeros(shape=(640, 480, 3), dtype=np.uint8)

    #Main processing loop
    while True:
        
        #Grab frames from stereo camera
        imgRaw = webCamera.read_frame()

        #Process frame to find edges
        imgEdges = VisionLibrary.process_image_edges(imgRaw)

        #Show images
        cv.imshow('Raw Image', imgRaw)
        cv.imshow('Image Edges', imgEdges)
        
        #Look for user break
        if cv.waitKey(1) == 27:
            break

    #Close all open windows
    cv.destroyAllWindows()

    #Release all cameras
    webCamera.release_cam()


# Run main program
if __name__ == '__main__':
    main()
