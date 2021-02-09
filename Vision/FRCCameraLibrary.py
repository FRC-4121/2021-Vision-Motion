# -*- coding: utf-8 -*-
#!/usr/bin/env python3

######################################################################
#                                                                    #
#                        FRC Camera Library                          #
#                                                                    #
#  This class provides methods and utilities for web cameras used    #
#  for vision processing during an FRC game.  Reading of the webcam  #
#  frames is threaded for improved performance.                      #
#                                                                    #
# @Version: 2.0                                                      #
# @Created: 2021-2-8                                                 #
# @Author: Team 4121                                                 #
#                                                                    #
######################################################################

'''FRC Camera Library - Provides threaded camera methods and utilities'''

# System imports
import os

# Module Imports
import cv2 as cv
import numpy as np
from threading import Thread

# Set global variables
calibration_dir = '/home/pi/Team4121/Config'


# Define the web camera class
class FRCWebCam:

    # Define initialization
    def __init__(self, src, name, settings):

        # Initialize instance variables
        self.undistort_img = False

        # Set up web camera
        self.device_id = src
        self.camStream = cv.VideoCapture(self.device_id)
        self.camStream.set(cv.CAP_PROP_FRAME_WIDTH, int(settings['Width']))
        self.camStream.set(cv.CAP_PROP_FRAME_HEIGHT, int(settings['Height']))
        self.camStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.camStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.camStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Store frame size
        self.height = int(settings['Height'])
        self.width = int(settings['Width'])

        # Make sure video capture is opened
        if self.camStream.isOpened() == False:
            self.camStream.open(self.device_id)

        # Initialize blank frames
        self.frame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        # Grab an initial frame
        self.grabbed, self.frame = self.camStream.read()

        # Name the stream
        self.name = name

        # Read camera calibration files
        cam_matrix_file = calibration_dir + '/Camera_Matrix_Cam' + str(self.device_id) + '.txt'
        cam_coeffs_file = calibration_dir + '/Distortion_Coeffs_Cam' + str(self.device_id) + '.txt'
        if os.path.isfile(cam_matrix_file) == True and os.path.isfile(cam_coeffs_file) == True:
            self.cam_matrix = np.loadtxt(cam_matrix_file)
            self.distort_coeffs = np.loadtxt(cam_coeffs_file)
            self.undistort_img = True


    # Define frame read method
    def read_frame(self):

        # Declare frame for undistorted image
        newFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        # Grab new frame
        self.grabbed, self.frame = self.camStream.read()

        # Undistort image
        if self.undistort_img == True:
            h, w = self.frame.shape[:2]
            new_matrix, roi = cv.getOptimalNewCameraMatrix(self.cam_matrix,
                                                             self.distort_coeffs,
                                                             (w,h),1,(w,h))
            newFrame = cv.undistort(self.frame, self.cam_matrix,
                                    self.distort_coeffs, None,
                                    new_matrix)
            x,y,w,h = roi
            newFrame = newFrame[y:y+h,x:x+w]

        else:

            newFrame = self.frame

        # Return the most recent frame
        return newFrame


    # Define camera release method
    def release_cam(self):

        # Release the camera resource
        self.camStream.release()
