# -*- coding: utf-8 -*-

####################################################################
#                                                                  #
#                   FRC Stereo Camera Library                      #
#                                                                  #
#  This class provides methods and utilities for stereo cameras    #
#  used for vision processing during an FRC game.  The method for  #
#  reading frames from each camera is threaded for improve         #
#  performance.                                                    #
#                                                                  #
#  @Version: 1.0                                                   #
#  @Created: 2020-02-14                                            #
#  @Author: Team                                                   #
#                                                                  #
####################################################################

'''FRC Stereo Camera Library - Provides stereo camera methods and utilities'''

#!/usr/bin/env python3

# System imports
import os

# Module Imports
import cv2 as cv
import numpy as np
from threading import Thread

# Set global variables
calibration_dir = '/home/pi/Team4121/Config'


# Define the web camera class
class FRCStereoCam:

    # Define initialization
    def __init__(self, leftSrc, rightSrc, name, settings):

        # Initialize instance variables
        self.undistort_left = False
        self.undistort_right = False

        # Set up left camera
        self.left_id = leftSrc
        self.leftCamStream = cv.VideoCapture(self.left_id)
        self.leftCamStream.set(cv.CAP_PROP_FRAME_WIDTH, int(settings['Width']))
        self.leftCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, int(settings['Height']))
        self.leftCamStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.leftCamStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.leftCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Make sure lefgt camera is opened
        if self.leftCamStream.isOpened() == False:
            self.leftCamStream.open(self.left_id)

        # Set up right camera
        self.right_id = rightSrc
        self.rightCamStream = cv.VideoCapture(self.right_id)
        self.rightCamStream.set(cv.CAP_PROP_FRAME_WIDTH, int(settings['Width']))
        self.rightCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, int(settings['Height']))
        self.rightCamStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.rightCamStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.rightCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Make sure right camera is opened
        if self.rightCamStream.isOpened() == False:
            self.rightCamStream.open(self.right_id)

        # Grab an initial frames
        (self.leftGrabbed, self.leftFrame) = self.leftCamStream.read()
        (self.rightGrabbed, self.rightFrame) = self.rightCamStream.read()

        # Name the stream
        self.name = name

        # Initialize stop flag
        self.stopped = False

        # Read left camera calibrarion files
        left_matrix_file = calibration_dir + '/Camera_Matrix_Cam' + str(self.left_id) + '.txt'
        left_coeffs_file = calibration_dir + '/Distortion_Coeffs_Cam' + str(self.left_id) + '.txt'
        if os.path.isfile(left_matrix_file) == True and os.path.isfile(left_coeffs_file) == True:
            self.left_cam_matrix = np.loadtxt(left_matrix_file)
            self.left_distort_coeffs = np.loadtxt(left_coeffs_file)
            self.undistort_left = True
       
        # Read right camera calibration files
        right_matrix_file = calibration_dir + '/Camera_Matrix_Cam' + str(self.right_id) + '.txt'
        right_coeffs_file = calibration_dir + '/Distortion_Coeffs_Cam' + str(self.right_id) + '.txt'
        if os.path.isfile(right_matrix_file) == True and os.path.isfile(right_coeffs_file) == True:
            self.right_cam_matrix = np.loadtxt(right_matrix_file)
            self.right_distort_coeffs = np.loadtxt(right_coeffs_file)
            self.undistort_right = True

    # Define camera thread start method
    def start_camera(self):

        #Define camera thread
        camThread = Thread(target=self.update, name=self.name, args=())
        camThread.daemon = True
        camThread.start()

        return self


    # Define camera thread stop method
    def stop_camera(self):

        #Set stop flag
        self.stopped = True


    # Define camera update method
    def update(self):

        # Main thread loop
        while True:

            # Check stop flag
            if self.stopped:
                return
            
            # If not stopping, grab new frame
            (self.leftGrabbed, self.leftFrame) = self.leftCamStream.read()
            (self.rightGrabbed, self.rightFrame) = self.rightCamStream.read()


    # Define frame read method
    def read_frame(self):

        # Return the most recent frame
        return self.leftFrame, self.rightFrame


    # Define camera release method
    def release_cam(self):

        # Stop the camera thread
        self.stop_camera()

        # Release the camera resource
        self.leftCamStream.release()
        self.rightCamStream.release()
