# -*- coding: utf-8 -*-
#!/usr/bin/env python3

####################################################################
#                                                                  #
#                   FRC Stereo Camera Library                      #
#                                                                  #
#  This class provides methods and utilities for stereo cameras    #
#  used for vision processing during an FRC game.  The method for  #
#  reading frames from each camera is threaded for improve         #
#  performance.                                                    #
#                                                                  #
#  @Version: 2.0                                                   #
#  @Created: 2020-02-14                                            #
#  @Revised: 2021-02-16                                            #
#  @Author: Team 4121                                              #
#                                                                  #
####################################################################

'''FRC Stereo Camera Library - Provides stereo camera methods and utilities'''

# System imports
import os

# Module Imports
import cv2 as cv
import numpy as np
from threading import Thread

# Set global variables
calibration_dir = '/home/pi/Team4121/Config'
#calibration_dir = 'C:/FRC-Test/Config/Calibration'


# Define the web camera class
class FRCStereoCam:

    # Define initialization
    def __init__(self, leftSrc, rightSrc, name, settings):

        # Name the stream
        self.name = name

        # Initialize instance variables
        self.undistort_left = False
        self.undistort_right = False

        # Initialize stop flag
        self.stopped = False

        # Read left camera calibrarion files
        left_matrix_file = (calibration_dir + '/Camera_Matrix_Cam' + 
                                str(leftSrc) + '.txt')
        left_coeffs_file = (calibration_dir + '/Distortion_Coeffs_Cam' + 
                                str(leftSrc) + '.txt')
        if (os.path.isfile(left_matrix_file) == True and 
          os.path.isfile(left_coeffs_file) == True):
            self.left_cam_matrix = np.loadtxt(left_matrix_file)
            self.left_distort_coeffs = np.loadtxt(left_coeffs_file)
            self.undistort_left = True
       
        # Read right camera calibration files
        right_matrix_file = (calibration_dir + '/Camera_Matrix_Cam' + 
                                str(rightSrc) + '.txt')
        right_coeffs_file = (calibration_dir + '/Distortion_Coeffs_Cam' + 
                                str(rightSrc) + '.txt')
        if (os.path.isfile(right_matrix_file) == True and 
          os.path.isfile(right_coeffs_file) == True):
            self.right_cam_matrix = np.loadtxt(right_matrix_file)
            self.right_distort_coeffs = np.loadtxt(right_coeffs_file)
            self.undistort_right = True

        # Set up left camera
        self.left_id = leftSrc
        self.leftCamStream = cv.VideoCapture(self.left_id)
        self.leftCamStream.set(cv.CAP_PROP_FRAME_WIDTH, 
                                int(settings['Width']))
        self.leftCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, 
                                int(settings['Height']))
        self.leftCamStream.set(cv.CAP_PROP_BRIGHTNESS, 
                                float(settings['Brightness']))
        self.leftCamStream.set(cv.CAP_PROP_EXPOSURE, 
                                int(settings['Exposure']))
        self.leftCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Make sure left camera is opened
        if self.leftCamStream.isOpened() == False:
            self.leftCamStream.open(self.left_id)

        # Set up right camera
        self.right_id = rightSrc
        self.rightCamStream = cv.VideoCapture(self.right_id)
        self.rightCamStream.set(cv.CAP_PROP_FRAME_WIDTH, 
                                int(settings['Width']))
        self.rightCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, 
                                int(settings['Height']))
        self.rightCamStream.set(cv.CAP_PROP_BRIGHTNESS, 
                                float(settings['Brightness']))
        self.rightCamStream.set(cv.CAP_PROP_EXPOSURE, 
                                int(settings['Exposure']))
        self.rightCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Make sure right camera is opened
        if self.rightCamStream.isOpened() == False:
            self.rightCamStream.open(self.right_id)

        # Store frame size
        self.height = int(settings['Height'])
        self.width = int(settings['Width'])

        # Initialize blank frames
        self.leftFrame = np.zeros(shape=(int(settings['Width']), 
                                    int(settings['Height']), 3), 
                                    dtype=np.uint8)
        self.rightFrame = np.zeros(shape=(int(settings['Width']), 
                                    int(settings['Height']), 3), 
                                    dtype=np.uint8)

        # Grab initial frames
        (self.leftGrabbed, self.leftFrame) = self.leftCamStream.read()
        (self.rightGrabbed, self.rightFrame) = self.rightCamStream.read()


    # Define camera thread start method
    def start_camera_thread(self):

        #Define camera thread
        camThread = Thread(target=self.update, name=self.name, args=())
        camThread.daemon = True
        camThread.start()

        return self


    # Define camera thread stop method
    def stop_camera_thread(self):

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
            self.leftGrabbed, self.leftFrame = self.leftCamStream.read()
            self.rightGrabbed, self.rightFrame = self.rightCamStream.read()


    # Define frame read method
    def read_frame(self):

        # Declare frame for undistorted image
        newLeftFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)
        newRightFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        # Grab new frame
        self.leftGrabbed, self.leftFrame = self.leftCamStream.read()
        self.rightGrabbed, self.rightFrame = self.rightCamStream.read()

        # Undistort images
        if self.undistort_left == True and self.undistort_right == True:

            left_h, left_w = self.leftFrame.shape[:2]
            (new_left_matrix, left_roi) = cv.getOptimalNewCameraMatrix(
                                            self.left_cam_matrix,
                                            self.left_distort_coeffs,
                                            (left_w,left_h),
                                            1,
                                            (left_w,left_h))
            newLeftFrame = cv.undistort(self.leftFrame, 
                                        self.left_cam_matrix,
                                        self.left_distort_coeffs, 
                                        None,
                                        new_left_matrix)
            x,y,w,h = left_roi
            newLeftFrame = newLeftFrame[y:y+h,x:x+w]

            right_h, right_w = self.rightFrame.shape[:2]
            (new_right_matrix, right_roi) = cv.getOptimalNewCameraMatrix(
                                            self.right_cam_matrix,
                                            self.right_distort_coeffs,
                                            (right_w,right_h),
                                            1,
                                            (right_w,right_h))
            newRightFrame = cv.undistort(self.rightFrame, 
                                        self.right_cam_matrix,
                                         self.right_distort_coeffs, 
                                         None,
                                         new_right_matrix)
            x,y,w,h = right_roi
            newRightFrame = newRightFrame[y:y+h,x:x+w]

        else:
            
            newLeftFrame = self.leftFrame
            newRightFrame = self.rightFrame

        # Return the most recent frame
        return newLeftFrame, newRightFrame


    # Define frame read method
    def read_frame_threaded(self):

        # Undistort images
        if self.undistort_left == True and self.undistort_right == True:

            left_h, left_w = self.leftFrame.shape[:2]
            (new_left_matrix, left_roi) = cv.getOptimalNewCameraMatrix(
                                            self.left_cam_matrix,
                                            self.left_distort_coeffs,
                                            (left_w,left_h),
                                            1,
                                            (left_w,left_h))
            newLeftFrame = cv.undistort(self.leftFrame, 
                                        self.left_cam_matrix,
                                        self.left_distort_coeffs, 
                                        None,
                                        new_left_matrix)
            x,y,w,h = left_roi
            newLeftFrame = newLeftFrame[y:y+h,x:x+w]

            right_h, right_w = self.rightFrame.shape[:2]
            (new_right_matrix, right_roi) = cv.getOptimalNewCameraMatrix(
                                            self.right_cam_matrix,
                                            self.right_distort_coeffs,
                                            (right_w,right_h),
                                            1,
                                            (right_w,right_h))
            newRightFrame = cv.undistort(self.rightFrame, 
                                        self.right_cam_matrix,
                                         self.right_distort_coeffs, 
                                         None,
                                         new_right_matrix)
            x,y,w,h = right_roi
            newRightFrame = newRightFrame[y:y+h,x:x+w]

        else:
            
            newLeftFrame = self.leftFrame
            newRightFrame = self.rightFrame

        # Return the most recent frame
        return newLeftFrame, newRightFrame


    # Define camera release method
    def release_cam(self):

        # Release the camera resource
        self.leftCamStream.release()
        self.rightCamStream.release()
