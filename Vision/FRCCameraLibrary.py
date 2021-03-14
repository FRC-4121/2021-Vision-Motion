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
# @Created: 2020-02-07                                               #
# @Revised: 2021-02-16                                               #
# @Author: Team 4121                                                 #
#                                                                    #
######################################################################

'''FRC Camera Library - Provides threaded camera methods and utilities'''

# System imports
import sys
import os
import logging

# Module Imports
import cv2 as cv
import numpy as np
from threading import Thread

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

# Set global variables
calibration_dir = '/home/pi/Team4121/Config'


# Define the web camera class
class FRCWebCam:

    # Define initialization
    def __init__(self, src, name, settings, logfilenumber, videofile):

        #Open a log file
        logFilename = '/home/pi/Team4121/Logs/Webcam_Log_' + logfilenumber + '.txt'
        self.log_file = open(logFilename, 'w')
        self.log_file.write('Initializing webcam: ' + str(logfilenumber))
        self.log_file.write('')

        # Initialize instance variables
        self.undistort_img = False

        # Store frame size
        self.height = int(settings['Height'])
        self.width = int(settings['Width'])

        # Set up web camera
        self.device_id = src
        self.camStream = cv.VideoCapture(self.device_id)
        self.camStream.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.camStream.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.camStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.camStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        # Set up video writer
        self.videoFilename = '/home/pi/Team4121/Videos/' + videofile + '.avi'
        self.fourcc = cv.VideoWriter_fourcc('M','J','P','G')
        self.camWriter = cv.VideoWriter()

        try:
            self.camWriter.open(self.videoFilename, self.fourcc, 
                                float(settings['FPS']), 
                                (self.width, self.height),
                                True)
        except:
            self.log_file.write('Error opening video writer for file: ' + self.videoFilename)
        
        if (self.camWriter.isOpened()):
            self.log_file.write("Video writer is open")
        else:
            self.log_file.write("Video writer is NOT open")

        # Make sure video capture is opened
        if self.camStream.isOpened() == False:
            self.camStream.open(self.device_id)

        # Initialize blank frames
        #self.frame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        # Grab an initial frame
        self.grabbed, self.frame = self.camStream.read()

        # Name the stream
        self.name = name

        # Initialize stop flag
        self.stopped = False

        # Read camera calibration files
        cam_matrix_file = calibration_dir + '/Camera_Matrix_Cam' + str(self.device_id) + '.txt'
        cam_coeffs_file = calibration_dir + '/Distortion_Coeffs_Cam' + str(self.device_id) + '.txt'
        if os.path.isfile(cam_matrix_file) == True and os.path.isfile(cam_coeffs_file) == True:
            self.cam_matrix = np.loadtxt(cam_matrix_file)
            self.distort_coeffs = np.loadtxt(cam_coeffs_file)
            self.undistort_img = True
        
        # Log init complete message
        self.log_file.write("Webcam initialization complete")


    # Define camera thread start method
    def start_camera_thread(self):

        # Define camera thread
        camThread = Thread(target=self.update, name=self.name, args=())
        camThread.daemon = True
        camThread.start()

        return self


    # Define camera thread stop method
    def stop_camera_thread(self):

        # Set stop flag
        self.stopped = True    


    # Define threaded update method
    def update(self):

        # Main thread loop
        while True:

            # Check stop flag
            if self.stopped:
                return

            # If not stopping, grab new frame
            self.grabbed, self.frame = self.camStream.read()


    # Define frame read method
    def read_frame(self):

        # Declare frame for undistorted image
        newFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        try:

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

        except Exception as read_error:

            # Write error to log
            self.log_file.write('Error reading video:')
            self.log_file.write(type(read_error))
            self.log_file.write(read_error.args)
            self.log_file.write(read_error)

        # Return the most recent frame
        return newFrame


    # Define threaded frame read method
    def read_frame_threaded(self):

        # Declare frame for undistorted image
        newFrame = np.zeros(shape=(self.width, self.height, 3), dtype=np.uint8)

        try:

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

        except Exception as read_error:

            # Write error to log
            self.log_file.write('Error reading video (threaded):')
            self.log_file.write(type(read_error))
            self.log_file.write(read_error.args)
            self.log_file.write(read_error)

        # Return the most recent frame
        return newFrame


    # Define video writing method
    def write_video(self, img):

        # Check if write is opened
        if (self.camWriter.isOpened()):

            # Write the image
            try:

                self.camWriter.write(img)
                return True

            except Exception as write_error:

                # Print exception info
                self.log_file.write('Error writing video:')
                self.log_file.write(type(write_error))
                self.log_file.write(write_error.args)
                self.log_file.write(write_error)
                return False
        
        else:

            self.log_file.write('Video writer not opened')
            return False


    # Define camera release method
    def release_cam(self):

        # Release the camera resource
        self.camStream.release()

        # Release video writer
        self.camWriter.release()

        # Close the log file
        self.log_file.write('Webcam closed. Video writer closed.')
        self.log_file.close()

