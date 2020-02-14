# -*- coding: utf-8 -*-

#-----------------------------------------------------------------
#
# FRC Stereo Camera Library
#
# This class provides methods and utilities for stereo cameras
# used for vision processing during an FRC game.
# 
# @Version: 1.0
#  
# @Created: 2020-2-14
#
# @Author: Team 4121
#
#-----------------------------------------------------------------

'''FRC Stereo Camera Library - Provides stereo camera methods and utilities'''

#!/usr/bin/env python3

#System imports
import os

#Module Imports
import cv2 as cv
from threading import Thread

#Define the web camera class
class FRCStereoCam:

    #Define initialization
    def __init__(self, leftSrc, rightSrc, name, settings):

        #Set up left camera
        self.leftCamStream = cv.VideoCapture(leftSrc)
        self.leftCamStream.set(cv.CAP_PROP_FRAME_WIDTH, int(settings['Width']))
        self.leftCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, int(settings['Height']))
        self.leftCamStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.leftCamStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.leftCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        #Set up right camera
        self.rightCamStream = cv.VideoCapture(rightSrc)
        self.rightCamStream.set(cv.CAP_PROP_FRAME_WIDTH, int(settings['Width']))
        self.rightCamStream.set(cv.CAP_PROP_FRAME_HEIGHT, int(settings['Height']))
        self.rightCamStream.set(cv.CAP_PROP_BRIGHTNESS, float(settings['Brightness']))
        self.rightCamStream.set(cv.CAP_PROP_EXPOSURE, int(settings['Exposure']))
        self.rightCamStream.set(cv.CAP_PROP_FPS, int(settings['FPS']))

        #Grab an initial frames
        (self.leftGrabbed, self.leftFrame) = self.leftCamStream.read()
        (self.rightGrabbed, self.rightFrame) = self.rightCamStream.read()

        #Name the stream
        self.name = name

        #Initialize stop flag
        self.stopped = False


    #Define camera thread start method
    def start_camera(self):

        #Define camera thread
        camThread = Thread(target=self.update, name=self.name, args=())
        camThread.daemon = True
        camThread.start()

        return self


    #Define camera thread stop method
    def stop_camera(self):

        #Set stop flag
        self.stopped = True


    #Define camera update method
    def update(self):

        #Main thread loop
        while True:

            #Check stop flag
            if self.stopped:
                return
            
            #If not stopping, grab new frame
            (self.leftGrabbed, self.leftFrame) = self.leftCamStream.read()
            (self.rightGrabbed, self.rightFrame) = self.rightCamStream.read()


    #Define frame read method
    def read_frame(self):

        #Return the most recent frame
        return self.leftFrame, self.rightFrame


    #Define camera release method
    def release_cam(self):

        #Stop the camera thread
        self.stop_camera()

        #Release the camera resource
        self.leftCamStream.release()
        self.rightCamStream.release()
