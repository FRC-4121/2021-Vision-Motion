# -*- coding: utf-8 -*-

#-----------------------------------------------------------------
#
# FRC Camera Library
#
# This class provides methods and utilities for web cameras used
# for vision processing during an FRC game.
# 
# @Version: 1.0
#  
# @Created: 2020-2-7
#
# @Author: Team 4121
#
#-----------------------------------------------------------------

'''FRC Camera Library - Provides camera methods and utilities'''

#!/usr/bin/env python3

#System imports
import os

#Module Imports
import cv2 as cv
from threading import Thread

#Define the web camera class
class FRCWebCam:

    #Define initialization
    def __init__(self, src, name, settings):

        #Set up web camera
        self.camStream = cv.VideoCapture(src)
        self.camStream.set(cv.CAP_PROP_FRAME_WIDTH, settings['Width'])
        self.camStream.set(cv.CAP_PROP_FRAME_HEIGHT, settings['Height'])
        self.camStream.set(cv.CAP_PROP_BRIGHTNESS, settings['Brightness'])
        self.camStream.set(cv.CAP_PROP_EXPOSURE, settings['Exposure'])
        self.camStream.set(cv.CAP_PROP_FPS, settings['FPS'])

        #Grab an initial frame
        (self.grabbed, self.frame) = self.camStream.read()

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
            (self.grabbed, self.frame) = self.camStream.read()


    #Define frame read method
    def read_frame(self):

        #Return the most recent frame
        return self.frame


    #Define camera release method
    def release_cam(self):

        #Stop the camera thread
        self.stop_camera()

        #Release the camera resource
        self.camStream.release()
