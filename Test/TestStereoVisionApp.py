#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
#                                                                  #
#                 FRC Stereo Camera Test App                       #
#                                                                  #
#  This file contains a program for testing stereo vision for FRC  #
#  robot applications.  This program uses methods out of the       #
#  FRCStereoCameraLibrary for defining cameras and grabbing        #
#  synchronised frames from the two web cams.                      #
#                                                                  #
#  @Version: 1.0                                                   #
#  @Created: 2020-07-09                                            #
#  @Author: Team 4121                                              #
#                                                                  #
####################################################################

"""Stereo camera test application"""

# System imports
import sys
import os
import imp

# Setup paths for PI use
sys.path.append('/usr/local/lib/python3.5/dist-packages')
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')

# Setup paths for Windows use
#sys.path.append('C:/FRC-Test/Libraries')

# Module imports
import cv2 as cv
import numpy as np
import math
import datetime
import time
import logging
#from matplotlib import pyplot as plt

# Team 4121 module imports
from FRCVisionLibrary import VisionLibrary
from FRCStereoCameraLibrary import FRCStereoCam

# Declare global variables
cameraFile = '/home/pi/Team4121/Config/2020CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2020VisionSettings.txt'
#cameraFile = 'C:/FRC-Test/Config/2020CameraSettings.txt'
#visionFile = 'C:/FRC-Test/Config/2020VisionSettings.txt'
cameraValues={}


#Read vision settings file
def read_settings_file():

    #Declare global variables
    global cameraFile
    global cameraValues

    #Open the file and read contents
    try:
            
        #Open the file for reading
        in_file = open(cameraFile, 'r')
            
        #Read in all lines
        value_list = in_file.readlines()
            
        #Process list of lines
        for line in value_list:
                
            #Remove trailing newlines and whitespace
            clean_line = line.strip()

            #Split the line into parts
            split_line = clean_line.split(',')

            #Save the value into dictionary
            cameraValues[split_line[0]] = split_line[1]

    except:

        cameraValues['BallCamFOV'] = 27.3
        cameraValues['BallCamWidth'] = 320
        cameraValues['BallCamHeight'] = 240
        cameraValues['BallCamFPS'] = 15
        cameraValues['BallCamBrightness'] = 50
        cameraValues['BallCamExposure'] = 50
        cameraValues['BallCamCalFactor'] = 1
        cameraValues['BallCamFocalLength'] = 333.29
        cameraValues['GoalCamFOV'] = 27.3
        cameraValues['GoalCamWidth'] = 320
        cameraValues['GoalCamHeight'] = 240
        cameraValues['GoalCamFPS'] = 15
        cameraValues['GoalCamBrightness'] = 0
        cameraValues['GoalCamExposure'] = 0
        cameraValues['GoalCamCalFactor'] = 1
        cameraValues['GoalCamFocalLength'] = 333.29


#Define main processing function
def mainloop():

    #Read camera settings file
    read_settings_file()

    #Create stereo camera stream
    camSettings = {}
    camSettings['Width'] = cameraValues['BallCamWidth']
    camSettings['Height'] = cameraValues['BallCamHeight']
    camSettings['Brightness'] = cameraValues['BallCamBrightness']
    camSettings['Exposure'] = cameraValues['BallCamExposure']
    camSettings['FPS'] = cameraValues['BallCamFPS']
    stereoCamera = FRCStereoCam(0, 1, "StereoCam", camSettings)
    stereoCamera.start_camera()

    #Create blank images
    leftImg = np.zeros(shape=(int(cameraValues['BallCamWidth']), int(cameraValues['BallCamHeight']), 3), dtype=np.uint8)
    rightImg = np.zeros(shape=(int(cameraValues['BallCamWidth']), int(cameraValues['BallCamHeight']), 3), dtype=np.uint8)

    #Main processing loop
    while True:
        
        #Grab frames from stereo camera
        leftImg, rightImg = stereoCamera.read_frame()

        #Blur image to remove noise
        leftImg_Blur = cv.GaussianBlur(leftImg.copy(),(5,5),0)
        rightImg_Blur = cv.GaussianBlur(rightImg.copy(),(5,5),0)

        #Convert images to grayscale
        leftImg_Gray = cv.cvtColor(leftImg_Blur, cv.COLOR_BGR2GRAY)
        rightImg_Gray = cv.cvtColor(rightImg_Blur, cv.COLOR_BGR2GRAY)

        #Create depth map
        stereo = cv.StereoBM.create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(leftImg_Gray,rightImg_Gray)

        #Show images
        cv.imshow('Camera 1', leftImg_Gray)
        cv.imshow('Camera 2', rightImg_Gray)
        
        #Show depth map
        #plt.imshow(disparity,'gray')
        #plt.show()

        if cv.waitKey(1) == 27:
            break

    #Close all open windows
    cv.destroyAllWindows()

    #Release all cameras
    stereoCamera.release_cam()


#Define main function
def main():
    mainloop()

if __name__ == '__main__':
    main()
    
