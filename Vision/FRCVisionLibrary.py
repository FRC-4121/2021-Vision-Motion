# -*- coding: utf-8 -*-

#-----------------------------------------------------------------
#
# FRC Vision Library
#
# This class provides numerous methods and utilities for vision
# processing during an FRC game.  The provided methods cover
# finding standard game elements (balls, cubes, etc.) as well as 
# retroreflective vision targets in video frames.
# 
# @Version: 1.0
#  
# @Created: 2020-1-8
#
# @Author: Team 4121
#
#-----------------------------------------------------------------

'''FRC Vision Library - Provides vision processing for game elements'''

#!/usr/bin/env python3

#System imports
import os

#Module Imports
import cv2 as cv
import numpy as np 
import math

#Define the vision library class
class VisionLibrary:

    #Define class fields
    visionFile = ""
    ball_values = {}
    goal_values = {}
    tape_values = {}


    #Define class initialization
    def __init__(self, visionfile):
        
        #Read in vision settings file
        VisionLibrary.visionFile = visionfile
        self.read_vision_file(VisionLibrary.visionFile)


    #Read vision settings file
    def read_vision_file(self, file):

        #Declare local variables
        value_section = ''
        new_section = False

        #Open the file and read contents
        try:
            
            #Open the file for reading
            in_file = open(file, 'r')
            
            #Read in all lines
            value_list = in_file.readlines()
            
            #Process list of lines
            for line in value_list:
                
                #Remove trailing newlines and whitespace
                clean_line = line.strip()

                #Split the line into parts
                split_line = clean_line.split(',')

                #Determine section of the file we are in
                if split_line[0].upper() == 'BALL:':
                    value_section = 'BALL'
                    new_section = True
                elif split_line[0].upper() == 'GOALTARGET:':
                    value_section = 'GOALTARGET'
                    new_section = True
                elif split_line[0].upper() == 'VISIONTAPE:':
                    value_section = 'VISIONTAPE'
                    new_section = True
                elif split_line[0] == '':
                    value_section = ''
                    new_section = True
                else:
                    new_section = False

                #Take action based on section
                if new_section == False:
                    if value_section == 'BALL':
                        VisionLibrary.ball_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'GOALTARGET':
                        VisionLibrary.goal_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'VISIONTAPE':
                        VisionLibrary.tape_values[split_line[0].upper()] = split_line[1]
                    else:
                        new_section = True
        
        except FileNotFoundError:
            return False
        
        return True


    #Define basic image processing method for contours
    def process_image_contours(self, imgRaw, hsvMin, hsvMax):
    
        #Blur image to remove noise
        blur = cv.GaussianBlur(imgRaw.copy(),(13,13),0)
        
        #Convert from BGR to HSV colorspace
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        #Set pixels to white if in target HSV range, else set to black
        mask = cv.inRange(hsv, hsvMin, hsvMax)

        #Find contours in mask
        _, contours, _ = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    
        return contours


    #Define basic image processing method for edge detection
    def process_image_edges(self, imgRaw):

        #Blur image to remove noise
        blur = cv.GaussianBlur(imgRaw.copy(),(13,13),0)
        
        #Convert from BGR to HSV colorspace
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        #Detect edges
        edges = cv.Canny(hsv, 35, 125)

        #Find contours using edges
        _, contours, _ = cv.findContours(edges.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

        return contours


    #Find ball game pieces
    def detect_game_balls(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):

        #Read HSV values from dictionary and make tuples
        hMin = int(VisionLibrary.ball_values['HMIN'])
        hMax = int(VisionLibrary.ball_values['HMAX'])
        sMin = int(VisionLibrary.ball_values['SMIN'])
        sMax = int(VisionLibrary.ball_values['SMAX'])
        vMin = int(VisionLibrary.ball_values['VMIN'])
        vMax = int(VisionLibrary.ball_values['VMAX'])
        ballHSVMin = (hMin, sMin, vMin)
        ballHSVMax = (hMax, sMax, vMax)
        
        #Initialize values to be returned
        targetRadius = 0 #px
        targetX = -1 #px
        targetY = -1 #px
        distanceToBall = 0 #inches
        angleToBall = 0 #degrees
        ballOffset = 0
        screenPercent = 0
        foundBall = False

        #Find contours in the mask and clean up the return style from OpenCV
        ballContours = self.process_image_contours(imgRaw, ballHSVMin, ballHSVMax)

        #Only proceed if at least one contour was found
        if len(ballContours) > 0:
 
            largestContour = max(ballContours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largestContour)

            if radius > int(VisionLibrary.ball_values['MINRADIUS']):

                targetRadius = radius
                targetX = x
                targetY = y
                foundBall = True

            #Distance and angle offset calculations
            if targetRadius > 0:
            
                inches_per_pixel = float(VisionLibrary.ball_values['RADIUS'])/targetRadius #set up a general conversion factor
                distanceToBall = inches_per_pixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                offsetInInches = inches_per_pixel * (targetX - cameraWidth / 2)
                angleToBall = math.degrees(math.atan((offsetInInches / distanceToBall)))
                screenPercent = math.pi * targetRadius * targetRadius / (cameraWidth * cameraHeight)
                ballOffset = -offsetInInches
          
            else:
            
                distanceToBall = -1
                angleToBall = -1

        return targetX, targetY, targetRadius, distanceToBall, angleToBall, ballOffset, screenPercent, foundBall


    #Define general tape detection method (rectangle good for generic vision tape targets)
    def detect_tape_rectangle(self, imgRaw, cameraWidth, cameraHeight, cameraFOV):

        focalLength = 334.29
        baseTargetWidthPx = 95#from 120 inches
        mountAngle = 29.5
    
        #Read HSV values from dictionary and make tupples
        hMin = int(VisionLibrary.tape_values['HMIN'])
        hMax = int(VisionLibrary.tape_values['HMAX'])
        sMin = int(VisionLibrary.tape_values['SMIN'])
        sMax = int(VisionLibrary.tape_values['SMAX'])
        vMin = int(VisionLibrary.tape_values['VMIN'])
        vMax = int(VisionLibrary.tape_values['VMAX'])
        tapeHSVMin = (hMin, sMin, vMin)
        tapeHSVMax = (hMax, sMax, vMax)

        #Values to be returned
        targetX = -1
        targetY = -1
        targetW = -1
        targetH = -1
        aspectRatio = 0
        centerOffset = 0
        cameraAngle = 0
        botAngle = 0
        distanceToTape = 0
        horizAngleToTape = 0
        vertAngleToTape = 0
        #robotLeft = 
        foundTape = False
        inchesPerPixel = 0
        distanceTerm1 = 0
        distanceTerm2 = 0
        horizOffsetPixels = 0
        rect = None
        box = None

        #Return dictionary
        tapeCameraValues = {}
        tapeRealWorldValues = {}
        
    
        #Find alignment tape in image
        tapeContours = self.process_image_contours(imgRaw, tapeHSVMin, tapeHSVMax)
  
        #Continue with processing if alignment tape found
        if len(tapeContours) > 0:

            #find the largest contour and check it against the mininum tape area
            largestContour = max(tapeContours, key=cv.contourArea)
                        
            if cv.contourArea(largestContour) > int(VisionLibrary.tape_values['MINAREA']):
                
                #Find horizontal rectangle
                targetX, targetY, targetW, targetH = cv.boundingRect(largestContour)

                #Find angled rectangle
                rect = cv.minAreaRect(largestContour)#((x, y), (h, w), angle)
                box = cv.boxPoints(rect)
                box = np.int0(box)

                #Find angle of bot to target
                angle = rect[2]
                print(str(angle))
                if abs(angle) < 45:
                    cameraAngle = abs(angle)
                    #targetW = int(rect[1][0])
                    #robotLeft = False
                else:
                    cameraAngle = 90 - abs(angle)
                    #targetW = int(rect[1][1])

                botAngle = 2 * cameraAngle

                #Set flag
                foundTape = True
                
                aspectRatio = targetW / targetH
##                if abs(aspectRatio - float(VisionLibrary.tape_values['TAPEWIDTH'])/float(VisionLibrary.tape_values['TAPEHEIGHT'])) < float(VisionLibrary.tape_values['ARTOLERANCE']):
##                    foundTape = True

            #calculate real world values of found tape
            if foundTape:
                
                inchesPerPixel = float(VisionLibrary.tape_values['TAPEWIDTH']) / targetW

##                if (targetW * inchesPerPixel) > float(VisionLibrary.tape_values['TAPEWIDTH']):
##                    targetW = float(VisionLibrary.tape_values['TAPEWIDTH']) / inchesPerPixel
##                    
                #depthAngle = math.acos(targetW * inchesPerPixel / float(VisionLibrary.tape_values['TAPEWIDTH']))
                horizOffsetPixels = (targetX + targetW/2) - cameraWidth / 2
                horizOffsetInInches = inchesPerPixel * horizOffsetPixels

##                constantTerm = (2 * math.tan(math.degrees(cameraFOV)))
##                print("cameraWidth: " + str(cameraWidth))
##                print("cameraFOV: " + str(cameraFOV))
##                print("demonimator: " + str(constantTerm))
##                distanceTerm1Part1 = math.pow(cameraWidth/(2 * math.tan(math.radians(cameraFOV))), 2)
##                print("Part 1: " + str(distanceTerm1Part1))
##                distanceTerm1Part2 = math.pow(horizOffsetPixels, 2)
##                print("Part 2: " + str(distanceTerm1Part2))
##
##                distanceTerm1 = math.sqrt(math.pow(cameraWidth/(2 * math.tan(math.radians(cameraFOV))), 2) + math.pow(horizOffsetPixels, 2))
##                distanceTerm2 = math.tan(math.radians(botAngle)) * ((cameraWidth / 2) - horizOffsetPixels)
                #distanceToTape = inchesPerPixel * (distanceTerm1 + distanceTerm2)
                #distanceToTape = inchesPerPixel * (cameraWidth / (2 * math.tan(math.radians(cameraFOV))))
                
                straightLineDistance = float(VisionLibrary.tape_values['TAPEWIDTH']) * focalLength / targetW
                print(str(straightLineDistance))
                #depthStraightDistance = straightLineDistance * targetW / baseTargetWidthPx
                groundStraightDistance = straightLineDistance * math.cos(math.radians(mountAngle))
                distanceToTape = groundStraightDistance / math.cos(math.radians(botAngle))                
                
                horizAngleToTape = math.degrees(math.atan((horizOffsetInInches / distanceToTape)))
                vertOffsetInInches = inchesPerPixel * ((cameraHeight / 2) - (targetY - targetH/2))
                vertAngleToTape = math.degrees(math.atan((vertOffsetInInches / distanceToTape)))
                centerOffset = -horizOffsetInInches

        #Fill dictionary
        tapeCameraValues['TargetX'] = targetX
        tapeCameraValues['TargetY'] = targetY
        tapeCameraValues['TargetW'] = targetW
        tapeCameraValues['TargetH'] = targetH
        tapeCameraValues['IPP'] = inchesPerPixel
        tapeCameraValues['Term1'] = distanceTerm1
        tapeCameraValues['Term2'] = distanceTerm2
        tapeCameraValues['Offset'] = horizOffsetPixels
        tapeRealWorldValues['AspectRatio'] = aspectRatio
        tapeRealWorldValues['CenterOffset'] = centerOffset
        tapeRealWorldValues['Distance'] = distanceToTape
        tapeRealWorldValues['HAngle'] = horizAngleToTape
        tapeRealWorldValues['VAngle'] = vertAngleToTape
        tapeRealWorldValues['TargetRotation'] = cameraAngle
        tapeRealWorldValues['BotAngle'] = botAngle

        return tapeCameraValues, tapeRealWorldValues, foundTape, rect, box

