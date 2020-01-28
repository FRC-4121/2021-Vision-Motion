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
class FRCVisionLibrary:

    #Define class fields
    visionFile = ""
    camera_values = {}
    ball_values = {}
    goal_values = {}
    tape_values = {}
    

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


    #Find ball game pieces
    def detect_game_balls(self, imgRaw):

        #Read HSV values from dictionary and make tuples
        hMin = int(FRCVisionLibrary.ball_values['HMIN'])
        hMax = int(FRCVisionLibrary.ball_values['HMAX'])
        sMin = int(FRCVisionLibrary.ball_values['SMIN'])
        sMax = int(FRCVisionLibrary.ball_values['SMAX'])
        vMin = int(FRCVisionLibrary.ball_values['VMIN'])
        vMax = int(FRCVisionLibrary.ball_values['VMAX'])
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

        if len(ballContours) == 2:
            ballContours = ballContours[0]
        elif len(ballContours) == 3:
            ballContours = ballContours[1]

        #Only proceed if at least one contour was found
        if len(ballContours) > 0:
 
            #largestContour = max(ballContours, key=cv.contourArea)
            for contour in ballContours:
            
                ((x, y), radius) = cv.minEnclosingCircle(contour)

                if radius > float(FRCVisionLibrary.ball_values['MINRADIUS']):

                    cv.circle(imgRaw, (int(x), int(y)), int(radius), (0, 255, 0), 2) #Draw a green circle around all balls detected

                if radius > targetRadius:

                    targetRadius = radius
                    targetX = x
                    targetY = y
                    foundBall = True

            #Distance and angle offset calculations
            if targetRadius > 0:
            
                inches_per_pixel = float(FRCVisionLibrary.ball_values['RADIUS'])/targetRadius #set up a general conversion factor
                distanceToBall = inches_per_pixel * (int(FRCVisionLibrary.camera_values['WIDTH']) / (2 * math.tan(math.radians(float(FRCVisionLibrary.camera_values['FOV'])))))
                offsetInInches = inches_per_pixel * (targetX - int(FRCVisionLibrary.camera_values['WIDTH']) / 2)
                angleToBall = math.degrees(math.atan((offsetInInches / distanceToBall)))
                screenPercent = math.pi * targetRadius * targetRadius / (int(FRCVisionLibrary.camera_values['WIDTH']) * int(FRCVisionLibrary.camera_values['HEIGHT']))
                ballOffset = -offsetInInches
          
            else:
            
                distanceToBall = -1
                angleToBall = -1

            cv.circle(imgRaw, (int(targetX), int(targetY)), int(targetRadius), (0, 0, 255), 2) #Draw a red circle around largest ball detected

        return targetX, targetY, targetRadius, distanceToBall, angleToBall, ballOffset, screenPercent, foundBall


    #Find vision target for goal
    def detect_goal_target(self, imgRaw):
        
        #Set constraints for detecting vision targets
        visionTargetWidth = float(FRCVisionLibrary.goal_values['TARGETWIDTH']) #inches
        visionTargetHeight = float(FRCVisionLibrary.goal_values['TARGETHEIGHT']) #inches
        minTargetArea = int(FRCVisionLibrary.goal_values['MINTARGETAREA']) #in square px, for individual pieces of tape, calculated for viewing from ~4ft
        minRegionArea = int(FRCVisionLibrary.goal_values['MINREGIONAREA']) #in square px, for paired pieces of tape, calculated for viewing from ~4ft

        #Define HSV range for cargo ship vision targets
        #values with light in Fab Lab
        hMin = int(FRCVisionLibrary.goal_values['HMIN'])
        hMax = int(FRCVisionLibrary.goal_values['HMAX'])
        sMin = int(FRCVisionLibrary.goal_values['SMIN'])
        sMax = int(FRCVisionLibrary.goal_values['SMAX'])
        vMin = int(FRCVisionLibrary.goal_values['VMIN'])
        vMax = int(FRCVisionLibrary.goal_values['VMAX'])
        visionTargetHSVMin = (hMin, sMin, vMin)
        visionTargetHSVMax = (hMax, sMax, vMax)

        #List to collect datapoints of all contours located
        #Append tuples in form (x, y, w, h, a)
        visionTargetValues = []

        #List to collect datapoints and area of all paired contours calculated
        #Append tuples in form (regionArea, x, y, w, h)
        visionRegionValues = []

        #Other processing values
        inchesPerPixel = -1
        diffTargets = -1
    
        #Values to be returned
        targetX = -1
        targetY = -1
        targetW = -1
        targetH = -1
        centerOffset = 0
        distanceToVisionTarget = 0
        angleToVisionTarget = 0
        foundVisionTarget = False

        #Find contours in mask
        visionTargetContours = self.process_image_contours(imgRaw, visionTargetHSVMin, visionTargetHSVMax)
    
        #only continue if contours are found
        if len(visionTargetContours) > 0:
        
            #Loop over all contours
            for testContour in visionTargetContours:

             #Get bounding rectangle dimensions
                x, y, w, h = cv.boundingRect(testContour)
                rect = cv.minAreaRect(testContour)
                a = rect[2]
                box = cv.boxPoints(rect)
                box = np.int0(box)
                   

                #If large enough, draw a rectangle and store the values in the list
                if cv.contourArea(testContour) > minTargetArea:

                    #this method will draw an angled box around the contour
                    cv.drawContours(imgRaw,[box],0,(0,0,255),2)     

                    visionTargetTuple = (x, y, w, h, a)
                    visionTargetValues.append(visionTargetTuple)

            #Only continue if two appropriately sized contours were found
            if len(visionTargetValues) > 1:

                #Sort the contours found into a left-to-right order (sorting by x-value)
                visionTargetValues.sort(key=itemgetter(0))

                #Compare each contour to the next-right-most contour to determine distance between them
                for i in range(len(visionTargetValues) - 1):

                    #Create a conversion factor between inches and pixels with a known value (the target height)
                    #and the height of the left-most contour found
                    inchesPerPixel = visionTargetHeight/visionTargetValues[i][3]
                
                    #Calculate the pixel difference between contours (right x - (left x + left width))
                    diffTargets = visionTargetValues[i + 1][0] - (visionTargetValues[i][0] + visionTargetValues[i][2])
                
                    #Check the distance against the expected angle with a tolerance, check the area, and store 
                    #the matched pairs in the indices list
                    rightAngleGood = visionTargetValues[i + 1][4] < -10 and visionTargetValues[i + 1][4] > -20
                    leftAngleGood = visionTargetValues[i][4] < -70 and visionTargetValues[i][4] > -80

                    if leftAngleGood and rightAngleGood:

                        #Calculate area of region found (height * (left width + right width + diffTargets))
                        regionHeight = visionTargetValues[i][3] #using left height
                        regionWidth = visionTargetValues[i][2] + visionTargetValues[i + 1][2] + diffTargets
                        regionArea = regionWidth * regionHeight

                        #Check area and draw rectangle (for testing)
                        if regionArea > minRegionArea:

                            x = visionTargetValues[i][0]
                            y = visionTargetValues[i][1]
                            w = regionWidth
                            h = regionHeight
                            cv.rectangle(imgRaw,(x,y),(x+w,y+h),(0,0,255),1) 
                        
                            visionRegionTuple = (regionArea, x, y, w, h)
                            visionRegionValues.append(visionRegionTuple)
                        
                #Only proceed if an appropriately sized merged region is found
                if len(visionRegionValues) > 0:

                    #Sort the collected paired regions from largest area to smallest area (largest area is index 0)
                    visionRegionValues.sort(key=itemgetter(0), reverse = True)

                    #Assign final values to be returned
                    targetX = visionRegionValues[0][1]
                    targetY = visionRegionValues[0][2]
                    targetW = visionRegionValues[0][3]
                    targetH = visionRegionValues[0][4]

                    foundVisionTarget = True
                                
                    distanceToVisionTarget = inchesPerPixel * (imgWidthVision / (2 * math.tan(math.radians(cameraFieldOfView))))
                    offsetInInches = inchesPerPixel * ((targetX + targetW/2) - imgWidthVision / 2)
                    angleToVisionTarget = math.degrees(math.atan((offsetInInches / distanceToVisionTarget)))
                    centerOffset = -offsetInInches

        #Return results
        return targetX, targetY, targetW, targetH, distanceToVisionTarget, angleToVisionTarget, centerOffset, foundVisionTarget


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
                if split_line[0].upper() == 'CAMERA:':
                    value_section = 'CAMERA'
                    new_section = True
                elif split_line[0].upper() == 'BALL:':
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
                    if value_section == 'CAMERA':
                        FRCVisionLibrary.camera_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'BALL':
                        FRCVisionLibrary.ball_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'GOALTARGET':
                        FRCVisionLibrary.goal_values[split_line[0].upper()] = split_line[1]
                    elif value_section == 'VISIONTAPE':
                        FRCVisionLibrary.tape_values[split_line[0].upper()] = split_line[1]
                    else:
                        new_section = True
        
        except FileNotFoundError:
            return False
        
        return True


    #Define class initialization
    def __init__(self, visionfile):
        
        #Read in vision settings file
        FRCVisionLibrary.visionFile = visionfile
        self.read_vision_file(FRCVisionLibrary.visionFile)
