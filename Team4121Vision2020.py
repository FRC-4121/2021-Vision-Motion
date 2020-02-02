#----------------------------------------------------------------------------------------------#
#                               North Canton Hoover High School                                #
#                                                                                              #
#                                Team 4121 - Norsemen Robotics                                 #
#                                                                                              #
#                               Vision & Motion Processing Code                                #
#----------------------------------------------------------------------------------------------#
#                                                                                              #
#  This code continuously analyzes images from one or more USB cameras to identify on field    #
#  game pieces and vision targets.  For game pieces, the code will identify all game pieces    #
#  within the camera's field of view and determine the closest one.  The distance and angle    #
#  to the closest game piece is calculated and made available to the main robot code through   #
#  network tables.  The closest game piece is highlighted with a green box while all other     #
#  found game pieces are highlighted with a red box.  The annotated video is streamed to the   #
#  driver station for display.  The annotated video is also saved to a file for post game      #
#  review and analysis.  For vision targets, the code will identify all vision targets and     #
#  calculate the angle and distance to each one.  Vision target information is made available  #
#  to the main robot code through network tables.                                              #
#                                                                                              #
#  This code also continuously interrogates a VMX-Pi board to determine linear and angular     #
#  motion in all three axes.  This information is made available to the main robot code        #
#  through network tables.                                                                     #
#                                                                                              #
#----------------------------------------------------------------------------------------------#
#                                                                                              #
#  Authors:  Jonas Muhlenkamp                                                                  #
#            Ricky Park                                                                        #
#            Tresor Nshimiye                                                                   #
#            Tim Fuller - Mentor                                                               #
#                                                                                              #
#  Creation Date: 3/1/2018                                                                     #
#                                                                                              #
#  Revision: 3.0                                                                               #
#                                                                                              #
#  Revision Date: 2/18/2019                                                                    #
#                                                                                              #
#----------------------------------------------------------------------------------------------#

#!/usr/bin/env python3

#System imports
import sys
import imp

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/usr/local/lib/vmxpi/')

#Module imports
import cv2 as cv
import numpy as np
import datetime
import time
import logging
import argparse
from operator import itemgetter
import math
import cscore as cs
from cscore import CameraServer
from networktables import NetworkTables
from time import sleep

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Initialize operating constants
imgWidthVision = 320  
imgHeightVision = 240
imgWidthDriver = 160
imgHeightDriver = 120
#cameraFieldOfView = 23.5 #value designated for vision tape
cameraFieldOfView = 27.3


#Define program control flags
writeVideo = True
sendVisionToDashboard = True

#Define image processing method
def process_image(imgRaw, hsvMin, hsvMax):
    
    #Blur image to remove noise
    blur = cv.GaussianBlur(imgRaw.copy(),(13,13),0)
        
    #Convert from BGR to HSV colorspace
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Set pixels to white if in target HSV range, else set to black
    mask = cv.inRange(hsv, hsvMin, hsvMax)

    #Find contours in mask
    _, contours, _ = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    
    return contours


#Define processing class
def detect_ball_target(imgRaw):

    #Define constraints for ball detection
    ballRadius = 3.5 #in inches
    minRadius = 5 #in pixels, this can be tweaked as needed

    #Define the lower and upper boundaries of the "green"
    #ball in the HSV color space
    ballHSVMin = (20, 77, 116)
    ballHSVMax = (77, 255, 255)
   
    #Values to be returned
    targetRadius = 0 #px
    targetX = -1 #px
    targetY = -1 #px
    distanceToBall = 0 #inches
    angleToBall = 0 #degrees
    ballOffset = 0
    screenPercent = 0
    foundBall = False;

    #List of ball x, y, radius.  Append in form (x, y, radius)
    ballValues = []

    #Find contours in the mask and clean up the return style from OpenCV
    ballContours = process_image(imgRaw, ballHSVMin, ballHSVMax)

    if len(ballContours) == 2:
        ballContours = ballContours[0]
    elif len(ballContours) == 3:
        ballContours = ballContours[1]

    #Only proceed if at least one contour was found
    if len(ballContours) > 0:
 
        #largestContour = max(ballContours, key=cv.contourArea)
        for contour in ballContours:
            
            ((x, y), radius) = cv.minEnclosingCircle(contour)

            if radius > minRadius:

                #ballTuple = (x, y, radius)
                #ballValues.append(ballTuple)
                
                if radius > targetRadius:

                    targetRadius = radius
                    targetX = x
                    targetY = y
                    foundBall = True

        #Distance and angle offset calculations
        if targetRadius > 0:
            
            inches_per_pixel = ballRadius/targetRadius #set up a general conversion factor
            distanceToBall = inches_per_pixel * (imgWidthVision / (2 * math.tan(math.radians(cameraFieldOfView))))
            offsetInInches = inches_per_pixel * (targetX - imgWidthVision / 2)
            angleToBall = math.degrees(math.atan((offsetInInches / distanceToBall)))
            screenPercent = math.pi * targetRadius * targetRadius / (imgWidthVision * imgHeightVision)
            ballOffset = -offsetInInches
          
        else:
            
            distanceToBall = -1
            angleToBall = -1

        for ball in ballValues:

            cv.circle(imgRaw, (int(ball[0]), int(ball[1])), int(ball[2]), (0, 255, 0), 2)

        #cv.circle(imgRaw, (int(targetX), int(targetY)), int(targetRadius), (0, 0, 255), 2) #Draw a red circle around largest ball detected

    return targetX, targetY, targetRadius, distanceToBall, angleToBall, ballOffset, screenPercent, foundBall

    
#Define general tape detection method (rectangle good for generic vision tape targets)
def detect_tape_rectangle(imgRaw):
    
    #Define constraints for detecting floor tape
    tapeWidth = 39.25 #in inches
    tapeHeight = 17.0 #in inches
    minTapeArea = 50 #in square px, can be tweaked if needed

    #Define HSV range for white alignment tape
    tapeHSVMin = (60, 90, 80)
    tapeHSVMax = (115, 255, 255)

    #Values to be returned
    targetX = -1
    targetY = -1
    targetW = -1
    targetH = -1
    centerOffset = 0
    distanceToTape = 0
    horizAngleToTape = 0
    vertAngleToTape = 0
    foundTape = False
    
    #Find alignment tape in image
    tapeContours = process_image(imgRaw, tapeHSVMin, tapeHSVMax)
  
    #Continue with processing if alignment tape found
    if len(tapeContours) > 0:

        #find the largest contour and check it against the mininum tape area
        #largestContour = max(tapeContours, key=cv.contourArea)
        largestContour = tapeContours[0]

        for contour in tapeContours:
            
            if cv.contourArea(contour) > minTapeArea:
                
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(imgRaw,(x,y),(x+w,y+h),(0,0,255),2) #floor tape

                if cv.contourArea(contour) > cv.contourArea(largestContour):
                    largestContour = contour
  
        targetX, targetY, targetW, targetH = cv.boundingRect(largestContour)
        foundTape = True

        #calculate real world values of tape
        inchesPerPixel = float(tapeHeight) / targetH
        distanceToTape = inchesPerPixel * (imgWidthVision / (2 * math.tan(math.radians(cameraFieldOfView))))
        horizOffsetInInches = inchesPerPixel * ((targetX + targetW/2) - imgWidthVision / 2)
        horizAngleToTape = math.degrees(math.atan((horizOffsetInInches / distanceToTape)))
        vertOffsetInInches = inchesPerPixel * ((imgHeightVision / 2) - (targetY - targetH/2))
        vertAngleToTape = math.degrees(math.atan((vertOffsetInInches / distanceToTape)))
        centerOffset = -horizOffsetInInches

    return targetX, targetY, targetW, targetH, centerOffset, distanceToTape, horizAngleToTape, vertAngleToTape, foundTape


#Define contour detector function
def detect_vision_targets(imgRaw):

    #Set constraints for detecting vision targets
    visionTargetWidth = 3.313 #in inches
    visionTargetHeight = 5.826 #in inches
    minTargetArea = 300 #in square px, for individual pieces of tape, calculated for viewing from ~4ft
    minRegionArea = 2000 #in square px, for paired pieces of tape, calculated for viewing from ~4ft

    #Define HSV range for cargo ship vision targets
    #values with light in Fab Lab
    visionTargetHSVMin = (59, 0, 78)
    visionTargetHSVMax = (114, 255, 255)

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
    visionTargetContours = process_image(imgRaw, visionTargetHSVMin, visionTargetHSVMax)
    
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


#Define main processing function
def main():

    #Define global variables
    global imgWidthDriver
    global imgHeightDriver
    global imgWidthVision
    global imgHeightVision
    global writeVideo

    #Define camera configuration variables
    driverCameraBrightness = 1
    visionCameraBrightness = 25
    driverFramesPerSecond = 15
    visionFramesPerSecond = 15
    visionCameraExposure = 40
   
    #Define local flags
    networkTablesConnected = False
    driverCameraConnected = False
    visionCameraConnected = False
    foundBall = False
    foundTape = False
    foundVisionTarget = False

    #Get current time as a string
    currentTime = time.localtime(time.time())
    timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

    #Open a log file
    logFilename = '/data/Logs/Run_Log_' + timeString + '.txt'
    log_file = open(logFilename, 'w')
    log_file.write('run started on %s.\n' % datetime.datetime.now())
    log_file.write('')

##    #Load VMX module
##    vmxpi = imp.load_source('vmxpi_hal_python', '/usr/local/lib/vmxpi/vmxpi_hal_python.py')
##    vmx = vmxpi.VMXPi(False,50)
##    if vmx.IsOpen() is False:
##        log_file.write('Error:  Unable to open VMX Client.\n')
##        log_file.write('\n')
##        log_file.write('        - Is pigpio (or the system resources it requires) in use by another process?\n')
##        log_file.write('        - Does this application have root privileges?')
##        log_file.close()
##        sys.exit(0)

    #Connect NetworkTables
    try:
        NetworkTables.initialize(server='10.41.21.2')
        visionTable = NetworkTables.getTable("vision")
        navxTable = NetworkTables.getTable("navx")
        smartDash = NetworkTables.getTable("SmartDashboard")
        networkTablesConnected = True
        log_file.write('Connected to Networktables on 10.41.21.2 \n')
    except:
        log_file.write('Error:  Unable to connect to Network tables.\n')
        log_file.write('Error message: ', sys.exec_info()[0])
        log_file.write('\n')

##    #Navx configuration
##    navxTable.putNumber("ZeroGyro", 0)
##    #navxTable.putNumber("ZeroDisplace", 0)
##
##    #Reset yaw gyro
##    vmx.getAHRS().Reset()
##    vmx.getAHRS().ZeroYaw()
##
##    #Reset displacement
##    vmx.getAHRS().ResetDisplacement()

    #Set up a camera server
    camserv = CameraServer.getInstance()
    camserv.enableLogging

    #Start capturing webcam videos
##    try:
##        #startAutomaticCapture is enough to stream a non-annotated video
##        driverCameraPath = '/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.5:1.0-video-index0'
##        driverCamera = camserv.startAutomaticCapture(name = "DriverCamera", path=driverCameraPath)
##        driverCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, imgWidthDriver, imgHeightDriver, driverFramesPerSecond)
##        driverCamera.setBrightness(driverCameraBrightness)
##        driverCameraConnected = True
##        log_file.write('Connected to driver camera on Path = %s.\n' % driverCameraPath)
##    except:
##        log_file.write('Error:  Unable to connect to driver camera.\n')
##        log_file.write('Error message: ', sys.exec_info()[0])
##        log_file.write('\n')

    try:
        visionCameraPath = '/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0'
        visionCamera = camserv.startAutomaticCapture(name="VisionCamera", path=visionCameraPath)
        visionCamera.setBrightness(visionCameraBrightness)
        visionCamera.setResolution(imgWidthVision,imgHeightVision)
        visionCamera.setFPS(visionFramesPerSecond)
        visionCamera.setExposureManual(visionCameraExposure)
        visionCameraConnected = True
        log_file.write('Connected to vision camera on Path = %s.\n' % visionCameraPath)
    except:
        log_file.write('Error:  Unable to connect to vision camera.\n')
        log_file.write('Error message: ', sys.exec_info()[0])
        log_file.write('\n')        

    #Define vision video sink
    if visionCameraConnected == True:
        visionSink = camserv.getVideo(name="VisionCamera")
  
    #Define output stream for processed vision images
    if (visionCameraConnected == True):
        visionOutputStream = camserv.putVideo("VisionStream", imgWidthVision, imgHeightVision)

    #Set video codec and create VideoWriter
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    videoFilename = '/data/Match_Videos/RobotVisionCam-' + timeString + '.avi'
    visionImageOut = cv.VideoWriter(videoFilename,fourcc,visionFramesPerSecond,(imgWidthVision,imgHeightVision))

    #Create blank vision image
    imgVision= np.zeros(shape=(imgWidthVision, imgHeightVision, 3), dtype=np.uint8)

    #Start main processing loop
    while (True):

        #Read in an image from a file (for testing)
        #img = cv.imread('RetroreflectiveTapeImages2019/CargoStraightDark90in.jpg')
        #if img is None:
        #    break

        #Initialize video time stamp
        visionVideoTimestamp = 0
        
        #Grab frames from the vision web camera
        if visionCameraConnected == True:
            visionVideoTimestamp, imgVision = visionSink.grabFrame(imgVision)

        #Check for frame errors
        visionFrameGood = True
        if (visionVideoTimestamp == 0):
            log_file.write('Vision video error: \n')
            log_file.write(visionSink.getError())
            log_file.write('\n')
            visionFrameGood = False
            #sleep (float(visionFramesPerSecond * 2) / 1000.0)
            continue

        #Continue processing if we have no errors
        if (visionFrameGood == True):

            #Call detection methods
            ballX, ballY, ballRadius, ballDistance, ballAngle, ballOffset, ballScreenPercent, foundBall = detect_ball_target(imgVision)
##            #tapeX, tapeY, tapeW, tapeH, tapeOffset, tapeDistance, tapeHAngle, tapeVAngle, foundTape = detect_tape_rectangle(imgVision)
##            #visionTargetX, visionTargetY, visionTargetW, visionTargetH, visionTargetDistance, visionTargetAngle, visionTargetOffset, foundVisionTarget = detect_vision_targets(imgVision)
##
##            #Update networktables and log file
##            if networkTablesConnected == True:
##
##                visionTable.putNumber("RobotStop", 0)
##                visionTable.putBoolean("WriteVideo", writeVideo)
##
##                visionTable.putNumber("BallX", round(ballX, 2))
##                visionTable.putNumber("BallY", round(ballY, 2))
##                visionTable.putNumber("BallRadius", round(ballRadius, 2))
##                visionTable.putNumber("BallDistance", round(ballDistance, 2))
##                visionTable.putNumber("BallAngle", round(ballAngle, 2))
##                visionTable.putNumber("BallOffset", round(ballOffset, 2))
##                visionTable.putNumber("BallScreenPercent", round(ballScreenPercent, 2))
##                visionTable.putBoolean("FoundBall", foundBall)
##                
####                if foundBall == True:
####                    
####                    log_file.write('Cargo found at %s.\n' % datetime.datetime.now())
####                    log_file.write('  Ball distance: %.2f \n' % round(ballDistance, 2))
####                    log_file.write('  Ball angle: %.2f \n' % round(ballAngle, 2))
####                    log_file.write('  Ball offset: %.2f \n' % round(ballOffset, 2))
####                    log_file.write('\n')
####
####                if foundTape == True:
####                    visionTable.putNumber("TapeX", round(tapeX, 2))
####                    visionTable.putNumber("TapeY", round(tapeY, 2))
####                    visionTable.putNumber("TapeW", round(tapeW, 2))
####                    visionTable.putNumber("TapeH", round(tapeH, 2))
####                    visionTable.putNumber("TapeOffset", round(tapeOffset, 2))
####                    visionTable.putBoolean("FoundTape", foundTape)
####                    log_file.write('Floor tape found at %s.\n' % datetime.datetime.now())
####                    log_file.write('  Tape offset: %.2f \n' % round(tapeOffset, 2))
####                    log_file.write('\n')
####
####
####                visionTable.putNumber("VisionTargetX", round(visionTargetX, 2))
####                visionTable.putNumber("VisionTargetY", round(visionTargetY, 2))
####                visionTable.putNumber("VisionTargetW", round(visionTargetW, 2))
####                visionTable.putNumber("VisionTargetH", round(visionTargetH, 2))
####                visionTable.putNumber("VisionTargetDistance", round(visionTargetDistance, 2))
####                visionTable.putNumber("VisionTargetAngle", round(visionTargetAngle, 2))
####                visionTable.putNumber("VisionTargetOffset", round(visionTargetOffset, 2))
####                visionTable.putBoolean("FoundVisionTarget", foundVisionTarget)
####
####                if foundVisionTarget == True:
####                    
####                    log_file.write('Vision target found at %s.\n' % datetime.datetime.now())
####                    log_file.write('  Vision target distance: %.2f \n' % round(visionTargetDistance, 2))
####                    log_file.write('  Vision target angle: %.2f \n' % round(visionTargetAngle, 2))
####                    log_file.write('  Vision target offset: %.2f \n' % round(visionTargetOffset, 2))
####                    log_file.write('\n')
##
##            #Draw various contours on the image
            if foundBall == True:
                cv.circle(imgVision, (int(ballX), int(ballY)), int(ballRadius), (0, 0, 255), 2) #ball
                cv.putText(imgVision, 'Distance to Ball: %.2f' %ballDistance, (10, 15), cv.FONT_HERSHEY_SIMPLEX, .5,(255, 0, 0), 2)
                cv.putText(imgVision, 'Angle to Ball: %.2f' %ballAngle, (10, 30), cv.FONT_HERSHEY_SIMPLEX, .5,(255, 0, 0), 2)                
##            #if foundTape == True:
##                #cv.rectangle(imgVision,(tapeX,tapeY),(tapeX+tapeW,tapeY+tapeH),(0,255,0),2) #floor tape
##                #cv.putText(imgVision, 'Distance to Tape: %.2f' %tapeDistance, (320, 400), cv.FONT_HERSHEY_SIMPLEX, .75,(0, 0, 255), 2)
##                #cv.putText(imgVision, 'Horiz. Angle to Tape: %.2f' %tapeHAngle, (320, 430), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
##                #cv.putText(imgVision, 'Vert. Angle to Tape: %.2f' %tapeVAngle, (320, 460), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
##            #if foundVisionTarget == True:
##                #cv.rectangle(imgVision,(visionTargetX,visionTargetY),(visionTargetX+visionTargetW,visionTargetY+visionTargetH),(0,255,0),2) #vision targets
##                #cv.putText(imgVision, 'Distance to Vision: %.2f' %visionTargetDistance, (10, 400), cv.FONT_HERSHEY_SIMPLEX, .75,(0, 255, 0), 2)
##                #cv.putText(imgVision, 'Angle to Vision: %.2f' %visionTargetAngle, (10, 440), cv.FONT_HERSHEY_SIMPLEX, .75,(0, 255, 0), 2)
##
##            #Put timestamp on image
##            cv.putText(imgVision, str(datetime.datetime.now()), (10, 30), cv.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 2)
##
##        #Update navx network table
####        if networkTablesConnected == True:
####            navxTable.putNumber("GyroAngle", round(vmx.getAHRS().GetAngle(), 2))
####            navxTable.putNumber("GyroYaw", round(vmx.getAHRS().GetYaw(), 2))
####            navxTable.putNumber("GyroPitch", round(vmx.getAHRS().GetPitch(), 2))
####            navxTable.putNumber("YVelocity", round(vmx.getAHRS().GetVelocityY(), 4))
####            navxTable.putNumber("XVelocity", round(vmx.getAHRS().GetVelocityX(), 4))
####            navxTable.putNumber("YDisplacement", round(vmx.getAHRS().GetDisplacementY(), 4))
####            navxTable.putNumber("XDisplacement", round(vmx.getAHRS().GetDisplacementX(), 4))
####            navxTable.putNumber("YVelocity", round(vmx.getAHRS().GetVelocityY(), 4))
####            navxTable.putNumber("XVelocity", round(vmx.getAHRS().GetVelocityX(), 4))
####            navxTable.putNumber("YAccel", round(vmx.getAHRS().GetWorldLinearAccelY(), 4))
####            navxTable.putNumber("XAccel", round(vmx.getAHRS().GetWorldLinearAccelX(), 4))
####
##        #Check vision network table dashboard value
##        sendVisionToDashboard = visionTable.getBoolean("SendVision", False)
##
##        #Send vision to dashboard (for testing)
##        if (visionCameraConnected == True) and (sendVisionToDashboard == True):
##            visionOutputStream.putFrame(imgVision)
##            
        #Write processed image to file
        #visionImageOut.write(imgVision)

        #Display the vision camera stream (for testing only)
        cv.imshow("Vision", imgVision)

##        #Check for gyro re-zero
##        gyroInit = navxTable.getNumber("ZeroGyro", 0)
##        if gyroInit == 1:
##            vmx.getAHRS().Reset()
##            vmx.getAHRS().ZeroYaw()
##            navxTable.putNumber("ZeroGyro", 0)

        #Check for displacement zero
        #dispInit = navxTable.getNumber("ZeroDisplace", 0)
        #if dispInit == 1:
        #    vmx.getAHRS().ResetDisplacement()
        #    navxTable.putNumber("ZeroDisplace", 0)
        
        #Check for stop code from robot or keyboard (for testing)
        if cv.waitKey(1) == 27:
            break
##        robotStop = visionTable.getNumber("RobotStop", 0)
##        if (robotStop == 1) or (visionCameraConnected == False) or (networkTablesConnected == False):
##            break


    #Close all open windows (for testing)
    cv.destroyAllWindows()

    #Close video file
    visionImageOut.release()
    visionOutputStream = None
    visionSink = None

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    log_file.close()
    

#define main function
if __name__ == '__main__':
    main()
