#----------------------------------------------------------------------------------------------#
#                               North Canton Hoover High School                                #
#                                                                                              #
#                                Team 4121 - Norsemen Robotics                                 #
#                                                                                            #
#                               Vision & Motion Processing Code                                #
#------------------------------cd ----------------------------------------------------------------#
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
#  Revision: 4.0                                                                               #
#                                                                                              #
#  Revision Date: 2/2/2020                                                                     #
#                                                                                              #
#----------------------------------------------------------------------------------------------#

#!/usr/bin/env python3

#System imports
import sys
import imp

#Setup paths
sys.path.append('/home/pi/.local/lib/python3.5/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')
sys.path.append('/usr/local/lib/vmxpi/')
#sys.path.append('C:\\Users\\timfu\\Documents\\Team4121\\Libraries')

#Module imports
import cv2 as cv
import numpy as np
import datetime
import time
import logging
import argparse
from operator import itemgetter
import math
from networktables import NetworkTables
from time import sleep

#Team 4121 module imports
from FRCVisionLibrary import VisionLibrary
from FRCCameraLibrary import FRCWebCam
from FRCNavxLibrary import FRCNavx

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Declare global variables
cameraFile = '/home/pi/Team4121/Config/2020CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2020VisionSettings.txt'
cameraValues={}

#Define program control flags
videoTesting = True

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

        print('Using default camera values')
        
        cameraValues['BallCamFOV'] = 22.5
        cameraValues['BallCamWidth'] = 960
        cameraValues['BallCamHeight'] = 720
        cameraValues['BallCamFPS'] = 15
        cameraValues['BallCamBrightness'] = 0.30
        cameraValues['BallCamExposure'] = 30
        cameraValues['BallCamCalFactor'] = 1
        cameraValues['BallCamfocalLength'] = 334.29
        cameraValues['BallCamMountAngle'] = 0
        cameraValues['BallCamMountHeight'] = 0
        cameraValues['GoalCamFOV'] = 23.5
        cameraValues['GoalCamWidth'] = 320
        cameraValues['GoalCamHeight'] = 240
        cameraValues['GoalCamFPS'] = 15
        cameraValues['GoalCamBrightness'] = 0
        cameraValues['GoalCamExposure'] = 0
        cameraValues['GoalCamCalFactor'] = 1
        cameraValues['GoalCamFocalLength'] = 340.0
        cameraValues['GoalCamMountAngle'] = 25.0
        cameraValues['GoalCamMountHeight'] = 26.0


#Define ball layout detection function
def determineBallPattern(method, ballX, ballDistance, ballAngle):

    #Initialize return value
    ballPattern = -1
    ballPatternName = 'none'

    #Run selected method
    if method == 1:

        ballWidthPercent = (ballX / float(cameraValues['BallCamWidth']))

        #Blue 1 
        if (ballWidthPercent > 0.9) and (ballWidthPercent < 0.95):
            ballPattern = 1
            ballPatternName = "blue1"
        #Red1
        elif (ballWidthPercent > 0.4) and (ballWidthPercent < 0.6):
            ballPattern = 2
            ballPatternName = "red1"
        #Blue 2
        elif (ballWidthPercent > 0.68) and (ballWidthPercent < 0.72):
            ballPattern = 3
            ballPatternName = "blue2"
        #Red 2
        elif (ballWidthPercent > 0) and (ballWidthPercent < 0.05):
            ballPattern = 4
            ballPatternName = "red2"

    else:

        #Blue 1
        if ((ballDistance > 0) and (ballDistance < 10)) and ((ballAngle > 0) and (ballAngle < 10)):
            ballPattern = 1
            ballPatternName = 'blue1'
 
    #Return values
    return ballPattern, ballPatternName


#Define main processing function
def main():

    #Define global variables
    global writeVideo
   
    #Define local flags
    networkTablesConnected = False
    ballCamConnected = False
    goalCamConnected = False
    ballsFound = 0
    markersFound = 0
    foundTape = False
    foundVisionTarget = False
    tapeTargetLock = False

    #Create Navx object
    #navx = FRCNavx('NavxStream')
    #navx.start_navx()

    #Get current time as a string
    currentTime = time.localtime(time.time())
    timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

    #Open a log file
    logFilename = '/home/pi/Team4121/Logs/Run_Log_' + timeString + '.txt'
    log_file = open(logFilename, 'w')
    log_file.write('run started on %s.\n' % datetime.datetime.now())
    log_file.write('')

    #Read camera settings file
    read_settings_file()

    #Connect NetworkTables
    try:
        NetworkTables.initialize(server='10.41.21.2')
        visionTable = NetworkTables.getTable("vision")
        navxTable = NetworkTables.getTable("navx")
        networkTablesConnected = True
        log_file.write('Connected to Networktables on 10.41.21.2 \n')

        visionTable.putNumber("RobotStop", 0)
    except:
        log_file.write('Error:  Unable to connect to Network tables.\n')
        log_file.write('Error message: ', sys.exc_info()[0])
        log_file.write('\n')

    #Create ball camera stream
    ballCamSettings = {}
    ballCamSettings['Width'] = int(cameraValues['BallCamWidth'])
    ballCamSettings['Height'] = int(cameraValues['BallCamHeight'])
    ballCamSettings['Brightness'] = cameraValues['BallCamBrightness']
    ballCamSettings['Exposure'] = cameraValues['BallCamExposure']
    ballCamSettings['FPS'] = cameraValues['BallCamFPS']
    ballCamera = FRCWebCam('/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0', "BallCam", ballCamSettings)

    #Create goal camera stream
##    goalCamSettings = {}
##    goalCamSettings['Width'] = cameraValues['GoalCamWidth']
##    goalCamSettings['Height'] = cameraValues['GoalCamHeight']
##    goalCamSettings['Brightness'] = cameraValues['GoalCamBrightness']
##    goalCamSettings['Exposure'] = cameraValues['GoalCamExposure']
##    goalCamSettings['FPS'] = cameraValues['GoalCamFPS']
##    goalCamera = FRCWebCam('/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0', "GoalCam", goalCamSettings)

    #Create vision processing
    visionProcessor = VisionLibrary(visionFile)

    #Create blank vision image
    imgBallRaw = np.zeros(shape=(int(cameraValues['BallCamWidth']), int(cameraValues['BallCamHeight']), 3), dtype=np.uint8)
    #imgGoalRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)
    imgBlankRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)

    #Initialize gyro angle
    gyroAngle = 0
    
    #Start main processing loop
    while (True):

        #Read frames from cameras
        imgBallRaw = ballCamera.read_frame()
#        imgGoalRaw = goalCamera.read_frame()
        imgBlankRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)

        #Read gyro angle
        #gyroAngle = navx.read_angle()
        
        #Call detection methods
        ballsFound, ballData = visionProcessor.detect_game_balls(imgBallRaw, int(cameraValues['BallCamWidth']),int(cameraValues['BallCamHeight']),float(cameraValues['BallCamFOV']))
        markersFound, markerData = visionProcessor.detect_field_marker(imgBallRaw, int(cameraValues['BallCamWidth']),int(cameraValues['BallCamHeight']),float(cameraValues['BallCamFOV']))
##        tapeCameraValues, tapeRealWorldValues, foundTape, tapeTargetLock, rect, box = visionProcessor.detect_tape_rectangle(imgGoalRaw, int(cameraValues['GoalCamWidth']),
##                                                                                                                        int(cameraValues['GoalCamHeight']),
##                                                                                                                        float(cameraValues['GoalCamFOV']),
##                                                                                                                        float(cameraValues['GoalCamFocalLength']),
##                                                                                                                        float(cameraValues['GoalCamMountAngle']),
##                                                                                                                        float(cameraValues['GoalCamMountHeight']))

        #cv.putText(imgBallRaw, 'Balls found: %.2f' %ballsFound, (10, 110), cv.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)
        #cv.putText(imgBallRaw, 'Markers found: %.2f' %markersFound, (10, 140), cv.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255), 2)

        #Define ball variables for use later
        ballPatternNumber = 0
        ballPatternName = ""
       
        #Draw ball contours and target data on the image
        if ballsFound > 0:

            #Detect ball pattern
            #ballPatternNumber, ballPatternName = determineBallPattern(1, ballData[0]['x'], ballData[0]['distance'], ballData[0]['angle'])

            #Copy raw image
            imgBallNew = imgBallRaw

            #Loop over all contours and annotate image
            i = 0
            for ball in ballData:

                if i < 3:

                    if i == 0:
                        cv.circle(imgBallNew, (int(ballData[i]['x']), int(ballData[i]['y'])), int(ballData[i]['radius']), (0, 0, 255), 2)
                        cv.putText(imgBallNew, 'Distance to Ball: %.2f' %ballData[i]['distance'], (10, 15), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                        cv.putText(imgBallNew, 'Angle to Ball: %.2f' %ballData[i]['angle'], (10, 30), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                        cv.putText(imgBallNew, 'Radius: %.2f' %ballData[i]['radius'], (10, 45), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                    else:
                        cv.circle(imgBallNew, (int(ballData[i]['x']), int(ballData[i]['y'])), int(ballData[i]['radius']), (0, 255, 0), 2)

                    ballDistance = ballData[i]['distance']
                    ballAngle = ballData[i]['angle']
                    ballScreenPercent = ballData[i]['percent']
                    ballOffset = ballData[i]['offset']


                    if networkTablesConnected == True:
                        visionTable.putNumber("BallDistance" + str(i), ballDistance)
                        visionTable.putNumber("BallAngle" + str(i), ballAngle)
                        visionTable.putNumber("BallScreenPercent" + str(i), ballScreenPercent)
                        visionTable.putNumber("BallOffset" + str(i), ballOffset)

                else:

                    cv.circle(imgBallNew, (int(ballData[i]['x']), int(ballData[i]['y'])), int(ballData[i]['radius']), (0, 255, 0), 2)

                i += 1

        #Define marker variables for use later
        markerDistance = 0
        markerAngle = 0
        markerScreenPercent = 0
        
        #Draw vision markers
        if markersFound > 0:

            #Copy raw image
            imgBallNew = imgBallRaw

            #Loop over all contours and annotate image
            i = 0
            for marker in markerData:

                if i == 0:

                    cv.rectangle(imgBallNew, (int(markerData[i]['x']), int(markerData[i]['y'])), (int(markerData[i]['w']) + int(markerData[i]['x']), int(markerData[i]['y']) + int(markerData[i]['h'])), (0, 0, 255), 2)
                    cv.putText(imgBallNew, 'Distance to Marker: %.2f' %markerData[i]['distance'], (10, 60), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                    cv.putText(imgBallNew, 'Angle to Marker: %.2f' %markerData[i]['angle'], (10, 75), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                    
                else:

                    cv.rectangle(imgBallNew, (int(markerData[i]['x']), int(markerData[i]['y'])), (int(markerData[i]['w']) + int(markerData[i]['x']), int(markerData[i]['y']) + int(markerData[i]['h'])), (0, 255, 0), 2)

                i += 1
                
        #Draw vision tape contours and target data on the image
        # if foundTape == True:
        #     imgGoalNew = imgGoalRaw
        #     if tapeTargetLock:
        #         cv.rectangle(imgGoalNew,(tapeCameraValues['TargetX'],tapeCameraValues['TargetY']),(tapeCameraValues['TargetX']+tapeCameraValues['TargetW'],tapeCameraValues['TargetY']+tapeCameraValues['TargetH']),(0,255,0),2) #vision tape
        #         cv.drawContours(imgGoalNew, [box], 0, (0,0,255), 2)
            
        #     cv.putText(imgBlankRaw, 'Tape Distance (A): %.2f' %tapeRealWorldValues['StraightDistance'], (10, 30), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Tape Distance (S): %.2f' %tapeRealWorldValues['TapeDistance'], (10, 50), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Wall Distance: %.2f' %tapeRealWorldValues['WallDistance'], (10, 70), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Bot Angle: %.2f' %tapeRealWorldValues['BotAngle'], (10, 90), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'IPP: %.2f' %tapeCameraValues['IPP'], (10, 130), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Vert Offset: %.2f' %tapeRealWorldValues['VertOffset'], (10, 150), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Offset: %.2f' %tapeCameraValues['Offset'], (10, 170), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Target Width: %.2f' %tapeCameraValues['TargetW'], (10, 190), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
        #     cv.putText(imgBlankRaw, 'Apparent Width: %.2f' %tapeRealWorldValues['ApparentWidth'], (10, 210), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)


        #Put values in NetworkTables
        if networkTablesConnected == True:

            #navxTable.putNumber("GyroAngle", gyroAngle)

            visionTable.putBoolean("FoundTape", foundTape)

            if foundTape == True:
                visionTable.putBoolean("TargetLock", tapeTargetLock)
                visionTable.putNumber("TapeDistance", tapeRealWorldValues['TapeDistance'])
                visionTable.putNumber("TapeOffset", tapeCameraValues['Offset'])

            visionTable.putBoolean("FoundBall", bool(ballsFound > 0))
            
            if ballsFound > 0:
                visionTable.putNumber("BallLayoutNum", ballPatternNumber)
                visionTable.putString("BallLayoutName", ballPatternName)

            if markersFound > 0:
                visionTable.putNumber("MarkerDistance", markerDistance)
                visionTable.putNumber("MarkerAngle", markerAngle)
                visionTable.putNumber("MarkerScreenPercent", markerScreenPercent)

        #Display the vision camera stream (for testing only)
        if videoTesting == True:
            cv.imshow("Ball", imgBallRaw)
#            cv.imshow("Goal", imgGoalNew)
#            cv.imshow("Data", imgBlankRaw)

        #Check for gyro re-zero
        # gyroInit = navxTable.getNumber("ZeroGyro", 0)
        # if gyroInit == 1:
        #     navx.reset_gyro()
        #     navxTable.putNumber("ZeroGyro", 0)
        
        #Check for stop code from robot or keyboard (for testing)
        if videoTesting == True:
            if cv.waitKey(1) == 27:
                break
            
        robotStop = visionTable.getNumber("RobotStop", 0)
        if (robotStop == 1) or (networkTablesConnected == False):
            break

        #Pause before next analysis
        #time.sleep(0.066) #should give ~15 FPS

    #Close all open windows (for testing)
    if videoTesting == True:
        cv.destroyAllWindows()

    #Release camera resources
    ballCamera.release_cam()
#    goalCamera.release_cam()

    #Release Navx resource
    #navx.stop_navx()
    
    #Close the log file
    #        navx.reset_gyro()
    #        navxTable.put
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    log_file.close()


#define main function
if __name__ == '__main__':
    main()
