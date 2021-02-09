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
#  Revision: 5.0                                                                               #
#                                                                                              #
#  Revision Date: 2/8/2021                                                                     #
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
cameraFile = '/home/pi/Team4121/Config/2021CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2021VisionSettings.txt'
videoDirectory = '/home/pi/Team4121/Videos'
cameraValues={}

#Define program control flags
useNavx = False
useFieldCam = True
useGoalCam = False
findBalls = True
findMarkers = True
findGoal = False
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
        
        cameraValues['FieldCamFOV'] = 22.5
        cameraValues['FieldCamWidth'] = 960
        cameraValues['FieldCamHeight'] = 720
        cameraValues['FieldCamFPS'] = 15
        cameraValues['FieldCamBrightness'] = 0.30
        cameraValues['FieldCamExposure'] = 30
        cameraValues['FieldCamCalFactor'] = 1
        cameraValues['FieldCamfocalLength'] = 334.29
        cameraValues['FieldCamMountAngle'] = 0
        cameraValues['FieldCamMountHeight'] = 0
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

    #Define flags
    networkTablesConnected = False
    ballCamConnected = False
    goalCamConnected = False
    foundTape = False
    foundVisionTarget = False
    tapeTargetLock = False

    #Define variables
    gyroAngle = 0
    ballsFound = 0
    markersFound = 0
    ballData = []
    markerData = []
    currentTime = []
    currentDate = []
    timeString = ''

    #Define objects
    navx = object
    visionTable = object
    navxTable = object
    fieldCamera = object
    goalCamera = object

    #Create Navx object
    if useNavx == True:
        navx = FRCNavx('NavxStream')

    #Get current time as a string
    if useNavx == True:
        currentTime = navx.read_time()
        currentDate = navx.read_date()
        timeString = str(navx.get_year(currentDate[4])) + str(currentDate[3]) + str(currentDate[2]) + str(currentTime[1]) + str(currentTime[2])
    else:
        currentTime = time.localtime(time.time())
        timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

    #Open a log file
    logFilename = '/home/pi/Team4121/Logs/Run_Log_' + timeString + '.txt'
    log_file = open(logFilename, 'w')
    log_file.write('run started on %s.\n' % datetime.datetime.now())
    log_file.write('')

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

    #Read camera settings file
    read_settings_file()

    #Create field camera stream (to find game pieces)
    if (findBalls == True) or (findMarkers == True):
        fieldCamSettings = {}
        fieldCamSettings['Width'] = int(cameraValues['FieldCamWidth'])
        fieldCamSettings['Height'] = int(cameraValues['FieldCamHeight'])
        fieldCamSettings['Brightness'] = cameraValues['FieldCamBrightness']
        fieldCamSettings['Exposure'] = cameraValues['FieldCamExposure']
        fieldCamSettings['FPS'] = cameraValues['FieldCamFPS']
        fieldCamera = FRCWebCam('/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0', "FieldCam", fieldCamSettings)

    #Create goal camera stream (to find vision tape marked shooting targets)
    if findGoal == True:
        goalCamSettings = {}
        goalCamSettings['Width'] = cameraValues['GoalCamWidth']
        goalCamSettings['Height'] = cameraValues['GoalCamHeight']
        goalCamSettings['Brightness'] = cameraValues['GoalCamBrightness']
        goalCamSettings['Exposure'] = cameraValues['GoalCamExposure']
        goalCamSettings['FPS'] = cameraValues['GoalCamFPS']
        goalCamera = FRCWebCam('/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.4:1.0-video-index0', "GoalCam", goalCamSettings)

    #Create vision processing
    visionProcessor = VisionLibrary(visionFile)

    #Create blank vision image
    imgFieldRaw = np.zeros(shape=(int(cameraValues['FieldCamWidth']), int(cameraValues['FieldCamHeight']), 3), dtype=np.uint8)
    imgGoalRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)

    #Start main processing loop
    while (True):

        #######################
        # Find field elements #
        #######################

        if (findBalls == True) or (findMarkers == True):

            #Define ball and marker variables
            ballPatternNumber = 0
            ballPatternName = ""

            #Read frame from camera
            imgFieldRaw = fieldCamera.read_frame()
            imgFieldNew = np.zeros(shape=(int(cameraValues['FieldCamWidth']), int(cameraValues['FieldCamHeight']), 3), dtype=np.uint8)

            #Call detection methods
            if findBalls == True:
                ballsFound, ballData = visionProcessor.detect_game_balls(imgFieldRaw, int(cameraValues['FieldCamWidth']),
                                                                        int(cameraValues['FieldCamHeight']),
                                                                        float(cameraValues['FieldCamFOV']))
            if findMarkers == True:
                markersFound, markerData = visionProcessor.detect_field_marker(imgFieldRaw, int(cameraValues['FieldCamWidth']),
                                                                        int(cameraValues['FieldCamHeight']),
                                                                        float(cameraValues['FieldCamFOV']))
            #Copy raw image
            imgFieldNew = imgFieldRaw

            #Draw ball contours and target data on the image
            if ballsFound > 0:

                #Detect ball pattern
                #ballPatternNumber, ballPatternName = determineBallPattern(1, ballData[0]['x'], ballData[0]['distance'], ballData[0]['angle'])

                
                #Loop over found balls and process data
                i = 0
                for ball in ballData:

                    if i < 3:

                        if i == 0:
                            cv.circle(imgFieldNew, (int(ball['x']), int(ball['y'])), int(ball['radius']), (0, 0, 255), 2)
                            cv.putText(imgFieldNew, 'Distance to Ball: %.2f' %ball['distance'], (10, 15), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                            cv.putText(imgFieldNew, 'Angle to Ball: %.2f' %ball['angle'], (10, 30), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                            cv.putText(imgFieldNew, 'Radius: %.2f' %ball['radius'], (10, 45), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                        else:
                            cv.circle(imgFieldNew, (int(ball['x']), int(ball['y'])), int(ball['radius']), (0, 255, 0), 2)

                    else:

                        cv.circle(imgFieldNew, (int(ball['x']), int(ball['y'])), int(ball['radius']), (0, 255, 0), 2)

                    #Write ball data to network table
                    if networkTablesConnected == True:
                        visionTable.putBoolean("FoundBall", bool(ballsFound > 0))
                        visionTable.putNumber("BallLayoutNum", ballPatternNumber)
                        visionTable.putString("BallLayoutName", ballPatternName)
                        visionTable.putNumber("BallDistance" + str(i), ball['distance'])
                        visionTable.putNumber("BallAngle" + str(i), ball['angle'])
                        visionTable.putNumber("BallScreenPercent" + str(i), ball['percent'])
                        visionTable.putNumber("BallOffset" + str(i), ball['offset'])

                    i += 1

            #Draw vision markers
            if markersFound > 0:

                #Copy raw image (only if not copied for balls)
                if findBalls == False:
                    imgFieldNew = imgFieldRaw

                #Loop over all contours and annotate image
                i = 0
                for marker in markerData:

                    if i == 0:

                        cv.rectangle(imgFieldNew, (int(marker['x']), int(marker['y'])), (int(marker['w']) + int(marker['x']), int(marker['y']) + int(marker['h'])), (0, 0, 255), 2)
                        cv.putText(imgFieldNew, 'Distance to Marker: %.2f' %marker['distance'], (10, 60), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                        cv.putText(imgFieldNew, 'Angle to Marker: %.2f' %marker['angle'], (10, 75), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
                    
                    else:

                        cv.rectangle(imgFieldNew, (int(marker['x']), int(marker['y'])), (int(marker['w']) + int(marker['x']), int(marker['y']) + int(marker['h'])), (0, 255, 0), 2)

                    #Write marker data to network table
                    if networkTablesConnected == True:
                        visionTable.putNumber("MarkerDistance" + str(i), marker['distance'])
                        visionTable.putNumber("MarkerAngle" + str(i), marker['angle'])
                        visionTable.putNumber("MarkerScreenPercent" + str(i), marker['percent'])
                        visionTable.putNumber("MarkerOffset" + str(i), marker['offset'])

                    i += 1

            #Display the vision camera stream (for testing only)
            if videoTesting == True:           
                cv.imshow("Ball", imgFieldNew)            

        #####################
        # Find goal targets #
        #####################

        if findGoal == True:

            #Read frame from camera
            imgGoalRaw = goalCamera.read_frame()
            imgBlankRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)
            imgGoalNew = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)

            #Call detection method
            tapeCameraValues, tapeRealWorldValues, foundTape, tapeTargetLock, rect, box = visionProcessor.detect_tape_rectangle(imgGoalRaw, int(cameraValues['GoalCamWidth']),
                                                                                                                            int(cameraValues['GoalCamHeight']),
                                                                                                                            float(cameraValues['GoalCamFOV']),
                                                                                                                            float(cameraValues['GoalCamFocalLength']),
                                                                                                                            float(cameraValues['GoalCamMountAngle']),
                                                                                                                            float(cameraValues['GoalCamMountHeight']))

            #Draw vision tape contours and target data on the image
            if foundTape == True:

                imgGoalNew = imgGoalRaw

                if tapeTargetLock:
                    cv.rectangle(imgGoalNew,(tapeCameraValues['TargetX'],tapeCameraValues['TargetY']),(tapeCameraValues['TargetX']+tapeCameraValues['TargetW'],tapeCameraValues['TargetY']+tapeCameraValues['TargetH']),(0,255,0),2) #vision tape
                    cv.drawContours(imgGoalNew, [box], 0, (0,0,255), 2)
            
                cv.putText(imgBlankRaw, 'Tape Distance (A): %.2f' %tapeRealWorldValues['StraightDistance'], (10, 30), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Tape Distance (S): %.2f' %tapeRealWorldValues['TapeDistance'], (10, 50), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Wall Distance: %.2f' %tapeRealWorldValues['WallDistance'], (10, 70), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Bot Angle: %.2f' %tapeRealWorldValues['BotAngle'], (10, 90), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'IPP: %.2f' %tapeCameraValues['IPP'], (10, 130), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Vert Offset: %.2f' %tapeRealWorldValues['VertOffset'], (10, 150), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Offset: %.2f' %tapeCameraValues['Offset'], (10, 170), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Target Width: %.2f' %tapeCameraValues['TargetW'], (10, 190), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)
                cv.putText(imgBlankRaw, 'Apparent Width: %.2f' %tapeRealWorldValues['ApparentWidth'], (10, 210), cv.FONT_HERSHEY_SIMPLEX, .45,(0, 0, 255), 1)

                #Write target data to network table
                if networkTablesConnected == True:
                    visionTable.putBoolean("FoundTape", foundTape)
                    visionTable.putBoolean("TargetLock", tapeTargetLock)
                    visionTable.putNumber("TapeDistance", tapeRealWorldValues['TapeDistance'])
                    visionTable.putNumber("TapeOffset", tapeCameraValues['Offset'])

            #Display the vision camera stream (for testing only)
            if videoTesting == True:           
                cv.imshow("Goal", imgGoalNew)
                cv.imshow("Data", imgBlankRaw)

        #####################
        # Process NavX Gyro #
        #####################

        #Get VMX gyro angle
        if useNavx == True:

            gyroInit = navxTable.getNumber("ZeroGyro", 0)  #Check for signal to re-zero gyro
            if gyroInit == 1:
                navx.reset_gyro()
                navxTable.putNumber("ZeroGyro", 0)      
            gyroAngle = navx.read_angle()  #Read gyro angle

        else:           
            
            gyroAngle = -9999  #Set default gyro angle
   
        #Put gyro value in NetworkTables
        if networkTablesConnected == True:
            navxTable.putNumber("GyroAngle", gyroAngle)

        #################################
        # Check for stopping conditions #
        #################################

        #Check for stop code from keyboard (for testing)
        if videoTesting == True:
            if cv.waitKey(1) == 27:
                break

        #Check for stop code from network tables
        if networkTablesConnected == True: 
            robotStop = visionTable.getNumber("RobotStop", 0)
            if (robotStop == 1) or (networkTablesConnected == False):
                break

        #Pause before next analysis
        #time.sleep(0.066) #should give ~15 FPS

    #Close all open windows (for testing)
    if videoTesting == True:
        cv.destroyAllWindows()

    #Release field camera
    if (findBalls == True) or (findMarkers == True):
        fieldCamera.release_cam()

    #Release goal camera
    if findGoal == True:
        goalCamera.release_cam()

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    log_file.close()


#define main function
if __name__ == '__main__':
    main()
