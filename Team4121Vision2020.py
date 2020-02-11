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
#  Revision: 4.0                                                                               #
#                                                                                              #
#  Revision Date: 2/2/2020                                                                    #
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

#Set up basic logging
logging.basicConfig(level=logging.DEBUG)

#Declare global variables
cameraFile = '/home/pi/Team4121/Config/2020CameraSettings.txt'
visionFile = '/home/pi/Team4121/Config/2020VisionSettings.txt'
cameraValues={}

#Define program control flags
writeVideo = True
sendVisionToDashboard = True


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
        cameraValues['GoalCamFOV'] = 27.3
        cameraValues['GoalCamWidth'] = 320
        cameraValues['GoalCamHeight'] = 240
        cameraValues['GoalCamFPS'] = 15
        cameraValues['GoalCamBrightness'] = 0
        cameraValues['GoalCamExposure'] = 0
        cameraValues['GoalCamCalFactor'] = 1


#Define main processing function
def main():

    #Define global variables
    global writeVideo
   
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

    #Read camera settings file
    read_settings_file()

    #Connect NetworkTables
    try:
        NetworkTables.initialize(server='10.41.21.2')
        visionTable = NetworkTables.getTable("vision")
        networkTablesConnected = True
        log_file.write('Connected to Networktables on 10.41.21.2 \n')
    except:
        log_file.write('Error:  Unable to connect to Network tables.\n')
        log_file.write('Error message: ', sys.exc_info()[0])
        log_file.write('\n')

    #Create ball camera stream
    ballCamSettings = {}
    ballCamSettings['Width'] = cameraValues['BallCamWidth']
    ballCamSettings['Height'] = cameraValues['BallCamHeight']
    ballCamSettings['Brightness'] = cameraValues['BallCamBrightness']
    ballCamSettings['Exposure'] = cameraValues['BallCamExposure']
    ballCamSettings['FPS'] = cameraValues['BallCamFPS']
    ballCamera = FRCWebCam(0, "BallCam", ballCamSettings)
    ballCamera.start_camera()

    #Create goal camera stream
    goalCamSettings = {}
    goalCamSettings['Width'] = cameraValues['GoalCamWidth']
    goalCamSettings['Height'] = cameraValues['GoalCamHeight']
    goalCamSettings['Brightness'] = cameraValues['GoalCamBrightness']
    goalCamSettings['Exposure'] = cameraValues['GoalCamExposure']
    goalCamSettings['FPS'] = cameraValues['GoalCamFPS']
    goalCamera = FRCWebCam(1, "GoalCam", goalCamSettings)
    goalCamera.start_camera()

    #Create vision processing
    visionProcessor = VisionLibrary(visionFile)

    #Create blank vision image
    imgBallRaw = np.zeros(shape=(int(cameraValues['BallCamWidth']), int(cameraValues['BallCamHeight']), 3), dtype=np.uint8)
    imgGoalRaw = np.zeros(shape=(int(cameraValues['GoalCamWidth']), int(cameraValues['GoalCamHeight']), 3), dtype=np.uint8)

    #Start main processing loop
    while (True):

        #Read in an image from a file (for testing)
        #img = cv.imread('RetroreflectiveTapeImages2019/CargoStraightDark90in.jpg')
        #if img is None:
        #    break

        #Read frames from cameras
        imgBallRaw = ballCamera.read_frame()
        imgGoalRaw = goalCamera.read_frame()

        
        #Call detection methods
        ballX, ballY, ballRadius, ballDistance, ballAngle, ballOffset, ballScreenPercent, foundBall = visionProcessor.detect_game_balls(imgBallRaw, int(cameraValues['BallCamWidth']),
                                                                                                                                                    int(cameraValues['BallCamHeight']),
                                                                                                                                                    float(cameraValues['BallCamFOV']))
        tapeX, tapeY, tapeW, tapeH, tapeOffset, tapeDistance, tapeHAngle, tapeVAngle, foundTape = visionProcessor.detect_tape_rectangle(imgGoalRaw, int(cameraValues['GoalCamWidth']),
                                                                                                                                                    int(cameraValues['GoalCamHeight']),
                                                                                                                                                    float(cameraValues['GoalCamFOV']))
        #visionTable.putNumber("BallX", round(ballX, 2))

        #Draw various contours on the image
        if foundBall == True:
            cv.circle(imgBallRaw, (int(ballX), int(ballY)), int(ballRadius), (0, 0, 255), 2) #ball
            cv.putText(imgBallRaw, 'Distance to Ball: %.2f' %ballDistance, (10, 15), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)
            cv.putText(imgBallRaw, 'Angle to Ball: %.2f' %ballAngle, (10, 30), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 0, 255), 2)                
        if foundTape == True:
            cv.rectangle(imgGoalRaw,(tapeX,tapeY),(tapeX+tapeW,tapeY+tapeH),(0,255,0),2) #floor tape
            cv.putText(imgGoalRaw, 'Target Width: %.2f' %tapeW, (10, 170), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 255, 0), 2)
            cv.putText(imgGoalRaw, 'Distance to Tape: %.2f' %tapeDistance, (10, 185), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 255, 0), 2)
            cv.putText(imgGoalRaw, 'Horiz. Angle to Tape: %.2f' %tapeHAngle, (10, 200), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 255, 0), 2)
            cv.putText(imgGoalRaw, 'Vert. Angle to Tape: %.2f' %tapeVAngle, (10, 215), cv.FONT_HERSHEY_SIMPLEX, .5,(0, 255, 0), 2)


####            navxTable.putNumber("GyroAngle", round(vmx.getAHRS().GetAngle(), 2))

####


        #Display the vision camera stream (for testing only)
        cv.imshow("Ball", imgBallRaw)
        cv.imshow("Goal", imgGoalRaw)

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

        #Pause before next analysis
        time.sleep(0.066) #should give ~15 FPS

    #Close all open windows (for testing)
    cv.destroyAllWindows()

    ballCamera.release_cam()
    goalCamera.release_cam()

    #Close the log file
    log_file.write('Run stopped on %s.' % datetime.datetime.now())
    log_file.close()
    

#define main function
if __name__ == '__main__':
    main()