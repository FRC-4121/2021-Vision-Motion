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
sys.path.append('/home/pi/Programs/Python/Libraries')
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
#cameraFieldOfView = 23.5 #value designated for vision tape
cameraFieldOfView = 27.3

#Define program control flags
writeVideo = True
sendVisionToDashboard = True


#Define main processing function
def main():

    #Define global variables
    global imgWidthVision
    global imgHeightVision
    global writeVideo

    #Define camera configuration variables
    visionCameraBrightness = 25
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
        log_file.write('Error message: ', sys.exc_info()[0])
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
        log_file.write('Error message: ', sys.exc_info()[0])
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
