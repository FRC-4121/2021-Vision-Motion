# -*- coding: utf-8 -*-

#-----------------------------------------------------------------
#
# FRC Navx Library
#
# This class is a wrapper around the HAL-Navx libraries.  This
# class provides threading of the Navx board interactions.
# 
# @Version: 1.0
#  
# @Created: 2020-2-11
#
# @Author: Team 4121
#
#-----------------------------------------------------------------

'''FRC Camera Library - Provides camera methods and utilities'''

#!/usr/bin/env python3

#System imports
import sys
import imp

#Setup paths
sys.path.append('/usr/local/lib/vmxpi/')

#Module imports
import math
import time
import datetime
import logging
from threading import Thread

#Define the Navx class
class FRCNavx:

    #Define initialization
    def __init__(self, name):

        #Load VMX module
        self.vmxpi = imp.load_source('vmxpi_hal_python', '/usr/local/lib/vmxpi/vmxpi_hal_python.py')
        self.vmx = self.vmxpi.VMXPi(False,50)
        self.vmxOpen = self.vmx.IsOpen()

        #Log error if VMX didn't open properly
        if self.vmxOpen is False:

            #Get current time as a string
            currentTime = time.localtime(time.time())
            timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

            #Open a log file
            logFilename = '/data/Logs/Navx_Log_' + timeString + '.txt'
            self.log_file = open(logFilename, 'w')
            self.log_file.write('Navx initialized on %s.\n' % datetime.datetime.now())
            self.log_file.write('')

            #Write error message
            self.log_file.write('Error:  Unable to open VMX Client.\n')
            self.log_file.write('\n')
            self.log_file.write('        - Is pigpio (or the system resources it requires) in use by another process?\n')
            self.log_file.write('        - Does this application have root privileges?')
            self.log_file.close()

        #Set name of Navx thread
        self.name = name

        #Initialize stop flag
        if self.vmxOpen is True:
            self.stopped = False
        else:
            self.stopped = True

        #Initialize Navx values
        self.angle = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        
        #Reset Navx
        if self.vmxOpen is True:
            self.vmx.getAHRS().Reset()
            self.vmx.getAHRS().ZeroYaw()
        

    #Define Navx thread start method
    def start_navx(self):

        #Define navx thread
        navxThread = Thread(target=self.update, name=self.name, args=())
        navxThread.daemon = True
        navxThread.start()

        return self


    #Define Navxstop method
    def stop_navx(self):

        #Set stop flag
        self.stopped = True


    #Define main thread update method
    def update(self):

        #Main thread loop
        while True:

            #Check stop flag
            if self.stopped:
                return

            #If not stopped, read Navx values
            self.angle = round(self.vmx.getAHRS().GetAngle(), 2)
            #self.yaw = round(self.vmx.getAHRS().GetYaw(), 2)
            #self.pitch = round(self.vmx.getAHRS().GetPitch(), 2)

            time.sleep(.5)
    

    #Define read angle method
    def read_angle(self):

        return self.angle


    #Define read yaw method
    def read_yaw(self):

        return self.yaw


    #Define read pitch method
    def read_pitch(self):

        return self.pitch


    #Define reset gyro method
    def reset_gyro(self):

        self.vmx.Reset()
        self.vmx.ZeroYaw()


    #Define current time method
    def get_time(self):

        return
