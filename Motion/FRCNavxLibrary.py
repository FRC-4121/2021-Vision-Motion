# -*- coding: utf-8 -*-

##################################################################
#                                                                #
#                       FRC Navx Library                         #
#                                                                #
#  This class is a wrapper around the HAL-Navx libraries.  This  #
#  class provides threading of the Navx board interactions.      #
#                                                                #
#  @Version: 1.0                                                 #
#  @Created: 2020-2-11                                           #
#  @Author: Team 4121                                            #
#                                                                #
##################################################################

'''FRC Navx Library - Provides threaded methods and utilities for Navx board'''

#!/usr/bin/env python3

# System imports
import sys
import imp

# Setup paths
sys.path.append('/usr/local/lib/vmxpi/')

# Module imports
import math
import time
import datetime
import logging
from threading import Thread


# Define the Navx class
class FRCNavx:

    # Define initialization
    def __init__(self, name):

        # Load VMX module
        self.vmxpi = imp.load_source('vmxpi_hal_python', '/usr/local/lib/vmxpi/vmxpi_hal_python.py')
        self.vmx = self.vmxpi.VMXPi(False,50)
        self.vmxOpen = self.vmx.IsOpen()

        # Log error if VMX didn't open properly
        if self.vmxOpen is False:

            # Get current time as a string
            currentTime = time.localtime(time.time())
            timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)

            # Open a log file
            logFilename = '/data/Logs/Navx_Log_' + timeString + '.txt'
            self.log_file = open(logFilename, 'w')
            self.log_file.write('Navx initialized on %s.\n' % datetime.datetime.now())
            self.log_file.write('')

            # Write error message
            self.log_file.write('Error:  Unable to open VMX Client.\n')
            self.log_file.write('\n')
            self.log_file.write('        - Is pigpio (or the system resources it requires) in use by another process?\n')
            self.log_file.write('        - Does this application have root privileges?')
            self.log_file.close()

        # Set name of Navx thread
        self.name = name

        # Initialize Navx values
        self.angle = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.time = []
        self.date = []
        
        # Reset Navx and initialize time
        if self.vmxOpen is True:
            self.vmx.getAHRS().Reset()
            self.vmx.getAHRS().ZeroYaw()
            self.time = self.vmx.getTime().GetRTCTime()
            self.date = self.vmx.getTime().GetRTCDate()
       

    # Define read angle method
    def read_angle(self):

        self.angle = round(self.vmx.getAHRS().GetAngle(), 2)
        return self.angle


    # Define read yaw method
    def read_yaw(self):

        self.yaw = round(self.vmx.getAHRS().GetYaw(), 2)
        return self.yaw


    # Define read pitch method
    def read_pitch(self):

        self.pitch = round(self.vmx.getAHRS().GetPitch(), 2)
        return self.pitch


    # Define reset gyro method
    def reset_gyro(self):

        self.vmx.Reset()
        self.vmx.ZeroYaw()


    # Define read time method
    def read_time(self):

        self.time = self.vmx.getTime().GetRTCTime()
        return self.time
    

    # Define read date method
    def read_date(self):

        self.date = self.vmx.getTime().GetRTCDate()
        return self.date
    

    # Define set time method
    def set_time(self, newtime):

        success = self.vmx.getTime().SetRTCTime(newtime[0], 
                                                newtime[1],
                                                newtime[2])
        
        return success
    

    # Define set date method
    def set_date(self, newdate):

        success = self.vmx.getTime().SetRTCDate(newdate[0],
                                                newdate[1],
                                                newdate[2],
                                                newdate[3])

        return success
    

    # Define get raw time method
    def get_raw_time(self):

        currentTime = self.read_time()
        currentDate = self.read_date()
        timeString = str(self.get_year(currentDate[4])) + str(currentDate[3]) + str(currentDate[2]) + str(currentTime[1]) + str(currentTime[2])
        return timeString
        

    # Define day of week conversion method
    def get_day_name(self, weekday):

        if weekday == 1:
            return 'Monday'
        elif weekday == 2:
            return 'Tuesday'
        elif weekday == 3:
            return 'Wednesday'
        elif weekday == 4:
            return 'Thursday'
        elif weekday == 5:
            return 'Friday'
        elif weekday == 6:
            return 'Saturday'
        elif weekday == 7:
            return 'Sunday'
    

    # Define month conversion method
    def get_month_name(self, month):

        if month == 1:
            return 'January'
        elif month == 2:
            return 'February'
        elif month == 3:
            return 'March'
        elif month == 4:
            return 'April'
        elif month == 5:
            return 'May'
        elif month == 6:
            return 'June'
        elif month == 7:
            return 'July'
        elif month == 8:
            return 'August'
        elif month == 9:
            return 'September'
        elif month == 10:
            return 'October'
        elif month == 11:
            return 'November'
        elif month == 12:
            return 'December'
    

    # Define year conversion method
    def get_year(self, year):

        return (year + 2000)

