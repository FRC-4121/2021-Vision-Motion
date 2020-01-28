# -*- coding: utf-8 -*-

#-----------------------------------------------------------------
#
# FRC Game Field Mapper
#
# This class maps out an FRC game field including all fixed
# obstacles and scoring locations.  The obstacles and scoring
# locations are read in from a text file so the game map can be
# customized for each year's game.
#
# The game field is divided into 1' x 1' squares.  An extra
# square is included on each side of the field to represent the
# hard edge walls of the field as well as potential scoring locations.
# The FRC field is always 54' x 27' so, accounting for the edge 
# squares, the field is represented as a 29 x 56 NUMPY array.
#
# The position of field/game elements are represented by integers
# in the array according to the following key:
#   -1 = impassable field element square
#    0 = open field square
#    1 = 4121 robot position
#    2 = Scoring square - type 1
#    3 = Scoring square - type 2
#    9 = Other robots
#
# The 4121 robot always starts in one of the left edge (column 1)
# squares.  A robot heading of 0 degrees points to the right edge
# of the field.  A heading of +90 degrees points to the bottom
# edge of the field and so on.
#
# This class provides methods for tracking and returning the
# position of the robot in real time.  Methods are also provided
# to return the direction and distance to the closest scoring
# square of a specified type.
# 
# @Version: 1.0
#  
# @Created: 2019-12-28
#
# @Author: Team 4121
#
#-----------------------------------------------------------------

'''FRC Field Mapper - Provides tracking of robot on game field'''

#Imports
import numpy as np
import math

#Define the field mapper class
class FrcFieldMapper:

    #Define class fields
    mapFile = ""
    fieldMap = 0
    robotPosition = [0,0,0]


    #Initialize field
    def InitializeField(self):
        
        #Initialize edges of field
        FrcFieldMapper.fieldMap[0:28,0] = -1
        FrcFieldMapper.fieldMap[0:28,55] = -1
        FrcFieldMapper.fieldMap[0,0:55] = -1
        FrcFieldMapper.fieldMap[28,0:55] = -1

        #Load current game field elements
        fieldData = np.loadtxt(FrcFieldMapper.mapFile, dtype=int, delimiter=',')

        #Apply data to game field
        for x,y,h in fieldData:
            FrcFieldMapper.fieldMap[x,y] = h
            if h == 1:
                FrcFieldMapper.robotPosition[0] = x
                FrcFieldMapper.robotPosition[1] = y


    #Round a number to specified decimal places
    def RoundNumber(self, n, decimals=0):
        multiplier = 10**decimals
        round_abs = math.floor(abs(n)*multiplier + 0.5) / multiplier
        return math.copysign(round_abs, n)


    #Update robot's position on the field
    def UpdatePosition(self, dx, dy, angle):

        #Round movement to next highest foot
        dx_squares = int(FrcFieldMapper.RoundNumber(self, (dx / 12), 0))
        dy_squares = int(FrcFieldMapper.RoundNumber(self, (dy / 12), 0))

        #Adjust robot position
        current_x = FrcFieldMapper.robotPosition[0]
        current_y = FrcFieldMapper.robotPosition[1]
        new_x = current_x + dx_squares
        new_y = current_y + dy_squares
        FrcFieldMapper.fieldMap[current_x, current_y] = 0
        FrcFieldMapper.fieldMap[new_x, new_y] = 1
        FrcFieldMapper.robotPosition[0] = new_x
        FrcFieldMapper.robotPosition[1] = new_y
        FrcFieldMapper.robotPosition[2] = angle


    #Define class initialization
    def __init__(self, mapfile):
        
        #Initialize game field
        FrcFieldMapper.mapFile=mapfile
        FrcFieldMapper.fieldMap = np.zeros((29,56), dtype=int)
        self.InitializeField()

