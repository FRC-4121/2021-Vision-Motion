# -*- coding: utf-8 -*-

#-----------------------------------------------------------------------
#
# FRC Game Field Mapper
#
# This class maps out an FRC game field including all fixed obstacles, 
# loading stations, and scoring locations.  The game field itself is 
# divided into M x N squares. An extra square is included on each side 
# of the field to represent the hard edge walls of the field as well as 
# potential scoring locations. In code, the field is represented as an 
# (M+2) x (N+2) NUMPY array. The size of the field as well as the 
# physical size of each square are read in from a text file.  In this
# class, the size of the game field is referred to as LENGTH (M) and
# WIDTH (N).  The size of each grid sqaure is referred to as the
# SCALEFACTOR, which has units of inches.
# 
# The field obstacles, loading, and scoring locations are read in from 
# a separate text file so the map can be customized for each game.
#
# The position of field/game elements are represented by integers
# in the array.  The standard integers used every year are:
#   -1 = impassable field element square
#    0 = open field square
#    1 = 4121 robot position
#    2 = Other robots
# The meaning of other integers can vary from game to game and is,
# therefore, read in from the same text file used to set the size
# of the game field.
# 
# The left edge of the map is assumed to be the alliance stations.
# A robot heading of 0 degrees points to the right edge of the field.
# Angles are measured clockwise which means a heading of +90 degrees 
# points to the bottom edge of the field.
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
#-----------------------------------------------------------------------

'''FRC Field Mapper - Provides tracking of robot on game field'''

#Imports
import numpy as np
import math

#Define the field mapper class
class FrcFieldMapper:

    #Define class fields
    fieldDesignFile=""
    gameElementsFile = ""
    elementsMapFile = ""
    fieldValues = {}
    fieldMap = 0
    robotPosition = [0,0,0]


    #Define class initialization
    def __init__(self, designFile, elementsFile, mappingFile):
        
        #Set game files
        self.fieldDesignFile = designFile
        self.gameElementsFile = elementsFile
        self.elementsMapFile = mappingFile
        
        #Initialize game field
        self.init_game_field()
        self.init_game_elements()


    #Initialize overall structure of game field
    def init_game_field(self):

        #Read in the field setup file
        self.read_field_file()

        #Setup NUMPY array field representation
        fieldWidth = int(self.fieldValues["WIDTH"]) + 2
        fieldLength = int(self.fieldValues["LENGTH"]) + 2
        self.fieldMap = np.zeros(((fieldWidth + 2),(fieldLength + 2)),
									dtype=int)
       

    #Initialize game elements
    def init_game_elements(self):
        
        #Initialize edges of field
        widthEdge = int(FrcFieldMapper.fieldValues["WIDTH"]) + 1
        lengthEdge = int(FrcFieldMapper.fieldValues["LENGTH"]) + 1
        self.fieldMap[0:widthEdge,0] = -1
        self.fieldMap[0:widthEdge,lengthEdge] = -1
        self.fieldMap[0,0:lengthEdge] = -1
        self.fieldMap[widthEdge,0:lengthEdge] = -1

        #Load current game field elements
        fieldData = np.loadtxt(FrcFieldMapper.gameElementsFile,
								dtype=int,
								delimiter=',')

        #Apply data to game field
        for x,y,h in fieldData:
            self.fieldMap[x,y] = h
            if h == 1:
                self.robotPosition[0] = x
                self.robotPosition[1] = y


    #Read field setup file
    def read_field_file(self):

        #Open the file and read contents
        try:
            
            #Open the file for reading
            in_file = open(self.designFile, 'r')
            
            #Read in all lines
            value_list = in_file.readlines()
            
            #Process list of lines
            for line in value_list:
                
                #Remove trailing newlines and whitespace
                clean_line = line.strip()

                #Split the line into parts
                split_line = clean_line.split(',')

                #Save the value into dictionary
                self.fieldValues[split_line[0]] = split_line[1]

        except:

            self.fieldValues['LENGTH'] = 54
            self.fieldValues['WIDTH'] = 27
            self.fieldValues['SCALEFACTOR'] = 12.0


    #Round a number to specified decimal places
    def RoundNumber(self, n, decimals=0):
        multiplier = 10**decimals
        round_abs = math.floor(abs(n)*multiplier + 0.5) / multiplier
        return math.copysign(round_abs, n)


    #Update robot's position on the field
    def UpdatePosition(self, distance, angle):

        #Convert distance and angle to X, Y movement
        dx = distance * math.cos(math.radians(angle))
        dy = distance * math.sin(math.radians(angle))

        #Round movement to next highest foot
        scaleFactor = float(self.fieldValues['SCALEFACTOR'])
        dx_squares = int(self.RoundNumber((dx / scaleFactor), 0))
        dy_squares = int(self.RoundNumber((dy / scaleFactor), 0))

        #Adjust robot position
        current_x = self.robotPosition[0]
        current_y = self.robotPosition[1]
        new_x = current_x
        new_y = current_y

        if angle <= 90.0 and angle >= -90.0:
            new_x = current_x + dx_squares
            if angle >= 0:
                new_y = current_y + dy_squares
            else:
                new_y = current_y - dy_squares
        else:
            new_x = current_x - dx_squares
            if angle >= 0:
                new_y = current_y + dy_squares
            else:
                new_y = current_y - dy_squares

        self.fieldMap[current_x, current_y] = 0
        self.fieldMap[new_x, new_y] = 1
        self.robotPosition[0] = new_x
        self.robotPosition[1] = new_y
        self.robotPosition[2] = angle
