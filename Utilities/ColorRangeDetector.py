#!/usr/bin/env python
# -*- coding: utf-8 -*-

###############################################################
#                                                             #
#                   Color Range Detector                      #
#                                                             #
#  This program provides tools to determine the color values  #
#  (RGB or HSV) of a target object in a captured video frame  #
#                                                             #
#  @Version: 1.1.0                                            #
#  @Created: 2021-01-17                                       #
#  @Author:  Jonas Muhlenkamp, Tim Fuller                     #
#  @FRCTeam: 4121                                             #
#                                                             #
###############################################################

'''FRC Color Range Detector'''

# Import modules
import cv2
import argparse
from operator import xor


# Define callback method
def callback(value):
    pass


# Create trackbar window
def setup_trackbars(range_filter):

    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:

        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)


# Retrieve current trackbar values
def get_trackbar_values(range_filter):

    values = []

    for i in ["MIN", "MAX"]:

        for j in range_filter:

            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


# Define the main method
def main():

    # Set type of filter
    #range_filter = "HSV"
    range_filter = "BGR"

    # Setup webcam capture
    camera = cv2.VideoCapture(0)
    camera.set(10, 0.3)  # Brightness
    camera.set(15, 30)  # Exposure

    # Create trackbar window
    setup_trackbars(range_filter)

    # Main program loop
    while True:

        # Read frame from camera
        ret, image = camera.read()

        # Make sure good frame retrieved
        if not ret:
            break

        # Blur image to remove noise
        blur = cv2.GaussianBlur(image.copy(),(5,5),0)

        # Convert color space (if HSV)
        if range_filter == 'HSV':
            frame_to_thresh = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        else:
            frame_to_thresh = blur.copy()
        
        # Get trackbar values
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        # Threshold frame based on trackbar values
        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        # Show the original and threshold images
        cv2.imshow("Original", image)
        cv2.imshow("Thresh", thresh)

        # Check if user wants to exit
        if cv2.waitKey(1) == 27:
            break

    # Release camera
    camera.release()

    # Close open windows
    cv2.destroyAllWindows()


# Run main method
if __name__ == '__main__':
    main()
