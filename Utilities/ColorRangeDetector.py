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
import numpy as np
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
    range_filter = "HSV"
    #range_filter = "BGR"

    # Setup webcam capture
    camera = cv2.VideoCapture('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0')
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)  # Brightness
    camera.set(cv2.CAP_PROP_EXPOSURE, 0)  # Exposure
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

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

        kernel = np.ones((3,3), np.uint8)
        erode = cv2.erode(blur, kernel, iterations=2)
        dilate = cv2.dilate(erode, kernel, iterations=2)

        # Convert color space (if HSV)
        if range_filter == 'HSV':
            frame_to_thresh = cv2.cvtColor(dilate, cv2.COLOR_BGR2HSV)
        else:
            frame_to_thresh = dilate.copy()
        
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
