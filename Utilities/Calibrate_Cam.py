# -*- coding: utf-8 -*-

###############################################################
#                                                             #
#                   FRC Camera Calibration                    #
#                                                             #
#  This program calculates calibration constants for webcams  #
#  used on FRC robots.  A standard chessboard image is used   #
#  as the calibration image.  This program assumes no more    #
#  than 4 webcams are connected at the same time.             #
#                                                             #
#  @Author: Team4121                                          #
#  @Created: 2020-06-20                                       #
#  @Version: 1.0                                              #
#                                                             #
###############################################################

"""FRC webcam calibration utility"""

# System imports
import sys
import os
import imp

# Module imports
import cv2 as cv
import numpy as np
import math
import datetime
import time
import logging

# Set general variables
#calibration_dir = 'C:/Users/timfu/Documents/FRC General/Camera'
#working_dir = 'C:/Users/timfu/Documents/FRC General/Camera/Calibration_Images'
calibration_dir = '/home/pi/Programs/Python/Camera'
working_dir = '/home/pi/Programs/Python/Camera/Calibration_Images'
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Set image variables
img_width = 640
img_height = 480
frames_per_sec = 15
img_brightness = 100
img_exposure = 50
number_of_images = 0
device_id = 0

# Create calibration arrays
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objpoints = []
imgpoints = []


# Define main processing function
def mainloop():

    # Clear image working directory
    print('Clearing working directory...')
    for fname in os.listdir(working_dir):
        os.remove(working_dir + '/' + fname)

    # Prompt for which camera
    device_temp = int(input('Camera to calibrate (0, 1, 2 or 3): '))
    if device_temp <= 3:
        device_id = device_temp
                      
    # Prompt for number of images
    number_of_images = int(input('Number of calibration images: '))

    # Set up camera
    camera = cv.VideoCapture(device_id)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, img_width)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, img_height)
    camera.set(cv.CAP_PROP_FPS, frames_per_sec)
    camera.set(cv.CAP_PROP_BRIGHTNESS, img_brightness)
    camera.set(cv.CAP_PROP_EXPOSURE, img_exposure)

    # Make sure video capture is open
    if camera.isOpened() == False:
        camera.open(device_id)

    # Create a blank image
    img = np.zeros(shape=(img_width, img_height, 3), dtype=np.uint8)

    # Grab the required number of calibration images
    good_images = 0
    while (good_images < number_of_images):

        # Loop until good image found
        returnflag = False
        while (returnflag == False):

            # Grab a frame of video
            (grabbed, frame) = camera.read()

            # Check that we have a frame before processing
            if grabbed == True:

                # Convert image to grayscale
                img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

                # Find the chessboard corners
                returnflag, corners = cv.findChessboardCorners(img_gray, (9,6), None)

                # Show image
                if returnflag == True:

                    # Increase good image count
                    good_images += 1
                
                    # Add object points to array
                    objpoints.append(objp)

                    # Sharpen corners
                    corners2 = cv.cornerSubPix(img_gray,corners,(11,11),(-1,1),criteria)

                    # Add image points to array
                    imgpoints.append(corners2)

                    # Save the image for future processing
                    img_filename = working_dir + '/CalibrationImage_' + str(good_images) + '.jpg'
                    cv.imwrite(img_filename, img)
                    print('Image saved as ' + img_filename)

                    # Draw chessboard on image and show
                    img = cv.drawChessboardCorners(frame, (9,6), corners2, returnflag)
                    cv.imshow('Camera Image',img)
                    cv.waitKey(2)

                else:

                    # Show raw image
                    cv.imshow('Camera Image',frame)
                    cv.waitKey(2)

            # Allow user to break out of loop
            if cv.waitKey(1) == 27:
                good_images = number_of_images
                break

        # Tell user to move chessboard image
        print('Move chessboard for next image')
        cv.waitKey(2000)

    # Close all open windows
    cv.destroyAllWindows()

    # Release video capture
    camera.release()

    # Calculate camera calibration
    print('Calculating camera calibration...')
    ret, cam_matrix, dist_coeffs, rotate_vecs, translate_vecs = cv.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)

    # Save camera matrix and distortion coefficients
    print('Saving camera calibration to files...')
    currentTime = time.localtime(time.time())
    timeString = str(currentTime.tm_year) + str(currentTime.tm_mon) + str(currentTime.tm_mday) + str(currentTime.tm_hour) + str(currentTime.tm_min)
    matrix_filename = calibration_dir + '/Camera_Matrix_Cam' + str(device_id) + '_' + timeString + '.txt'
    coeff_filename = calibration_dir + '/Distortion_Coeffs_Cam' + str(device_id) + '_' + timeString + '.txt'
    np.savetxt(matrix_filename, cam_matrix)
    np.savetxt(coeff_filename, dist_coeffs)

    # #Show raw camera image and undistorted image
    # print('Testing camera calibration...')
    # new_img = np.zeros(shape=(img_width, img_height, 3), dtype=np.uint8)
    # while(True):

    #     timestamp, new_img = cvsink.grabFrame(new_img)
    #     new_img_undist = cv.undistort(new_img, cam_matrix, dist_coeffs, None, optcam_matrix)
    #     cv.imshow('Original', new_img)
    #     cv.imshow('Undistorted', new_img_undist)

    #     if cv.waitKey(1) == 27:
    #         cv.destroyAllWindows()
    #         break
          

# Define main function
def main():
    mainloop()

if __name__ == '__main__':
    main()
