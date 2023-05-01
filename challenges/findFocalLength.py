#Based on the yetiRace code

# Standard imports
from __future__ import division
import sys,tty,termios,os
import time
import cv2
import numpy as np

from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
from PIL import Image

import ZeroBorg

# Camera settings
width = 640/2
height = 480/2
frameRate = 32

#Define object 
objectWidth = 7 #cm
initialDistance = 50 #cm

# Initializing the camera
camera = PiCamera()
camera.awb_mode = 'auto'
camera.resolution = (int(width), int(height))
camera.framerate = frameRate
rawCapture = PiRGBArray(camera, size=(int(width), int(height)))
time.sleep(0.1)

# Settings
# Color setting for the mask
hueLow = 0
saturationLow = 88
valueLow = 75

hueHigh = 11
saturationHigh = 25
valueHigh = 251

# Toggles for debuging
displayWindows = False
ready = False

# Variables
xPos = 0
steerMultiplier = 0.8

# frame = camera.capture(rawCapture, format="bgr", use_video_port=True)
# # Grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
# # Flip frame for right orientation
# imagePi = cv2.flip(frame, 0)
# imagePi = cv2.flip(imagePi,1)
# canvas = imagePi.copy()

img = cv2.imread("./pic.png", cv2.IMREAD_GRAYSCALE)

# Convert to hsv for better image processing
# img_norm = cv2.normalize(imagePi, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=np.uint8)
# img_hsv = cv2.cvtColor(img_norm, cv2.COLOR_BGR2HSV)

# Generate mask with pre defined colors -> lower and upper bound
# lower = (hueLow,saturationLow,valueLow)
# upper = (hueHigh,saturationHigh,valueHigh)
lower = np.array([100], dtype=np.uint8)
upper = np.array([200], dtype=np.uint8)
mask = cv2.inRange(img, lower, upper)

# Create the contours and find the center
try:
    # NB: using _ as the variable name for the output, as it is not used
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    blob = max(contours, key=lambda el: cv2.contourArea(el))
    #print(blob)

    # returns [center,size][angle]
    bounds = cv2.minAreaRect(blob)
    print(bounds[1][0])
    # cv2.minAreaRect() gives Box2D structure and not rotated rectangle around object

    focalLength = (bounds[1][0] * initialDistance) / objectWidth

    print(focalLength)
    camera.close()
except (ValueError, ZeroDivisionError):
    pass