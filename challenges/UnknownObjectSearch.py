﻿# Standard imports
from __future__ import division
import sys,tty,termios,os
import time
import math
import cv2
import numpy as np
import keyboard

from time import sleep

from picamera.array import PiRGBArray
from picamera import PiCamera

import ZeroBorg

# Camera settings
width = 640/2
height = 480/2
frameRate = 32

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
saturationHigh = 255
valueHigh = 251

# Toggles for debuging
displayWindows = False
ready = False

# Variables
xPos = 0;
steerMultiplier = 0.8

# Initial hardcoded values for calculating the focal length - TODO change with user input
KNOWN_DISTANCE = 50 #cm
KNOWN_WIDTH = 5.5 #cm

# Array to store distance and calculate median
medianDistanceArray = []

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    lower = (hueLow,saturationLow,valueLow)
    upper = (hueHigh,saturationHigh,valueHigh)
    mask = cv2.inRange(gray, lower, upper)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(cnts) > 0:
        # Find the contour with the largest area
        c = max(cnts, key=lambda el: cv2.contourArea(el))

        # finds the center of an object
        M = cv2.moments(c)
        # print(M)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        xPos = center[0]
        cv2.circle(canvas, center, 5, (0,255,0), -1)

        #save canvas to image to be displayed
        cv2.imwrite('stopAsCloseAsPossible.png', canvas)
    else:
        print('No contours found in the image')
        ZB.MotorsOff()
	# compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

# Setup the ZeroBorg
ZB = ZeroBorg.ZeroBorg()
#ZB.i2cAddress = 0x44                   # Uncomment and change the value if you have changed the board address
ZB.Init()
if not ZB.foundChip:
    boards = ZeroBorg.ScanForZeroBorg()
    if len(boards) == 0:
        print ('No ZeroBorg found, check you are attached :)')
    else:
        print ('No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress))
        for board in boards:
            print ('    %02X (%d)' % (board, board))
        print ('If you need to change the IC address change the setup line so it is correct, e.g.')
        print ('ZB.i2cAddress = 0x%02X' % (boards[0]))
    sys.exit()
#ZB.SetEpoIgnore(True)                  # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
ZB.SetCommsFailsafe(False)              # Disable the communications failsafe
ZB.ResetEpo()


# Movement settings (worked out from our YetiBorg v2 on a smooth surface)
timeForward1m = 3.0 #5.7                     # Number of seconds needed to move about 1 meter
timeSpin360   = 20.0 #4.8                     # Number of seconds needed to make a full left / right spin
testMode = False                        # True to run the motion tests, False to run the normal sequence

# Power settings
voltageIn = 8.4                         # Total battery voltage to the ZeroBorg (change to 9V if using a non-rechargeable battery)
voltageOut = 3.0                        # Maximum motor voltage

# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)

# Function to perform a general movement
def PerformMove(driveLeft, driveRight, numSeconds):
    # Set the motors running
    ZB.SetMotor1(-driveRight * maxPower) # Rear right
    ZB.SetMotor2(-driveRight * maxPower) # Front right
    ZB.SetMotor3(-driveLeft  * maxPower) # Front left
    ZB.SetMotor4(-driveLeft  * maxPower) # Rear left
    # Wait for the time
    time.sleep(numSeconds)
    # Turn the motors off
    ZB.MotorsOff()

# Function to spin an angle in degrees
def PerformSpin(angle):
    if angle < 0.0:
        # Left turn
        driveLeft  = -1.0
        driveRight = +1.0
        angle *= -1
    else:
        # Right turn
        driveLeft  = +1.0
        driveRight = -1.0
    # Calculate the required time delay
    numSeconds = (angle / 360.0) * timeSpin360
    # Perform the motion
    PerformMove(driveLeft, driveRight, numSeconds)

# Function to drive a distance in meters
def PerformDrive(meters):
    if meters < 0.0:
        # Reverse drive
        driveLeft  = -1.0
        driveRight = -1.0
        meters *= -1
    else:
        # Forward drive
        driveLeft  = +1.0
        driveRight = +1.0
    # Calculate the required time delay
    numSeconds = meters * timeForward1m
    # Perform the motion
    PerformMove(driveLeft, driveRight, numSeconds)

# Function to drive fullspeed with turn options -> slowdown a side
def Drive(right,left):
    ZB.SetMotor1(-maxPower + right) # Rear right
    ZB.SetMotor2(-maxPower + right) # Front right
    ZB.SetMotor3(-maxPower + left) # Front left
    ZB.SetMotor4(-maxPower + left) # Rear left

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                print("up")
                return 1
        elif k=='\x1b[B':
                print("down")
                return 0
        else:
                print("not an arrow key!")

print('Press upper arrow to start the yetiborg')
print('Press lower arrow to stop the yetiborg')

# Loop to capture images and update the robot's movements
while True:
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    lower = (hueLow,saturationLow,valueLow)
    upper = (hueHigh,saturationHigh,valueHigh)
    mask = cv2.inRange(gray, lower, upper)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # If the target object is detected, move towards it until it is centered in the screen
    if len(cnts) > 0:
        # Get the center coordinates of the target object
        c = max(cnts, key=lambda el: cv2.contourArea(el))
        (x, y, w, h) = cv2.boundingRect(c)
        target_center = (x + w // 2, y + h // 2)

        # Get the center coordinates of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Calculate the angle to turn the robot to center the target object in the screen
        angle = (target_center[0] - screen_center[0]) / screen_center[0] * 45  # Adjust the angle range as needed

        # Turn the robot by the calculated angle
        PerformSpin(angle)

    # If the target object is not detected, turn in place until it is found
    else:
        PerformSpin(45)

    # Exit the loop if the target object is centered in the screen
    if abs(angle) < 5:
        image = camera.capture('picture.jpg')
        sleep(5)
        img = cv2.imread('picture.jpg')
        height, width, channels = img.shape 

        # Flip frame for right orientation
        imagePi = cv2.flip(img,0)
        imagePi = cv2.flip(imagePi,1)

        # create a canvas to display edges
        canvas = imagePi.copy()

        marker = find_marker(imagePi)
        # focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
        focalLength = 234.26
        print('Focal length is: ', focalLength)


        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            if ready == False:
                ready = get()
                print(ready)
        
            imageArray = np.array(rawCapture.array)
            marker = find_marker(imageArray)
            cms = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
            print(cms)

            # Main yetiborg code
            if ready == True:

                Drive(0,0)

                print('distance', cms)
                if cms < 10:
                    ZB.MotorsOff()
                    break
                else:
                    Drive(0.1,0.1)

            # Might need to change it to drive for a certain period of time and then read distance again
            # for more reliable distance measurement

        key = cv2.waitKey(1) & 0xFF
        # Clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # If the `q` key was pressed, break from the loop
        if key == ord('q'):
        	break

# no clue why this doesn't work
""" except KeyboardInterrupt:
    print('Interrupted')
    ZB.MotorsOff()
    cv2.destroyAllWindows()
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)
 """