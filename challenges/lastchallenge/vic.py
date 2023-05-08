# Standard imports
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
import YetiBorg as yb

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
    if len(contours) > 0:
        # Get the center coordinates of the target object
        c = max(cnts, key=lambda el: cv2.contourArea(el))
        (x, y, w, h) = cv2.boundingRect(c)
        target_center = (x + w // 2, y + h // 2)

        # Get the center coordinates of the screen
        screen_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Calculate the angle to turn the robot to center the target object in the screen
        angle = (target_center[0] - screen_center[0]) / screen_center[0] * 45  # Adjust the angle range as needed

        # Turn the robot by the calculated angle
        yb.turn(angle)

    # If the target object is not detected, turn in place until it is found
    else:
        yb.turn(45)

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

except KeyboardInterrupt:
    print('Interrupted')
    ZB.MotorsOff()
    cv2.destroyAllWindows()
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)

