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
xPos = 0
steerMultiplier = 0.8

# # Setup the ZeroBorg
# ZB = ZeroBorg.ZeroBorg()
# #ZB.i2cAddress = 0x44                   # Uncomment and change the value if you have changed the board address
# ZB.Init()
# if not ZB.foundChip:
#     boards = ZeroBorg.ScanForZeroBorg()
#     if len(boards) == 0:
#         print ('No ZeroBorg found, check you are attached :)')
#     else:
#         print ('No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress))
#         for board in boards:
#             print ('    %02X (%d)' % (board, board))
#         print ('If you need to change the IC address change the setup line so it is correct, e.g.')
#         print ('ZB.i2cAddress = 0x%02X' % (boards[0]))
#     sys.exit()
# #ZB.SetEpoIgnore(True)                  # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
# ZB.SetCommsFailsafe(False)              # Disable the communications failsafe
# ZB.ResetEpo()

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

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    cnts, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(cnts) > 0:
        # Find the contour with the largest area
        c = max(cnts, key=cv2.contourArea)
    else:
        print('No contours found in the image')
	# compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

KNOWN_DISTANCE = 24.0 #inch
KNOWN_WIDTH = 2.0 #inch

print('Press upper arrow to start the yetiborg')
print('Press lower arrow to stop the yetiborg')

# Main loop for the yetiborg
try:
    # Take a picture to calculate the focal length of the camera with a known object and distance
    # marker on object (center of obj) - W
    # distance from camera to marker - D
    # measure apparent width in pixels - P
    # get focal length of camera - F
    # F = (P x D) / W
    image = camera.capture('picture.jpg')
    sleep(5)
    img = cv2.imread('picture.jpg')
    height, width, channels = img.shape 
    #print(channels) # image has 3 channels

    # Flip frame for right orientation
    imagePi = cv2.flip(img,0)
    imagePi = cv2.flip(imagePi,1)

    marker = find_marker(imagePi)
    focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
    print("Focal length is: ",focalLength)
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if ready == False:
            ready = get()
            print(ready)

        imageArray = np.array(rawCapture.array)
        #img = cv2.imread(imageArray)
        marker = find_marker(imageArray)
        inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

        print("Distance from camera to object: ",inches)
        sleep(1)

        key = cv2.waitKey(1) & 0xFF
        # Clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # If the `q` key was pressed, break from the loop
        if key == ord('q'):
        	break

except KeyboardInterrupt:
    print('Interrupted')
    cv2.destroyAllWindows()
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)