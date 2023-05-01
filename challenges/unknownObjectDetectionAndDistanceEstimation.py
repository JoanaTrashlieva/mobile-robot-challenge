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

# Initializing the camera
camera = PiCamera()
camera.awb_mode = 'auto'
camera.resolution = (int(width), int(height))
buffer_size = width * height * 3
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

# Create variable for focal length - TODO needs recalculation in better 
focalLength = 99.93995257786342

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

def find_focal_length():
    camera.capture(rawCapture, format="bgr", use_video_port=True)
    image = rawCapture.array
    img_flipped = np.fliplr(image)
    img_flipped_again = np.fliplr(img_flipped)

    img_hsv = cv2.cvtColor(img_flipped_again, cv2.COLOR_BGR2HSV)

    lower = np.array([100], dtype=np.uint8)
    upper = np.array([200], dtype=np.uint8)
    mask = cv2.inRange(img_hsv, lower, upper)

    # Create the contours and find the center
    try:
        # NB: using _ as the variable name for the output, as it is not used
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        blob = max(contours, key=lambda el: cv2.contourArea(el))
        #print(blob)

        # returns [center,size][angle]
        bounds = cv2.minAreaRect(blob)
        print("Bounds: ", bounds[1][0])
        # cv2.minAreaRect() gives Box2D structure and not rotated rectangle around object

        # focalLength = (bounds[1][0] * initialDistance) / objectWidth

        print("Focal length: ", focalLength)
        camera.close()
    except (ValueError, ZeroDivisionError):
        pass


def distance_to_camera(objectWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    distance = (objectWidth * focalLength) / perWidth
    return (objectWidth * focalLength) / perWidth

print('Press upper arrow to start the yetiborg')
print('Press lower arrow to stop the yetiborg')

# Main loop for the yetiborg
try:
    # print("Focal length: ", focalLength)
    # Define focal length, but run only once
    # find_focal_length()

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if ready == False:
            ready = get()

        # Grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
        # Flip frame for right orientation
        imagePi = cv2.flip(frame.array,0)
        imagePi = cv2.flip(imagePi,1)
        canvas = imagePi.copy()

        # Convert to hsv for better image processing
        img_hsv = cv2.cvtColor(imagePi, cv2.COLOR_BGR2HSV)

        # Generate mask with pre defined colors -> lower and upper bound
        lower = (hueLow,saturationLow,valueLow)
        upper = (hueHigh,saturationHigh,valueHigh)
        mask = cv2.inRange(img_hsv, lower, upper)

        # Create the contours and find the center
        try:
            # NB: using _ as the variable name for the output, as it is not used
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            #print(blob)

            # returns [center,size][angle]
            bounds = cv2.minAreaRect(blob)
            print("Bounds: ", bounds[1][0])
            # cv2.minAreaRect() gives Box2D structure and not rotated rectangle around object

            # focalLength = (bounds[1][0] * initialDistance) / objectWidth
            # bounds[1][0] ? is that how cv2 calculates the width of the object?

            #distance
            cms = distance_to_camera(objectWidth, focalLength, bounds[1][0])
            print("Distance: ",cms) #  always 90????? :( 
            # doesn't save the focalLength as a global variable ? it keeps recalculating the focalLength 
            
            M = cv2.moments(blob)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            xPos = center[0]
            cv2.circle(canvas, center, 5, (0,255,0), -1) # small circle to be the center of the object

        except (ValueError, ZeroDivisionError):
            pass

        # Main yetiborg code
        if ready == True:
            # Dynamic speed adjustment based on how far the line is from the center
            something = abs(xPos - (width/2))
            adjustValue = something/width
           
        # Displaying the windows for debuging
        if displayWindows == True:
            cv2.imshow("imagePi", imagePi)
            cv2.imshow('canvas',canvas)
            cv2.imshow('mask',mask)

        key = cv2.waitKey(1) & 0xFF
        # Clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # camera.close()
        # If the `q` key was pressed, break from the loop
        if key == ord('q'):
        	break

except KeyboardInterrupt:
    print('Interrupted')
    #ZB.MotorsOff()
    cv2.destroyAllWindows()
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)
