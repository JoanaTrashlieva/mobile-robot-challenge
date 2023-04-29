#Based on the yetiRace code

# Standard imports
from __future__ import division
import sys,tty,termios,os
import time
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera

import ZeroBorg

# Camera settings
width = 640/2
height = 480/2
frameRate = 32

#Define object 
objectWidth = 7 #cm
initialDistance = 90 #cm

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

def distance_to_camera(objectWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    print("objectWidth", objectWidth)
    print("focalLength", focalLength)
    print("perWidth", perWidth)
    distance = (objectWidth * focalLength) / perWidth
    print(type(distance))
    print("distance", distance)
    return (objectWidth * focalLength) / perWidth

print('Press upper arrow to start the yetiborg')
print('Press lower arrow to stop the yetiborg')

# Main loop for the yetiborg
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if ready == False:
            ready = get()

        # Grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
        # Flip frame for right orientation
        imagePi = cv2.flip(frame.array,0)
        imagePi = cv2.flip(imagePi,1)
        canvas = imagePi.copy();

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

            focalLength = (bounds[1][0] * initialDistance) / objectWidth

            #distance
            cms = distance_to_camera(objectWidth, focalLength, bounds[1][0])
            # print(cms) #  always 90????? :( 
            
            M = cv2.moments(blob)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            xPos = center[0]
            cv2.circle(canvas, center, 5, (0,255,0), -1) # small circle to be the center of the object

        except (ValueError, ZeroDivisionError):
            pass

        # Main yetiborg code
        if ready == True:

            #Drive(0,0)

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
