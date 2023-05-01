import cv2
import numpy as np
from yetiborg import YetiBorg

# Initialize YetiBorg
yb = YetiBorg()

# Set camera resolution
resolution = (640, 480)

# Initialize camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

# Set up the parameters for the blob detector
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 100
params.maxArea = 10000
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False
detector = cv2.SimpleBlobDetector_create(params)

while True:
    # Capture frame from camera
    _, frame = camera.read()

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold the frame
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Detect blobs in the thresholded frame
    keypoints = detector.detect(thresh)

    if len(keypoints) > 0:
        # Draw circle around each detected blob
        for keypoint in keypoints:
            x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
            cv2.circle(frame, (x, y), int(keypoint.size / 2), (0, 255, 0), 2)

        # Calculate distance to the nearest blob
        distance = yb.get_distance_to_object(keypoints[0].size)

        # Print the distance
        print('Distance:', distance)

    # Show the frame
    cv2.imshow('Frame', frame)

    # Check for exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
camera.release()
cv2.destroyAllWindows()
