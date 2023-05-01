import cv2
from YetiBorg import YetiBorg

# Create a YetiBorg object
yb = YetiBorg()

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply a Gaussian blur to the grayscale image
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Threshold the blurred image to create a binary image
    _, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Find the contours in the binary image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Get the largest contour
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        
        # Get the center point of the contour
        M = cv2.moments(c)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Get the size of the contour
        size = cv2.contourArea(c)
        
        # Calculate the distance to the object
        distance = yb.get_distance_to_object(size)
        
        # Display the distance on the image
        cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Draw a circle and a rectangle around the object
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
    
    # Display the image
    cv2.imshow('frame', frame)
    
    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
