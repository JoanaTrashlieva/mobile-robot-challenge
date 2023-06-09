
import cv2

# Load the image
image = cv2.imread('path/to/image.jpg')

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Threshold the image
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find contours in the image
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Loop over the contours
for contour in contours:
    # Find the minimum area rectangle
    rect = cv2.minAreaRect(contour)
    
    # New code is from here
    # Get the box points
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # Draw the box on the image
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

# Show the image
cv2.imshow('Image with Box', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
