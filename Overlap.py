import cv2
import numpy as np

# Function to perform non-maximum suppression
def non_max_suppression_fast(boxes, overlapThresh):
    if len(boxes) == 0:
        return []

    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    
    pick = []

    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]

    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    while len(idxs) > 0:
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        overlap = (w * h) / area[idxs[:last]]

        idxs = np.delete(idxs, np.concatenate(([last], np.where(overlap > overlapThresh)[0])))

    return boxes[pick].astype("int")

# Load the image
image = cv2.imread('path_to_your_image.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply a binary threshold to the image
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Find the contours
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

boxes = []

# Loop over the contours to find bounding boxes with area > 3000
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)
    area = w * h
    if area > 3000:
        boxes.append([x, y, x + w, y + h])

# Convert boxes to a NumPy array
boxes = np.array(boxes)

# Apply non-maximum suppression to merge overlapping rectangles
merged_boxes = non_max_suppression_fast(boxes, 0.3)

# Draw circles around the contours
for contour in contours:
    (x, y), radius = cv2.minEnclosingCircle(contour)
    center = (int(x), int(y))
    radius = int(radius)
    if cv2.contourArea(contour) > 3000:
        cv2.circle(image, center, radius, (0, 255, 0), 2)

# Display the image with circles
cv2.imshow('Contours with Circles', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
