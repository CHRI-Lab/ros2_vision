import cv2
import pyrealsense2
from realsense_depth import *

point = (400, 300)

# Update point using mouse position in real time
def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Track mouse position
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    # Show distance for a specific point
    cv2.circle(color_frame, point, 4, (0, 0, 255))  # Show a red circle targeting the point 
    distance = depth_frame[point[1], point[0]]  # [Y_pos, X_pos]

    cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Color frame", color_frame)

    # Update every milisecond, until press esc key
    key = cv2.waitKey(1)
    if key == 27:
        break