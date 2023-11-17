#!/usr/bin/env python3

"""
A simple ROS script to find the ROI of a camera frame
Open the camera, use the default ROI, and display the ROI
Also add a scope to dynamically change the ROI by dragging the mouse and then printing the new ROI on the terminal
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define the default ROI
roi = (0, 0, 640, 480)

# Define the resize size for images
resize_size = (640, 480)

# Initialize the ROS node
rospy.init_node("find_roi", anonymous=True)

# Open the camera
cap = cv2.VideoCapture(0)

# Create a bridge to convert ROS messages to OpenCV images
bridge = CvBridge()

# Set the loop rate
rate = rospy.Rate(10)

# Print Usage Instructions
print("Usage Instructions:")
print("Press 'q' to quit and print the ROI")
print("Press 'm' to change the ROI by dragging the mouse")
print("Then press 'q' to quit and print the new ROI")

# Loop until ROS is shutdown
while not rospy.is_shutdown():
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Resize the frame
    frame = cv2.resize(frame, resize_size)

    # Draw the ROI rectangle on the frame
    cv2.rectangle(
        frame, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), (255, 0, 0), 2
    )

    # Display the frame
    cv2.imshow("frame", frame)

    # Wait for a key press
    key = cv2.waitKey(1) & 0xFF

    # If the 'q' key is pressed, break from the loop and print the ROI
    if key == ord("q"):
        print("ROI: ", roi)
        break

    # Update the ROI if the mouse is dragged
    elif key == ord("m"):
        # Define the mouse callback function
        def mouse_callback(event, x, y, flags, param):
            """
            The mouse callback function
            Updates the ROI if the mouse is dragged
            """
            global roi
            if event == cv2.EVENT_LBUTTONDOWN:
                roi = (x, y, roi[2], roi[3])
            elif event == cv2.EVENT_LBUTTONUP:
                roi = (roi[0], roi[1], x - roi[0], y - roi[1])

        # Set the mouse callback function
        cv2.setMouseCallback("frame", mouse_callback)

        # print the ROI
        print(roi)

    # Sleep for the remainder of the loop
    rate.sleep()

# Release the camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
