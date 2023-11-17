#!/usr/bin/env python3

"""
A ROS script to open a camera and publish images to ROS
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Open the camera
cap = cv2.VideoCapture(0)

# Create a publisher for the image
pub = rospy.Publisher("camera/image", Image, queue_size=10)

# Create a bridge to convert ROS messages to OpenCV images
bridge = CvBridge()

# Initialize the ROS node
rospy.init_node("camera", anonymous=True)

# Set the loop rate
rate = rospy.Rate(10)

# Loop until ROS is shutdown
while not rospy.is_shutdown():
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to a ROS image
    try:
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Publish the ROS image
    pub.publish(ros_image)

    # Sleep for the remainder of the loop
    rate.sleep()

# Release the camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
