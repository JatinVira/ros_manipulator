#!/usr/bin/env python3

"""
A ROS script that combined a simple pub sub node with OpenCV to detect objects
The objects to be detcted are simple colored boxes.
There are only 4 colors: pink, green, blue, and yellow.
We have a predefined Region of Interest (ROI) for the entire image. 
Which can be defined as a rectangle at the beginning of the code.
The ROI is used to reduce the amount of data that we need to process and prevent false positives.
The program thus would simply subscribe to the camera topic, 
and then use OpenCV to detect the objects within the ROI.
After that detect the color of the object and publish the color to a topic.
Write the entire code in object oriented style.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class ObjectDetection:
    def __init__(self):
        # Initialize the node
        rospy.init_node("object_detection_cv", anonymous=True)

        # Define the publisher
        self.pub = rospy.Publisher("object_color", String, queue_size=10)

        # Define the subscriber
        self.sub = rospy.Subscriber("/camera/image", Image, self.callback)

        # Define the bridge
        self.bridge = CvBridge()

        # Define the ROI
        self.roi = (71, 279, 121, 116)

        # Define the resize size for images
        self.resize_size = (640, 480)

        # Define the color ranges in HSV
        self.pink_range = ((150, 100, 100), (180, 255, 255))
        self.green_range = ((40, 100, 100), (80, 255, 255))
        self.blue_range = ((100, 100, 100), (140, 255, 255))
        self.yellow_range = ((20, 100, 100), (40, 255, 255))

    def callback(self, data):
        """
        The callback function for the subscriber
        Simply converts the image from ROS format to OpenCV format,
        stores it in a variable, and then calls the detect_color function
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Call the detect_color function
        self.detect_color(cv_image)

    def detect_color(self, image):
        """
        Function to simply detect the color of the object in the image,
        Find the bounding box of the object,
        And then print out the bbox coordinates and the color of the object
        Parameters:
            image: The image in HSV format
        """
        # Crop the image to the ROI
        image = image[
            self.roi[1] : self.roi[1] + self.roi[3],
            self.roi[0] : self.roi[0] + self.roi[2],
        ]

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2GRAY)

        # Apply a Gaussian blur
        blur_image = cv2.GaussianBlur(gray_image, (7, 7), 0)

        # Apply a threshold
        _, thresh_image = cv2.threshold(
            blur_image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Find the contours
        contours, _ = cv2.findContours(
            thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # Find the largest contour, save it and delete the rest
        largest_contour = max(contours, key=cv2.contourArea)

        # Process only the largest contour
        contours = [largest_contour]

        # Draw the contours
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        # Find the bounding box of the contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Draw the bounding box
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)

        # Find the color of the object
        color = self.find_color(hsv_image[y : y + h, x : x + w])

        # Print the color and the bounding box coordinates
        print(color, x, y, w, h)

        # Publish the color
        self.pub.publish(color)

        # Show the image
        cv2.imshow("Image", image)
        cv2.waitKey(1)

    def find_color(self, image):
        """
        Function to find the color of the object in the image
        Parameters:
            image: The image in HSV format
        """
        # Find the average color in the image
        average_color = np.average(image, axis=(0, 1))

        # Convert the average color to HSV
        average_color = cv2.cvtColor(np.uint8([[average_color]]), cv2.COLOR_BGR2HSV)[0][
            0
        ]

        # Check if the average color is within the color ranges
        if self.in_range(average_color, self.pink_range):
            return "pink"
        elif self.in_range(average_color, self.green_range):
            return "green"
        elif self.in_range(average_color, self.blue_range):
            return "blue"
        elif self.in_range(average_color, self.yellow_range):
            return "yellow"
        else:
            return "unknown"

    def in_range(self, color, color_range):
        """
        Function to check if the color is within the color range
        Parameters:
            color: The color to be checked
            color_range: The color range
        """
        # Check if the color is within the color range
        if color[0] >= color_range[0][0] and color[0] <= color_range[1][0]:
            if color[1] >= color_range[0][1] and color[1] <= color_range[1][1]:
                if color[2] >= color_range[0][2] and color[2] <= color_range[1][2]:
                    return True

        return False

    def run(self):
        """
        The main function of the node
        """
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ObjectDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass
