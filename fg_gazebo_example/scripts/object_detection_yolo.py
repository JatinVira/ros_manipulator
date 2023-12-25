#!/usr/bin/env python3

"""
A simple ROS node that will subscribe to the camera topic,
Obtain the image from the camera, use a pretrained Pytorch model based on YOLOV8 to detect the objects,
then find the color and name of the object and publish it to a topic.
The code is written in object oriented style.
Use the following code for reference to make this code:
YOLO/yolov8_2.py
Also draw a rectangle around the object and display the image.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import time
import pandas as pd
from ultralytics import YOLO
from ultralytics.utils.plotting import (
    Annotator,
)  # ultralytics.yolo.utils.plotting is deprecated
import os

# Get the directory that the script file is in
script_dir = os.path.dirname(os.path.realpath(__file__))

# Join the script directory with the model filename
# model_path = os.path.join(script_dir, "Cubes_best.pt")
model_path = os.path.join(script_dir, "cubes2.pt")

# Global variables
camera_topic = "/camera/image"
region_of_interest = (20, 104, 168, 191)
confidence_threshold = 0.8
resize_size = (640, 480)


class ObjectDetection:
    def __init__(self):
        # Initialize the node
        rospy.init_node("object_detection_yolo", anonymous=True)
        print("Object Detection Node with YOLO V8 Initialized")

        self.init_complete = False

        # Define the bridge
        self.bridge = CvBridge()

        # Define the ROI
        self.roi = region_of_interest

        # Define the resize size for images
        self.resize_size = resize_size

        # Define the flag to check if the arm is ready
        self.ready = True  # False for laterrrr

        # Define the publisher
        self.pub = rospy.Publisher("box_color", String, queue_size=10)
        time.sleep(2)

        # Define another subscriber
        self.sub2 = rospy.Subscriber("/arm_status", String, self.callback2)
        time.sleep(2)

        # Define the YOLO model
        self.model = YOLO(model_path)
        print("YOLO Model Initialized")

        # Enable handshaking
        print("Handshaking with the Arm")
        self.pub.publish("Handshake Message")
        time.sleep(2)
        print("Handshake Delay Complete")

        self.init_complete = True
        print("Init set as TRUE")

        print("Connecting to the Arm")
        self.pub.publish("Connect Arm")
        time.sleep(2)
        print("Connection Delay Complete")

        # Define the subscriber
        self.sub = rospy.Subscriber(camera_topic, Image, self.callback)
        print("Subscribers Initialized")

        print("Object Detection Node Initialized")

    def callback2(self, data):
        # Callback function for the arm status subscriber
        if not self.init_complete:
            print("Waiting for Initialization to Complete")
            return
        print("Recieved Arm Status as {}".format(data.data))

        if data.data == "Ready":
            self.ready = True
            print("Set Arm Ready as TRUE")
        else:
            self.ready = False
            print("Set Arm Ready as FALSE")

    def callback(self, data):
        # Callback function for the camera subscriber
        if self.ready:
            try:
                # Convert the image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # resize the image
            cv_image = cv2.resize(cv_image, self.resize_size)

            # crop the image to the region of interest
            cv_image = cv_image[
                self.roi[1] : self.roi[1] + self.roi[3],
                self.roi[0] : self.roi[0] + self.roi[2],
            ]

            # Display the image
            cv2.imshow("ROI_Feed", cv_image)
            cv2.waitKey(1)

            # Call a function to detect the objects
            self.detect_objects(cv_image)

    def detect_objects(self, img):
        """
        Function to detect the largest object in the image.
        Args:
            img: The image in RGB format.
        """

        if not self.init_complete:
            print("Waiting for Initialization to Complete")
            return

        # Use the model to detect objects in the image without saving the results
        results = self.model.predict(img, save=False)

        # Extract bounding box data from the results
        bounding_boxes_data = results[0].boxes.data

        # Convert the bounding box data to a pandas DataFrame and cast to float
        bounding_boxes_df = pd.DataFrame(bounding_boxes_data.cpu()).astype("float")

        # Initialize variables to store the largest bounding box and its area
        largest_bounding_box = None
        largest_area = 0

        # Iterate over each row in the DataFrame (each bounding box)
        for _, bounding_box in bounding_boxes_df.iterrows():
            # Extract the coordinates of the bounding box
            x1 = int(bounding_box[0])
            y1 = int(bounding_box[1])
            x2 = int(bounding_box[2])
            y2 = int(bounding_box[3])

            # Calculate the area of the bounding box
            area = (x2 - x1) * (y2 - y1)

            # If this bounding box is larger than the current largest, update the largest bounding box and area
            if area > largest_area:
                largest_bounding_box = bounding_box
                largest_area = area

        # If a largest bounding box was found, find the label of the object in the bounding box
        if largest_bounding_box is not None:
            # Extract the coordinates of the bounding box
            x1 = int(largest_bounding_box[0])
            y1 = int(largest_bounding_box[1])
            x2 = int(largest_bounding_box[2])
            y2 = int(largest_bounding_box[3])

            # Draw a rectangle around the object
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0))

            # Extract the label of the object in the bounding box
            label = results[0].names[int(largest_bounding_box[5])]

            # Add the label to the image
            cv2.putText(
                img,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (36, 255, 12),
                2,
            )

            # Publish the color if label has color like Red, Blue, Green, Yellow
            if (
                self.ready
                and self.init_complete
                and label
                in [
                    "Red",
                    "Blue",
                    "Green",
                    "Yellow",
                ]
            ):
                print("Done Processing Valid Image so setting Arm Ready as False")
                self.ready = False
                print("Sending Color to Arm as {}".format(label))
                self.pub.publish(label)
                time.sleep(2)

        # Display the image with the bounding box and label
        cv2.imshow("Object_detection_feed", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    # Create an object of the class
    obj = ObjectDetection()

    # Keep the node alive
    rospy.spin()


# Path: fg_gazebo_example/scripts/object_detection_yolo.py
# refer the following code to make this code:
# YOLO/yolov8_2.py
