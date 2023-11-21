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
model_path = os.path.join(script_dir, "Cubes_best.pt")

# Global variables
# model_path = "Cubes_best.pt"
camera_topic = "/camera/image"
region_of_interest = (82, 58, 150, 150)
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

        # Define the color ranges in HSV
        self.pink_range = ((150, 100, 100), (180, 255, 255))
        self.green_range = ((40, 100, 100), (80, 255, 255))
        self.blue_range = ((102, 89, 85), (114, 209, 255))
        self.yellow_range = ((20, 100, 100), (40, 255, 255))

        # Define the publisher
        # self.pub = rospy.Publisher("box_color", String, queue_size=10)
        # time.sleep(4)

        # Define another subscriber
        # self.sub2 = rospy.Subscriber("/arm_status", String, self.callback2)
        # time.sleep(4)

        # Enable handshaking
        # print("Handshaking with the Arm")
        # self.pub.publish("Handshake Message")

        # Define the subscriber
        self.sub = rospy.Subscriber(camera_topic, Image, self.callback)

        # Define the YOLO model
        self.model = YOLO(model_path)

        self.init_complete = True
        print("YOLO Model Initialized")

    # def callback2(self, data):
    #     # Callback function for the arm status subscriber
    #     if data.data == "Ready":
    #         self.ready = True
    #     else:
    #         self.ready = False

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

            # Convert the image to RGB format
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Display the image
            # cv2.imshow("Image", cv_image)
            # cv2.waitKey(1)

            # Call a function to detect the objects
            self.detect_objects(cv_image)

    def detect_objects(self, img):
        # Function to detect the objects in the image
        # img is the image in RGB format

        # Detect the largest object in the image, and print the color and name of the object
        # without saving the results
        results = self.model(img, save=False)

        a = results[0].boxes.data
        px = pd.DataFrame(a).astype("float")
        list = []

        for index, row in px.iterrows():
            x1 = int(row[0])
            y1 = int(row[1])
            x2 = int(row[2])
            y2 = int(row[3])
            d = int(row[5])
            print(d)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0))

        for re in results:
            annotator = Annotator(img)
            boxes = re.boxes.cpu().numpy()
            ab = []

            # get boxes on cpu in numpy
            for box in boxes:
                # iterate boxes
                r = box.xyxy[0].astype(int)
                # get corner points as int
                Klass = re.names[int(box.cls[0])]
                # Get Class names
                print(r, Klass)

                d = r.tolist()
                ab.append(d)
                b = box.xyxy[0]
                # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)])
                print(d)

            boxes = re.boxes.cpu().numpy()
            # Get bounding boxes as images saved
            for i, box in enumerate(boxes):
                r = box.xyxy[0].astype(int)
                crop = img[r[1] : r[3], r[0] : r[2]]
                # cv2.imwrite('project/cropped/'+str(i) + ".png", crop)
                print("__________")

        img = annotator.result()
        cv2.imshow("YOLO V8 Detection", img)
        cv2.waitKey(1)

        # Dont publish anything for now and move to next part of code
        # Publish the color
        #
        # if self.ready and self.init_complete:
        #    print("Done Processing Image so setting Arm Ready as False")
        #   self.ready = False
        # print("Sending Color to Arm as {}".format(color))
        # self.pub.publish(color)
        # time.sleep(2)
        # Show the image
        # cv2.imshow("Image", img)
        # cv2.waitKey(1)
        #

        # display the image
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    # Create an object of the class
    obj = ObjectDetection()

    # Keep the node alive
    rospy.spin()


# Path: fg_gazebo_example/scripts/object_detection_yolo.py
# refer the following code to make this code:
# YOLO/yolov8_2.py
