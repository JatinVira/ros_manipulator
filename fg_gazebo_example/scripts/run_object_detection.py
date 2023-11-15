#!/usr

"""
A ROS script to detect objects in an image using YOLOv3
The script subscribes to the camera/image topic and 
publishes the detected objects to the camera/object topic
The script uses a pre-trained YOLOv3 model (pytorch) named Cubes_best.pt 
And it also has a region of interest (ROI) to reduce the search space and to prevent false positives
The ROI is a rectangle and it should have the provision to be defined at the start of the script
The script also uses ULTRALYTICS to detect objects in the ROI
Use the image topic to get the image from the camera and use that image to detect objects
Write a Object oriented code to make it more readable and easy to understand
"""
