import rosbag
import cv2
from cv_bridge import CvBridge
import os
import argparse


def convert_bag_to_images(bag_file, topic_name, output_folder):
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Initialize CvBridge
    bridge = CvBridge()

    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except Exception as e:
                print(e)
                continue

            # Extract the timestamp and construct a filename
            timestamp = int(f"{msg.header.stamp.secs}{msg.header.stamp.nsecs}")
            filename = f"{timestamp}.png"

            # Save the image
            cv2.imwrite(os.path.join(output_folder, filename), cv_image)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert a ROS bag file to a folder with images."
    )
    parser.add_argument("bag_file", type=str, help="Path to the ROS bag file")
    parser.add_argument(
        "topic_name", type=str, help="Name of the topic containing images"
    )
    parser.add_argument("output_folder", type=str, help="Path to the output folder")

    args = parser.parse_args()

    convert_bag_to_images(args.bag_file, args.topic_name, args.output_folder)
