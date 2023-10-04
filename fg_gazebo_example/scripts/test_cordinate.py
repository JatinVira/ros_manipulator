#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
import numpy as np
import math


def main():
    # Create a ros node and use moveit commander to ask the robot arm to move to a desired position.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_fk_demo", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = robot.get_group_names()[0]
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()

    # Add the ground plane as collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)

    # Get the current end-effector pose
    current_pose = move_group.get_current_pose().pose

    # Calculate the target position
    target_position = [
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z + 0.05,
    ]

    target_orientation = [0, 0, 0, 1]
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2]
    target_pose.pose.orientation.x = target_orientation[0]
    target_pose.pose.orientation.y = target_orientation[1]
    target_pose.pose.orientation.z = target_orientation[2]
    target_pose.pose.orientation.w = target_orientation[3]

    print(f"Moving to the target position: {target_position}")
    # Demand the robot arm to move to the target position

    # Try multiple times in case of planning failure
    for _ in range(5):
        move_group.set_pose_target(target_pose)
        success, trajectory, planning_time, error_code = move_group.plan()

        if success:
            move_group.execute(trajectory, wait=True)
            print("Done!")
            break
        else:
            print("Planning failed. Retrying...")
    else:
        print("Unable to find a valid path after multiple attempts.")


if __name__ == "__main__":
    main()
