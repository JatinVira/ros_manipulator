#!/usr/bin/env python3


# Create a program to control a doosan robot using ROS Noetic and Python3
# This program will be used to test the robot's movement
# The robot will move to a set of predefined positions
# The script uses the moveit_commander package to control the robot
# The script uses the moveit_commander.RobotCommander class to get the robot's current state
# The script uses the moveit_commander.MoveGroupCommander class to control the robot's movement
# The script uses the moveit_commander.PlanningSceneInterface class to add objects to the planning scene
# The script uses the geometry_msgs.msg.Pose class to define the robot's position and orientation
# The script Will move the group "arm" 

# Import the necessary packages
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_robot_test', anonymous=True)

# Instantiate a RobotCommander object. This object is an interface to the robot as a whole
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the arm
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# Get the name of the reference frame for the robot
planning_frame = group.get_planning_frame()

# Print the name of the reference frame for the robot
print("============ Reference frame: %s" % planning_frame)

# Get the name of the end-effector link for the group
eef_link = group.get_end_effector_link()

# Print the name of the end-effector link for the group
print("============ End effector link: %s" % eef_link)

# Get a list of all the groups in the robot
group_names = robot.get_group_names()

# Print the list of all the groups in the robot
print("============ Robot Groups:", robot.get_group_names())

# Print the current state of the robot
print("============ Printing robot state")

# Print the current joint positions
print(robot.get_current_state())

# Print the current pose of the end-effector
print(group.get_current_pose().pose)

# Set the goal pose of the end-effector
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.0
pose_goal.position.z = 0.4
group.set_pose_target(pose_goal)

# Plan the trajectory to the goal pose
plan = group.go(wait=True)

# Print the trajectory
print(plan)

# Print the current pose of the end-effector
print(group.get_current_pose().pose)

# Print the current joint positions
print(robot.get_current_state())

# Print the current joint values
print(group.get_current_joint_values())

# Print the current pose of the end-effector
print(group.get_current_pose().pose)

