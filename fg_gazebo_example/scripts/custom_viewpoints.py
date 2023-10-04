#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


def main():
    # Initialize a ROS node with a unique name (you can choose any name)
    rospy.init_node("moveit_example", anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = robot.get_group_names()[0]
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = moveit_commander.PlanningSceneInterface()

    # Add the ground plane as a collision object
    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane", plane_pose)

    move_group.set_max_acceleration_scaling_factor(1)
    move_group.set_max_velocity_scaling_factor(1)

    while True:  # Infinite loop
        target_joint_states = [
            [1.207, -1.731, 1.205, -1.642, -1.355, -0.295],
            [2.261, -1.088, 0.997, -1.732, -2.251, 0.601],
        ]
        for target_state in target_joint_states:
            success = False
            move_group.set_joint_value_target(target_state)
            success, trajectory, planning_time, error_code = move_group.plan()
            move_group.execute(trajectory, wait=True)


if __name__ == "__main__":
    main()
