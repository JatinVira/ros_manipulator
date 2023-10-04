# ros_manipulator

A package to containarize all ROS related development files for the Manipulation Project

## Installation

1. Install Ubuntu 20.04:
Refer the [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) for more details.
Make sure to install Ubuntu 20.04 from the [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso) download page.

2. Install ROS Noetic:
Refer the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu) for more details.

3. Install dependent ROS Packages

    ```bash
    sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-moveit
    ```

4. Create a catkin workspace, initialize it and source it:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_init_workspace
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5. Clone the repository into the catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/JatinVira/ros_manipulator.git
    ```

6. Building the workspace:

    ```bash
    cd ~/catkin_ws/
    rosdep install -i --from-path src
    catkin build fg_gazebo_example
    source ~/.bashrc
    ```

## Usage

1. Launch the Gazebo simulation:

    ```bash
    roslaunch fg_gazebo_example fg_gazebo_example.launch
    ```

2. Run the script to control the robot:

    ```bash
    rosrun fg_gazebo_example move_viewpoints.py
    ```

## References

1. [Foxglove Studio](https://foxglove.dev/blog/simulating-robotic-scenarios-with-ros1-and-gazebo)
2. [ROS Control](http://wiki.ros.org/ros_control)
3. [ROS Controllers](http://wiki.ros.org/ros_controllers)
4. [MoveIt!](https://moveit.ros.org/)
5. [MoveIt! Tutorials](https://ros-planning.github.io/moveit_tutorials/)
6. [MoveIt! Tutorials - Move Group Python Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
