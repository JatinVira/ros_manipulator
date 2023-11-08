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

4. Install catkin tools:

    ```bash
    sudo apt-get install python3-catkin-tools
    ```

5. Create a catkin workspace, initialize it, build it and then source it:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin build
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

6. Setup SSH Keys for GitHub:

    First open a new terminal and generate a new SSH key:

    ```bash
    cd
    ssh-keygen -t ed25519 -C "your_email@example.com"
    ```

    When you're prompted to "Enter a file in which to save the key," press Enter. This accepts the default file location.
    Keep pressing enter until the key is generated.

    View the contents of the public key file:

    ```bash
    cat ~/.ssh/id_ed25519.pub
    ```

    Copy the contents of the file and add it to your GitHub account.
    [Set Keys here](https://github.com/settings/keys)

    Refer the [GitHub SSH Key Setup Guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) for more details.

7. Clone the repository into the catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone git@github.com:JatinVira/ros_manipulator.git
    ```

8. Building the workspace:

    ```bash
    cd ~/catkin_ws/
    rosdep install -i --from-path src
    catkin build fg_gazebo_example
    source ~/.bashrc
    ```

9. Generating Simulation Models:

    ```bash
    cd ~/catkin_ws/src/ros_manipulator/fg_gazebo_example/
    python3 scripts/create_sdf_and_config.py
    ```

## Usage

1. Launch the Gazebo simulation:

    ```bash
    roslaunch fg_gazebo_example simulation.launch
    ```

2. Run the script to control the robot:

    ```bash
    rosrun fg_gazebo_example move_viewpoints.py
    ```

##Important Notes

1. Copy paste the entire model folder in the `~/.gazebo/models` directory for the simulation to work.


## References

1. [Foxglove Studio](https://foxglove.dev/blog/simulating-robotic-scenarios-with-ros1-and-gazebo)
2. [ROS Control](http://wiki.ros.org/ros_control)
3. [ROS Controllers](http://wiki.ros.org/ros_controllers)
4. [MoveIt!](https://moveit.ros.org/)
5. [MoveIt! Tutorials](https://ros-planning.github.io/moveit_tutorials/)
6. [MoveIt! Tutorials - Move Group Python Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
