<!-- Create a Readme file for me to demonstrate how to setup ROS serial with an Arduino -->

## Connect Arduino to ROS

### Step 1: Install ROS serial on your computer

```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

### Step 2: Upload the ROS serial firmware to your Arduino

```text
Open Arduino IDE
File -> Open -> catkin_ws/src/ros_manipulator/Arduino_ROS_Code/servo_sub/servo_sub.ino
Select the correct port and board in the Tools menu
Tools -> Board -> Arduino Uno
Tools -> Port -> /dev/ttyACM0 (or something similar)
Upload
```

### Step 3: Start the ROS Master

```bash
roscore
```

### Step 4: Start the ROS serial node by starting our Launch file

```bash
roslaunch fg_gazebo_example start_robot.launch
```

```text
Note: 
1. This will start the ROS serial node and connect it to the Arduino
2. The launch file accepts an argument called port (default is /dev/ttyACM0)
3. If you are using a different port, you can specify it as follows:
roslaunch fg_gazebo_example start_robot.launch port:=/dev/ttyACM1
```

### Step 5: Print the Data recieved from the Arduino

```bash
rostopic echo /arm_status
```

```text
Note: The data recieved from the Arduino can be interpreted to obtain the state of the Arm
```

### Step 6: MANUAL CONTROL

DONT DO THIS NOW:
Control the Servo Motor externally by publishing color data to topic

```bash
rostopic pub /box_color std_msgs/String "data: 'blue'"
```

```text
Note: There are only 4 colors that the servo motor can be controlled by: pink, green, blue, and yellow
Replace 'blue' with any of the other colors to control the servo motor
The Arduino will crash if new messages are published to the topic before the previous message is processed
```

### STEP 7: AUTONOMOUS CONTROL

### STEP 7.1: Find the Region of Interest (ROI)

```bash
roscd fg_gazebo_example
cd scripts
python3 find_roi.py
```

### Step 7.2: Run the Object Detection Node

1. Start the Camera Node

```bash
rosrun fg_gazebo_example start_camera.py
```

2. Start the Color Detection Node

```bash
rosrun fg_gazebo_example object_detection_yolo.py
```
