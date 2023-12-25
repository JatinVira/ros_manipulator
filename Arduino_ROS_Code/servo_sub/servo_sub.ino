// A Arduino sketch to setup a simple subscriber over ROS Serial to control a
// servo

// Logic:
// 1. Setup a subscriber to listen to the topic "box_color"
// 2. When a message is received, check the color and move the servo accordingly
// 3. There are a total of 6 Servo Motors, All of them move to move the arm to
// the desired position
// 4. There are a total of 4 colors, Blue, Green, Yellow and Red
// 5. The Arm will have a common fixed pickup position for all the colors
// 6. The Arm will have 4 different drop positions for each color
// 7. The Arm will have all the 6 joint angles fixed for each drop position and
// pickup position
// 8. The Arm will also have additional default position that its in after the
// pickup and drop is complete
// 9. The Arm will also have an intermediate position that it will move to
// between the pickup and drop position
// 10. The program has to simply move the arm to the pickup position, then to
// the intermediate position, then to the drop position and then back to the
// default position
// 11. The program has to check the color of the box and move the arm
// accordingly.
// 12. Also setup a publisher to publish the current status of the arm on the
// topic "arm_status"

// Include a few Arduino Library's
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

// Define the number of Servo Motors
#define NUM_SERVOS 6

// Define the Servo Open and Close Angles for the Gripper
#define GRIPPER_OPEN_ANGLE 95    // Degrees
#define GRIPPER_CLOSE_ANGLE 140  // Degrees

// Define toggle values for the gripper
#define GRIPPER_OPEN true
#define GRIPPER_CLOSE false

// Define the Servo Pins
#define SERVO_1_PIN 2
#define SERVO_2_PIN 3
#define SERVO_3_PIN 4
#define SERVO_4_PIN 5
#define SERVO_5_PIN 6
#define SERVO_6_PIN 7

// Define the time periods for various delays
#define DELAY_BW_SERVOS 800     // ms
#define DELAY_GRIPPER 100       // ms
#define DELAY_BW_POSE_SEQ 1000  // ms
#define DELAY_LOOP 10           // ms

// Define the Servo Objects
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;

// Servo Angles for the pickup position
int pickup_position[NUM_SERVOS] = {140, 120, 142, 155, 90, GRIPPER_OPEN_ANGLE};

// Servo Angles for the intermediate position
int intermediate_position[NUM_SERVOS] = {90, 90, 90,
                                         90, 90, GRIPPER_CLOSE_ANGLE};

// Servo Angles for the default position
int default_position[NUM_SERVOS] = {98, 120, 135, 165, 90, GRIPPER_OPEN_ANGLE};
// Servo Angles for the drop position for each color
int blue_drop_position[NUM_SERVOS] = {63,  112, 140,
                                      142, 90,  GRIPPER_OPEN_ANGLE};
int green_drop_position[NUM_SERVOS] = {66,  100, 145,
                                       168, 90,  GRIPPER_OPEN_ANGLE};
int yellow_drop_position[NUM_SERVOS] = {48,  105, 148,
                                        144, 90,  GRIPPER_OPEN_ANGLE};
int red_drop_position[NUM_SERVOS] = {48, 97, 147, 176, 90, GRIPPER_OPEN_ANGLE};

// Create a Node Handle
ros::NodeHandle nh;

// Callback Function for the Subscriber
void color_callback(const std_msgs::String &msg) {
  // move the arm to the respective position sequence
  move_arm(msg.data);
}

// Create a Subscriber Object to recieve the color of the box
ros::Subscriber<std_msgs::String> sub("box_color", &color_callback);

// Create a Publisher Object to publish the current status of the arm
std_msgs::String status_msg;
ros::Publisher status_pub("arm_status", &status_msg);

// Buffer to store the status message
std_msgs::String msg_to_send;

// A function to publish the status of the arm on the topic
void publish_status(std_msgs::String status) {
  status_msg.data = status.data;
  status_pub.publish(&status_msg);
}

// A function to move the gripper to close or open position
// Arguments: A boolean value to indicate whether to open or close the gripper
void move_gripper(bool open) {
  // Check if the gripper is to be opened or closed
  if (open) {
    // Open the gripper
    servo_6.write(GRIPPER_OPEN_ANGLE);
    delay(DELAY_GRIPPER);
  } else {
    // Close the gripper
    servo_6.write(GRIPPER_CLOSE_ANGLE);
    delay(DELAY_GRIPPER);
  }
}

// A function to move all the servos to their respective positions
// Arguments: An array containing the servo angles for each servo
// Move the servos one after the other and wait for 100ms after each servo is
// moved
void move_servos(int servo_angles[]) {
  // Check if intermediate poisition is passed as argument, if yes, do nothing
  // with the 6th servo Rest everything is same
  if (servo_angles == intermediate_position) {
    // Move each servo to its respective angle
    // servo_1.write(servo_angles[0]);
    // delay(DELAY_BW_SERVOS);
    servo_2.write(servo_angles[1]);
    delay(DELAY_BW_SERVOS);
    servo_3.write(servo_angles[2]);
    delay(DELAY_BW_SERVOS);
    servo_4.write(servo_angles[3]);
    delay(DELAY_BW_SERVOS);
    servo_5.write(servo_angles[4]);
    delay(DELAY_BW_SERVOS);
  } else {
    // Move each servo to its respective angle
    servo_1.write(servo_angles[0]);
    delay(DELAY_BW_SERVOS);
    servo_2.write(servo_angles[1]);
    delay(DELAY_BW_SERVOS);
    servo_3.write(servo_angles[2]);
    delay(DELAY_BW_SERVOS);
    servo_4.write(servo_angles[3]);
    delay(DELAY_BW_SERVOS);
    servo_5.write(servo_angles[4]);
    delay(DELAY_BW_SERVOS);
    servo_6.write(servo_angles[5]);
    delay(DELAY_BW_SERVOS);
  }
}

// position sequence for the color blue
void pos_seq_for_blue() {
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the pickup position
  move_servos(pickup_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Pickup Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Close the gripper
  move_gripper(GRIPPER_CLOSE);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Gripper Close";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the drop position
  move_servos(blue_drop_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Drop Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the default position
  move_servos(default_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Default Pos";
  publish_status(msg_to_send);
  delay(1000);

  msg_to_send.data = "FN Complete";
  publish_status(msg_to_send);
  delay(1000);
}

// position sequence for the color green
void pos_seq_for_green() {
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the pickup position
  move_servos(pickup_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Pickup Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Close the gripper
  move_gripper(GRIPPER_CLOSE);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Gripper Close";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the drop position
  move_servos(green_drop_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Drop Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the default position
  move_servos(default_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Default Pos";
  publish_status(msg_to_send);
  delay(1000);

  msg_to_send.data = "FN Complete";
  publish_status(msg_to_send);
  delay(1000);
}

// position sequence for the color yellow
void pos_seq_for_yellow() {
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the pickup position
  move_servos(pickup_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Pickup Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Close the gripper
  move_gripper(GRIPPER_CLOSE);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Gripper Close";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the drop position
  move_servos(yellow_drop_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Drop Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the default position
  move_servos(default_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Default Pos";
  publish_status(msg_to_send);
  delay(1000);

  msg_to_send.data = "FN Complete";
  publish_status(msg_to_send);
  delay(1000);
}

// position sequence for the color red
void pos_seq_for_red() {
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the pickup position
  move_servos(pickup_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Pickup Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Close the gripper
  move_gripper(GRIPPER_CLOSE);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Gripper Close";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the drop position
  move_servos(red_drop_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Drop Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Interm Pos";
  publish_status(msg_to_send);
  delay(1000);

  // Move the arm to the default position
  move_servos(default_position);
  delay(DELAY_BW_POSE_SEQ);
  msg_to_send.data = "Default Pos";
  publish_status(msg_to_send);
  delay(1000);

  msg_to_send.data = "FN Complete";
  publish_status(msg_to_send);
  delay(1000);
}

// function to pass the color as argument, and depending on the color,
// move the arm to the respective position sequence.
void move_arm(String color) {
  // Check the color
  if (color == "Blue") {
    msg_to_send.data = "Blue Received";
    publish_status(msg_to_send);
    delay(1000);
    pos_seq_for_blue();
    msg_to_send.data = "Ready";
    publish_status(msg_to_send);
    delay(1000);
    return;
  } else if (color == "Green") {
    msg_to_send.data = "Green Received";
    publish_status(msg_to_send);
    delay(1000);
    pos_seq_for_green();
    msg_to_send.data = "Ready";
    publish_status(msg_to_send);
    delay(1000);
    return;
  } else if (color == "Yellow") {
    msg_to_send.data = "Yellow Received";
    publish_status(msg_to_send);
    pos_seq_for_yellow();
    msg_to_send.data = "Ready";
    publish_status(msg_to_send);
    delay(1000);
    return;
  } else if (color == "Red") {
    msg_to_send.data = "Red Received";
    publish_status(msg_to_send);
    delay(1000);
    pos_seq_for_red();
    msg_to_send.data = "Ready";
    publish_status(msg_to_send);
    delay(1000);
    return;
  } else if (color == "Connect Arm") {
    msg_to_send.data = "Ready";
    publish_status(msg_to_send);
    delay(1000);
    return;
  } else {
    // Publish the status of the arm as : Invalid Color Received
    msg_to_send.data = "Invalid Color Received";
    publish_status(msg_to_send);
    delay(1000);
    return;
  }
}

// function to setup the servos
void setup_servos() {
  // Attach the servos to their respective pins
  servo_1.attach(SERVO_1_PIN);
  servo_2.attach(SERVO_2_PIN);
  servo_3.attach(SERVO_3_PIN);
  servo_4.attach(SERVO_4_PIN);
  servo_5.attach(SERVO_5_PIN);
  servo_6.attach(SERVO_6_PIN);
}

void setup_ROS() {
  // Setup the Subscriber
  nh.initNode();
  nh.subscribe(sub);

  // Setup the Publisher
  nh.advertise(status_pub);

  // Wait for the Subscriber to connect
  while (!nh.connected()) {
    nh.spinOnce();
  }
}

void test_arm() {
  // Test the default position
  move_servos(default_position);

  // Test the pickup position
  move_servos(pickup_position);

  // Test the intermediate position
  move_servos(intermediate_position);

  // Test the drop position for blue
  move_servos(blue_drop_position);

  // Test the drop position for green
  move_servos(green_drop_position);

  // Test the drop position for yellow
  move_servos(yellow_drop_position);

  // Test the drop position for red
  move_servos(red_drop_position);
}

// Setup Function
void setup() {
  // Setup the Servos
  setup_servos();

  // Setup the Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup ROS
  setup_ROS();

  // Test the Arm
  // test_arm();

  // Initialize the arm position to the default position
  move_servos(default_position);
  delay(1000);

  // Blink the onboard LED 5 times to indicate that the Subscriber is connected
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Publish the status of the arm as : Ready
  msg_to_send.data = "Ready";
  publish_status(msg_to_send);
  delay(1000);
}

// Loop Function
void loop() {
  // Spin the Subscriber
  nh.spinOnce();

  // Delay for 10ms
  delay(DELAY_LOOP);
}
