// A Arduino sketch to setup a simple subscriber over ROS Serial to control a
// servo Author: Jatin Vira

// Logic:
// 1. Setup a subscriber to listen to the topic "box_color"
// 2. When a message is received, check the color and move the servo accordingly
// 3. There are a total of 6 Servo Motors, All of them move to move the arm to
// the desired position
// 4. There are a total of 4 colors, Blue, Green, Yellow and Pink
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

// Define the Servo Pins
#define SERVO_1_PIN 2
#define SERVO_2_PIN 3
#define SERVO_3_PIN 4
#define SERVO_4_PIN 5
#define SERVO_5_PIN 6
#define SERVO_6_PIN 7

// Define the Servo Objects
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;

// Define an array containing the servo angles for the pickup position
int pickup_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Define an array containing the servo angles for the intermediate position
int intermediate_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Define 4 arrays containing the servo angles for the drop position for each
// color
int blue_drop_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int green_drop_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int yellow_drop_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int pink_drop_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

// Define an array containing the servo angles for the default position
int default_position[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

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

// A function to move all the servos to their respective positions
// Arguments: An array containing the servo angles for each servo
void move_servos(int servo_angles[]) {
  // Move each servo to its respective angle
  servo_1.write(servo_angles[0]);
  servo_2.write(servo_angles[1]);
  servo_3.write(servo_angles[2]);
  servo_4.write(servo_angles[3]);
  servo_5.write(servo_angles[4]);
  servo_6.write(servo_angles[5]);
}

// position sequence for the color blue
void pos_seq_for_blue() {
  // Move the arm to the pickup position
  move_servos(pickup_position);
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  // Move the arm to the drop position
  move_servos(blue_drop_position);
  // Move the arm to the default position
  move_servos(default_position);
}

// position sequence for the color green
void pos_seq_for_green() {
  // Move the arm to the pickup position
  move_servos(pickup_position);
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  // Move the arm to the drop position
  move_servos(green_drop_position);
  // Move the arm to the default position
  move_servos(default_position);
}

// position sequence for the color yellow
void pos_seq_for_yellow() {
  // Move the arm to the pickup position
  move_servos(pickup_position);
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  // Move the arm to the drop position
  move_servos(yellow_drop_position);
  // Move the arm to the default position
  move_servos(default_position);
}

// position sequence for the color pink
void pos_seq_for_pink() {
  // Move the arm to the pickup position
  move_servos(pickup_position);
  // Move the arm to the intermediate position
  move_servos(intermediate_position);
  // Move the arm to the drop position
  move_servos(pink_drop_position);
  // Move the arm to the default position
  move_servos(default_position);
}

// function to pass the color as argument, and depending on the color,
// move the arm to the respective position sequence.
void move_arm(String color) {
  // Check the color
  if (color == "blue") {
    msg_to_send.data = "Blue Color Received";
    publish_status(msg_to_send);
    pos_seq_for_blue();
  } else if (color == "green") {
    msg_to_send.data = "Green Color Received";
    publish_status(msg_to_send);
    pos_seq_for_green();
  } else if (color == "yellow") {
    msg_to_send.data = "Yellow Color Received";
    publish_status(msg_to_send);
    pos_seq_for_yellow();
  } else if (color == "pink") {
    msg_to_send.data = "Pink Color Received";
    publish_status(msg_to_send);
    pos_seq_for_pink();
  } else {
    // Blink the onboard LED 3 times
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    // Publish the status of the arm as : Invalid Color Received
    msg_to_send.data = "Invalid Color Received";
    publish_status(msg_to_send);
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

// Setup Function
void setup() {
  // Setup the Servos
  setup_servos();

  // Setup the Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Blink the onboard LED 5 times to indicate that the Subscriber is connected
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // Publish the status of the arm as ready
  msg_to_send.data = "Arm Setup Complete";
  publish_status(msg_to_send);
}

// Loop Function
void loop() {
  // Spin the Subscriber
  nh.spinOnce();
}
