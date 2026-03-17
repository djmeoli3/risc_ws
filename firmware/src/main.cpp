#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//standard ROS 2 message for now to test the link
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup() { //heartbeat test
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "risc_teensy_node", "", &support);
  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teensy_heartbeat");

  msg.data = 0;
}

void loop() {
  msg.data++;
  rcl_publish(&publisher, &msg, NULL);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(100);
}