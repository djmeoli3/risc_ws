#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <AccelStepper.h>
#include "config.h"

AccelStepper zaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);

enum State { IDLE, MOVING, EMERGENCY_STOP };
volatile State currentState = IDLE;

// ROS 2 Objects
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
bool micro_ros_connected = false;

void applyHardStop() {
    zaxis.setSpeed(0);
    zaxis.moveTo(zaxis.currentPosition());
    
    float backoff = 4.0 * STEPS_PER_MM; //
    
    if (digitalRead(LIMIT_SWITCH_FAR) == HIGH) {
        zaxis.moveTo(zaxis.currentPosition() - (long)round(backoff)); 
    } else if (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) {
        zaxis.moveTo(zaxis.currentPosition() + (long)round(backoff));
    }
    
    zaxis.setMaxSpeed(HOMING_SPEED_SCALED);
    while(zaxis.distanceToGo() != 0) {
        zaxis.run();
    }
    
    zaxis.setCurrentPosition(0);
    currentState = IDLE;
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__String * msg_received = (const std_msgs__msg__String *)msgin;
    if (msg_received->data.data == NULL) return;
    
    String input = String(msg_received->data.data);
    input.trim();
    
    if (input.startsWith("z")) {
        float targetMM = input.substring(1).toFloat();
        zaxis.setMaxSpeed(MOVE_SPEED_SCALED);
        zaxis.moveTo((long)round(targetMM * STEPS_PER_MM));
        currentState = MOVING;
    }
    else if (input == "x") {
        currentState = EMERGENCY_STOP;
        zaxis.setSpeed(0); 
        zaxis.moveTo(zaxis.currentPosition()); //kills target buffr
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);
    pinMode(LEAD_ENA_PIN, OUTPUT); 
    digitalWrite(LEAD_ENA_PIN, LOW); //
    digitalWrite(LED_PIN, HIGH);
    
    zaxis.setMaxSpeed(MOVE_SPEED_SCALED); 
    zaxis.setAcceleration(2000.0 * (float)MICROSTEPS);

    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        rclc_node_init_default(&node, NODE_NAME, "", &support);
        msg_sub.data.capacity = 100;
        msg_sub.data.data = (char*) malloc(msg_sub.data.capacity);
        rclc_subscription_init_default(&subscriber, &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_NAME);
        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
        micro_ros_connected = true;
    }
}

void loop() {
    if (!micro_ros_connected) return;

    if (currentState == MOVING) {
        //constant check safety limits
        if (digitalRead(LIMIT_SWITCH_FAR) == HIGH || digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) {
            applyHardStop();
        } 
        else if (zaxis.distanceToGo() == 0) {
            currentState = IDLE;
        }
    }

    zaxis.run();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}