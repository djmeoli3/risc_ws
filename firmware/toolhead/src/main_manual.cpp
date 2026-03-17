#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"

// MOTOR & SERVO OBJECTS
AccelStepper swing(AccelStepper::DRIVER, SWING_STEP_PIN, SWING_DIR_PIN);
AccelStepper rotate(AccelStepper::DRIVER, ROTATE_STEP_PIN, ROTATE_DIR_PIN);
Servo gripper;

// STATE VARIABLES
volatile float currentSwingRaw = 0;
volatile float currentRotateRaw = 0;
float targetSwingRaw = -1.0; 
float targetRotateRaw = -1.0; 
int f1, f2, f3, f4;

bool micro_ros_connected = false;

// ROS 2 OBJECTS
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String msg_sub;
std_msgs__msg__String msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float readRawEncoder(TwoWire &bus) {
    bus.beginTransmission(AS5600_ADDR);
    bus.write(0x0E); 
    if (bus.endTransmission() != 0) return -999;
    
    bus.requestFrom(AS5600_ADDR, (uint8_t)2);
    if (bus.available() >= 2) {
        uint16_t raw = (bus.read() << 8) | bus.read();
        return (raw * 360.0) / 4096.0;
    }
    return -999;
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    String input = String(msg->data.data);
    input.trim();
    
    char cmd = input.charAt(0);
    float val = input.substring(1).toFloat();

    switch (cmd) {
        case 's': 
            if (val == 1) targetSwingRaw = SWING_UP_RAW;   
            else if (val == 0) targetSwingRaw = SWING_DOWN_RAW; 
            break;

        case 'r': {
            float proposed = val;
            while (proposed >= 360.0) proposed -= 360.0;
            while (proposed < 0.0) proposed += 360.0;
            //deadzone protection
            if (proposed > 70.0 && proposed < 180.0) {
                proposed = (proposed < 125.0) ? 70.0 : 180.0;
            }
            targetRotateRaw = proposed;
            break;
        }

        case 'g': 
            if (val == 1) gripper.write(179);
            else if (val == 0) gripper.write(80);
            else gripper.write((int)val);
            break;
        
        case 'e': 
            analogWrite(EXTRACTOR_PIN, constrain((int)val, 0, 255)); 
            break;

        case 'x': 
            targetSwingRaw = -1.0; 
            targetRotateRaw = currentRotateRaw; 
            swing.stop(); rotate.stop();
            analogWrite(EXTRACTOR_PIN, 0);
            break;
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(SWING_ENA_PIN, OUTPUT);
    pinMode(ROTATE_ENA_PIN, OUTPUT);
    pinMode(EXTRACTOR_PIN, OUTPUT);
    
    digitalWrite(SWING_ENA_PIN, HIGH);
    digitalWrite(ROTATE_ENA_PIN, HIGH);

    Wire.begin(); Wire.setClock(400000);
    Wire1.begin(); Wire1.setClock(400000);
    
    swing.setMaxSpeed(4000 * MICROSTEPS); 
    rotate.setMaxSpeed(4000 * MICROSTEPS);
    gripper.attach(GRIPPER_SERVO_PIN);
    gripper.write(80); 
    
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        rclc_node_init_default(&node, NODE_NAME, "", &support);
        
        msg_sub.data.capacity = 128; 
        msg_sub.data.data = (char*) malloc(msg_sub.data.capacity * sizeof(char));
        msg_pub.data.capacity = 128;
        msg_pub.data.data = (char*) malloc(msg_pub.data.capacity * sizeof(char));

        rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_NAME);
        rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "toolhead_status");

        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
        micro_ros_connected = true;
    }
}

void loop() {
    if (!micro_ros_connected) return;

    static uint32_t lastSensorRead = 0;
    if (millis() - lastSensorRead > 20) { 
        float tempS = readRawEncoder(Wire1);
        if (tempS != -999) currentSwingRaw = tempS;
        
        delayMicroseconds(150); 
        
        float tempR = readRawEncoder(Wire);
        if (tempR != -999) currentRotateRaw = tempR;

        f1 = analogRead(FORCE_1_PIN);
        f2 = analogRead(FORCE_2_PIN);
        f3 = analogRead(FORCE_3_PIN);
        f4 = analogRead(FORCE_4_PIN);
        
        lastSensorRead = millis();
    }

    if (targetSwingRaw != -1.0) {
        float sError = targetSwingRaw - currentSwingRaw;
        if (targetSwingRaw == SWING_UP_RAW && currentSwingRaw < 180) sError = (targetSwingRaw - 360) - currentSwingRaw;
        else if (targetSwingRaw == SWING_DOWN_RAW && currentSwingRaw > 180) sError = (targetSwingRaw + 360) - currentSwingRaw;

        if (abs(sError) > TOLERANCE) {
            float sSpeed = (sError * 60 + (sError > 0 ? 200 : -200)) * MICROSTEPS;
            swing.setSpeed(constrain(sSpeed, -4000*MICROSTEPS, 4000*MICROSTEPS)); 
            swing.runSpeed();
        } else {
            swing.setSpeed(0); swing.runSpeed(); 
        }
    }

    if (targetRotateRaw != -1.0) {
        float rError = targetRotateRaw - currentRotateRaw;
        if (rError > 180) rError -= 360;
        if (rError < -180) rError += 360;

        if (abs(rError) > TOLERANCE) {
            float rSpeed = -(rError * 80 + (rError > 0 ? 200 : -200)) * MICROSTEPS;
            rotate.setSpeed(constrain(rSpeed, -4000*MICROSTEPS, 4000*MICROSTEPS));
            rotate.runSpeed();
        } else {
            rotate.setSpeed(0);
            rotate.runSpeed();
        }
    }
    
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    static uint32_t lastPub = 0;
    if (millis() - lastPub > 100) { 
        String status = "S:" + String(currentSwingRaw, 1) + 
                        " R:" + String(currentRotateRaw, 1) + 
                        " F:" + String(f1) + "," + String(f2) + "," + String(f3) + "," + String(f4);
        
        snprintf(msg_pub.data.data, msg_pub.data.capacity, "%s", status.c_str());
        msg_pub.data.size = strlen(msg_pub.data.data);
        rcl_publish(&publisher, &msg_pub, NULL);
        lastPub = millis();
    }
}