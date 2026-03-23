#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include "config.h"
#include "risc_bus.h"

// Objects
AccelStepper swing(AccelStepper::DRIVER, SWING_STEP_PIN, SWING_DIR_PIN);
AccelStepper rotate(AccelStepper::DRIVER, ROTATE_STEP_PIN, ROTATE_DIR_PIN);
Servo gripper;
IntervalTimer motorTimer; 

// ROS 2 Objects
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray cmd_msg;
std_msgs__msg__Float32MultiArray status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static float s_buf[11];
static float c_buf[15];

volatile float live_swing_target = SWING_UP_RAW; 
volatile float live_rot_target = 0.0;
volatile bool live_grip_open = false;
volatile bool live_extractor_on = false;
float currentSwingRaw = 0, currentRotateRaw = 0;
float sErr = 0, rErr = 0;

// High-speed interrupt for smooth pulses
void motorTick() {
    swing.runSpeed();
    rotate.runSpeed();
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 12) {
        live_swing_target = msg->data.data[CMD_TOOL_SWING_TARGET];
        live_rot_target   = msg->data.data[CMD_TOOL_ROT_TARGET];
        live_grip_open    = (msg->data.data[CMD_GRIP_OPEN] > 0.5f);
        live_extractor_on = (msg->data.data[CMD_EXTRACTOR_ON] > 0.5f);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    }
}

float readRawEncoder(TwoWire &bus) {
    bus.beginTransmission(AS5600_ADDR);
    bus.write(0x0E); 
    if (bus.endTransmission() != 0) return -999;
    bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (bus.available() >= 2) {
        uint16_t raw = (bus.read() << 8) | bus.read();
        return (raw * 360.0) / 4096.0;
    }
    return -999;
}

void setup() {
    delay(2000); 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(SWING_ENA_PIN, OUTPUT); 
    pinMode(ROTATE_ENA_PIN, OUTPUT);
    //toolhead NEMA 17s are high enable
    digitalWrite(SWING_ENA_PIN, HIGH); 
    digitalWrite(ROTATE_ENA_PIN, HIGH);
    
    pinMode(LIMIT_SWITCH_GRIPPER, INPUT_PULLUP);

    //aboslute encoder wires
    Wire.begin(); Wire1.begin();
    
    swing.setMaxSpeed(500 * MICROSTEPS); 
    rotate.setMaxSpeed(500 * MICROSTEPS);
    gripper.attach(GRIPPER_SERVO_PIN);

    motorTimer.begin(motorTick, 100); 

    status_msg.data.data = s_buf;
    status_msg.data.size = 11;
    status_msg.data.capacity = 11;
    cmd_msg.data.data = c_buf;
    cmd_msg.data.size = 0; 
    cmd_msg.data.capacity = 15;

    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "toolhead_node", "", &support);

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, "risc_command");
    rclc_publisher_init_default(&publisher, &node, ts, "toolhead_status");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA);

    digitalWrite(LED_PIN, LOW); 
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    float ts = readRawEncoder(Wire1); if (ts != -999) currentSwingRaw = ts;
    float tr = readRawEncoder(Wire); if (tr != -999) currentRotateRaw = tr;
    sErr = live_swing_target - currentSwingRaw;

    if (live_swing_target > 180 && currentSwingRaw < 100) sErr = (live_swing_target - 360) - currentSwingRaw;
    else if (live_swing_target < 100 && currentSwingRaw > 200) sErr = (live_swing_target + 360) - currentSwingRaw;
    
    if (abs(sErr) > TOLERANCE) {
        float sSpeed = (sErr * 15 + (sErr > 0 ? 50 : -50)) * MICROSTEPS;
        swing.setSpeed(constrain(sSpeed, -500*MICROSTEPS, 500*MICROSTEPS));
    } else { swing.setSpeed(0); }

    float normalizedR = currentRotateRaw - ROTATE_HOME_RAW;
    if (normalizedR < 0) normalizedR += 360;
    rErr = live_rot_target - normalizedR;
    if (rErr > 180) rErr -= 360; 
    if (rErr < -180) rErr += 360;

    if (abs(rErr) > TOLERANCE) {
        float rSpeed = -(rErr * 17 + (rErr > 0 ? 50 : -50)) * MICROSTEPS;
        rotate.setSpeed(constrain(rSpeed, -500*MICROSTEPS, 500*MICROSTEPS));
    } else { rotate.setSpeed(0); }

    gripper.write(live_grip_open ? 179 : 80);
    analogWrite(EXTRACTOR_PIN, live_extractor_on ? 200 : 0);

    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = 1.0f;
        status_msg.data.data[STAT_POS_ALPHA]     = currentSwingRaw; 
        status_msg.data.data[STAT_POS_BETA]      = currentRotateRaw;
        status_msg.data.data[GRIPPER_DETECT]     = (digitalRead(LIMIT_SWITCH_GRIPPER) == HIGH) ? 1.0f : 0.0f; 
        status_msg.data.data[STAT_PROX_SENSOR]   = 0.0f;
        bool atGoal = (abs(sErr) <= TOLERANCE && abs(rErr) <= TOLERANCE);
        status_msg.data.data[STAT_TASK_COMPLETE] = atGoal ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT] = 0.0f;
        status_msg.data.data[STAT_LIMIT_MAX_HIT] = 0.0f;
        
        //explicitly clear trailing indices to prevent extra garbage/noise
        status_msg.data.data[8] = 0.0f;
        status_msg.data.data[9] = 0.0f;
        status_msg.data.data[10] = 0.0f;

        rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}