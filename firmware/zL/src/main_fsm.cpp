#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <AccelStepper.h>
#include "config.h"
#include "risc_bus.h"

// Objects
AccelStepper zaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
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

volatile float live_target = 0.0;
volatile int live_state = 0;
bool homing_complete = false;

// High-speed interrupt for smooth stepping
void motorTick() {
    zaxis.run(); 
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 13) {
        int new_state = (int)msg->data.data[CMD_STATE_REQUEST];
        if (new_state != live_state && (new_state <= 1)) { 
            homing_complete = false; 
        }
        live_state = new_state;
        
        // Smart Indexing: Selects index 4 for ZL (HW_ID 3.0) or 5 for ZE (HW_ID 4.0)
        if (HW_ID == 3.0) {
            live_target = msg->data.data[CMD_ZL_TARGET];
        } else {
            live_target = msg->data.data[CMD_ZE_TARGET];
        }
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LEAD_ENA_PIN, OUTPUT);
    digitalWrite(LEAD_ENA_PIN, LOW);

    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);

    zaxis.setMaxSpeed(MOVE_SPEED_SCALED);
    zaxis.setAcceleration(2000.0 * (float)MICROSTEPS);

    // 100 microsecond interval for 10kHz pulse checks
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
    rclc_node_init_default(&node, NODE_NAME, "", &support);

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, SUB_TOPIC);
    rclc_publisher_init_default(&publisher, &node, ts, PUB_TOPIC);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    bool limitClose = (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH);
    bool limitFar = (digitalRead(LIMIT_SWITCH_FAR) == HIGH);

    if (live_state == 1) { // HOMING
        if (!limitClose && !homing_complete) {
            zaxis.setMaxSpeed(HOMING_SPEED_SCALED);
            zaxis.moveTo(-1000000); 
        } else {
            zaxis.stop();
            if (limitClose && !homing_complete) {
                zaxis.setCurrentPosition(0);
                homing_complete = true;
            }
        }
    } 
    else if (live_state >= 2) { // NAVIGATION & PLACEMENT
        long targetSteps = (long)(live_target * STEPS_PER_MM);
        
        // 1. Update target position first to create a distanceToGo
        if (zaxis.targetPosition() != targetSteps) {
            zaxis.moveTo(targetSteps);
        }

        // 2. Safety: Only stop if moving toward a hit limit
        if ((zaxis.distanceToGo() > 0 && limitFar) || (zaxis.distanceToGo() < 0 && limitClose)) {
            zaxis.stop(); 
        }
    } else {
        zaxis.stop();
    }

    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = HW_ID; 
        status_msg.data.data[STAT_POS_ALPHA]     = (float)zaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[GRIPPER_DETECT]     = 0.0f;
        status_msg.data.data[STAT_PROX_SENSOR]   = 0.0f;
        status_msg.data.data[STAT_TASK_COMPLETE] = (zaxis.distanceToGo() == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT] = (float)digitalRead(LIMIT_SWITCH_CLOSE);
        status_msg.data.data[STAT_LIMIT_MAX_HIT] = (float)digitalRead(LIMIT_SWITCH_FAR);
        rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}