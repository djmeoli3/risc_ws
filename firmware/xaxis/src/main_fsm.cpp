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
AccelStepper xaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
AccelStepper conveyor(AccelStepper::DRIVER, CONVEYOR_STEP_PIN, CONVEYOR_DIR_PIN);
AccelStepper pump(AccelStepper::DRIVER, PUMP_STEP_PIN, PUMP_DIR_PIN);
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
volatile bool live_conveyor = false;
volatile bool live_pump = false;
bool homing_complete = false;

// High-speed motor pulsing
void motorTick() {
    xaxis.run(); 
    conveyor.runSpeed();
    pump.runSpeed();
}

float getProximityData() {
    digitalWrite(PROX_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(PROX_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(PROX_TRIG_PIN, LOW);
    long duration = pulseIn(PROX_ECHO_PIN, HIGH, 10000); 
    if (duration == 0) return 0.0f;
    float distance = (duration * 0.0343) / 2.0;
    return (distance >= 2.0f && distance <= 20.0f) ? 1.0f : 0.0f;
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 13) {
        int new_state = (int)msg->data.data[CMD_STATE_REQUEST];
        if (new_state != live_state) { homing_complete = false; }
        live_state = new_state;
        live_target = msg->data.data[CMD_X_LEAD_TARGET];
        live_conveyor = (msg->data.data[CMD_CONVEYOR_ON] > 0.5f);
        live_pump = (msg->data.data[CMD_EXTRACTOR_ON] > 0.5f);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(2000);

    pinMode(LEAD_ENA_PIN, OUTPUT);
    pinMode(CONVEYOR_ENA_PIN, OUTPUT);
    pinMode(PUMP_ENA_PIN, OUTPUT);
    digitalWrite(LEAD_ENA_PIN, LOW);
    digitalWrite(CONVEYOR_ENA_PIN, LOW);
    digitalWrite(PUMP_ENA_PIN, LOW);

    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);
    pinMode(PROX_TRIG_PIN, OUTPUT);
    pinMode(PROX_ECHO_PIN, INPUT);

    xaxis.setMaxSpeed(MOVE_SPEED_SCALED);
    xaxis.setAcceleration(1500 * MICROSTEPS);
    conveyor.setMaxSpeed(CONVEYOR_SPEED_STEPS);
    pump.setMaxSpeed(PUMP_SPEED_STEPS);

    // Start Smooth Pulse Timer
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
    rclc_node_init_default(&node, "xaxis_node", "", &support);

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, "xaxis_command");
    rclc_publisher_init_default(&publisher, &node, ts, "xaxis_status");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA);

    digitalWrite(LED_PIN, LOW);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    bool closeLimit = (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH);
    bool farLimit = (digitalRead(LIMIT_SWITCH_FAR) == HIGH);

    if (live_state == 1) { //homing
        if (!closeLimit && !homing_complete) {
            //use moveTo instead of setSpeed to get smooth acceleration
            xaxis.setMaxSpeed(HOMING_SPEED_SCALED); 
            xaxis.moveTo(1000000); //drive toward the switch with a ramp
        } else {
            xaxis.stop();
            if (closeLimit && !homing_complete) {
                xaxis.setCurrentPosition(0);
                homing_complete = true;
            }
        }
    } 
    else if (live_state >= 2) { //navigation
        long targetSteps = (long)(live_target * STEPS_PER_MM);
        if (xaxis.targetPosition() != -targetSteps) xaxis.moveTo(-targetSteps);

        if (xaxis.distanceToGo() != 0) {
            if ((xaxis.distanceToGo() > 0 && !farLimit) || (xaxis.distanceToGo() < 0 && !closeLimit)) {
                //xaxis.run() is handled by the IntervalTimer for smoothness
            } else {
                xaxis.stop();
            }
        }
    } else {
        xaxis.stop();
    }

    conveyor.setSpeed(live_conveyor ? CONVEYOR_SPEED_STEPS : 0);
    pump.setSpeed(live_pump ? PUMP_SPEED_STEPS : 0);

    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = 2.0f;
        status_msg.data.data[STAT_POS_ALPHA]     = (float)xaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[GRIPPER_DETECT]     = 0.0f; 
        status_msg.data.data[STAT_PROX_SENSOR]   = getProximityData();
        status_msg.data.data[STAT_TASK_COMPLETE] = (xaxis.distanceToGo() == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT] = (float)digitalRead(LIMIT_SWITCH_CLOSE);
        status_msg.data.data[STAT_LIMIT_MAX_HIT] = (float)digitalRead(LIMIT_SWITCH_FAR);
        rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}