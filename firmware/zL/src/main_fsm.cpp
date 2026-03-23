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
AccelStepper yaxis(AccelStepper::DRIVER, WHEEL_STEP_PIN, WHEEL_DIR_PIN); 
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

volatile float live_z_target = 0.0;
volatile float live_y_target = 0.0;
volatile int live_state = 0;
bool homing_complete = false;

//800 steps / 212.79mm = 3.7594 steps/mm for wheels
const float STEPS_PER_MM_Y = 3.7594; 

// High-speed interrupt for smooth stepping
void motorTick() {
    zaxis.run(); 
    yaxis.run(); 
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 13) {
        int new_state = (int)msg->data.data[CMD_STATE_REQUEST];
        if (new_state != live_state && (new_state <= 1)) { 
            homing_complete = false; 
        }
        live_state = new_state;
        
        // ZL Specific Indexing
        live_z_target = msg->data.data[CMD_ZL_TARGET];     //index 4
        live_y_target = msg->data.data[CMD_WHEEL_FL_VEL];  //index 6
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LEAD_ENA_PIN, OUTPUT);
    pinMode(WHEEL_ENA_PIN, OUTPUT);
    digitalWrite(LEAD_ENA_PIN, LOW);
    digitalWrite(WHEEL_ENA_PIN, LOW);

    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);

    //z-axis config
    zaxis.setMaxSpeed(MOVE_SPEED_SCALED);
    zaxis.setAcceleration(1500.0 * (float)MICROSTEPS);

    //y-axis config
    yaxis.setPinsInverted(true, false, false); //the zL motor mirrors the zE motor, need to invert direction
    yaxis.setMaxSpeed(20.0 * (float)MICROSTEPS);
    yaxis.setAcceleration(10.0 * (float)MICROSTEPS);

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
    rclc_node_init_default(&node, "zl_node", "", &support);

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, "z_command");
    rclc_publisher_init_default(&publisher, &node, ts, "zl_status");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    bool limitClose = (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH);
    bool limitFar = (digitalRead(LIMIT_SWITCH_FAR) == HIGH);

    if (live_state == 1) { //homing
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
        yaxis.setCurrentPosition(0);//set zero immediately
        yaxis.stop();
    } 
    else if (live_state >= 2) { //navigation and placement
        long targetStepsZ = (long)(live_z_target * STEPS_PER_MM);
        long targetStepsY = (long)(live_y_target * STEPS_PER_MM_Y);
        
        if (zaxis.targetPosition() != targetStepsZ) zaxis.moveTo(targetStepsZ);
        if (yaxis.targetPosition() != targetStepsY) yaxis.moveTo(targetStepsY);

        if ((zaxis.distanceToGo() > 0 && limitFar) || (zaxis.distanceToGo() < 0 && limitClose)) {
            zaxis.stop(); 
        }
    } else {
        zaxis.stop();
        yaxis.stop();
    }

    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = 3.0f; // ZL ID
        status_msg.data.data[STAT_POS_ALPHA]     = (float)zaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[STAT_POS_BETA]      = (float)yaxis.currentPosition() / STEPS_PER_MM_Y;
        status_msg.data.data[STAT_TASK_COMPLETE] = (zaxis.distanceToGo() == 0 && yaxis.distanceToGo() == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT] = (float)limitClose;
        status_msg.data.data[STAT_LIMIT_MAX_HIT] = (float)limitFar;
        rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}