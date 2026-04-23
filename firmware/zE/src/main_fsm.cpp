#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <AccelStepper.h>
#include "config.h"
#include "risc_bus.h"

// ---------------------------------------------------------------------------
// objects
// ---------------------------------------------------------------------------
AccelStepper zaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
AccelStepper yaxis(AccelStepper::DRIVER, WHEEL_STEP_PIN, WHEEL_DIR_PIN);
IntervalTimer motorTimer;

// ros objects
rcl_subscription_t subscriber;
rcl_subscription_t ir_subscriber;
rcl_publisher_t    publisher;
std_msgs__msg__Float32MultiArray cmd_msg;
std_msgs__msg__Float32MultiArray status_msg;
std_msgs__msg__Bool              ir_msg;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

static float s_buf[12];
static float c_buf[17];

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------
volatile float live_z_target  = 0.0;
volatile float live_y_target  = 0.0;
volatile int   live_state      = 0;
volatile bool  buzzer_req      = false;
volatile bool     manual_active      = false;
static uint32_t  last_jog_cmd_time  = 0;
const uint32_t   JOG_TIMEOUT_MS     = 200;
bool           homing_complete = false;

// ir: true=adhesive ok, false=out
volatile bool  ir_adhesive_ok  = true;

// ---------------------------------------------------------------------------
// motor isr
// ---------------------------------------------------------------------------
void motorTick() {
    zaxis.run();
    yaxis.run();
}

// ---------------------------------------------------------------------------
// light stack
// ---------------------------------------------------------------------------
void updateLightStack(int state) {
    if (!ir_adhesive_ok) {
        digitalWrite(GREEN_STACK,  LOW);
        digitalWrite(YELLOW_STACK, LOW);
        digitalWrite(RED_STACK,    HIGH);
        return;
    }
    switch (state) {
        case 0:  // idle
            digitalWrite(GREEN_STACK,  HIGH);
            digitalWrite(YELLOW_STACK, LOW);
            digitalWrite(RED_STACK,    LOW);
            break;
        case 98: // paused
            digitalWrite(GREEN_STACK,  LOW);
            digitalWrite(YELLOW_STACK, HIGH);
            digitalWrite(RED_STACK,    HIGH);
            break;
        case 99: // safety stop
            digitalWrite(GREEN_STACK,  LOW);
            digitalWrite(YELLOW_STACK, LOW);
            digitalWrite(RED_STACK,    HIGH);
            break;
        default: // active build
            digitalWrite(GREEN_STACK,  LOW);
            digitalWrite(YELLOW_STACK, HIGH);
            digitalWrite(RED_STACK,    LOW);
            break;
    }
}

// ---------------------------------------------------------------------------
// ros callbacks
// ---------------------------------------------------------------------------
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < 15) return;

    int   new_state = (int)msg->data.data[CMD_STATE_REQUEST];
    float z_cmd     = msg->data.data[CMD_ZE_TARGET];

    manual_active = (new_state == 0 && z_cmd != 0.0f);
    if (manual_active) last_jog_cmd_time = millis();

    if (new_state != live_state && new_state == 1)
        homing_complete = false;

    live_state   = new_state;
    live_z_target = z_cmd;
    live_y_target = msg->data.data[CMD_WHEEL_FE_VEL];
    buzzer_req    = (msg->data.data[CMD_TRIGGER_HOMING] > 0.5f);
}


void ir_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    ir_adhesive_ok = msg->data;
}

// ---------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN,       OUTPUT);
    pinMode(LEAD_ENA_PIN,  OUTPUT);
    pinMode(WHEEL_ENA_PIN, OUTPUT);

    pinMode(BUZZER_STACK, OUTPUT);
    pinMode(GREEN_STACK,  OUTPUT);
    pinMode(YELLOW_STACK, OUTPUT);
    pinMode(RED_STACK,    OUTPUT);
    pinMode(ESTOP_PIN,    INPUT_PULLUP);

    digitalWrite(LEAD_ENA_PIN,  LOW);
    digitalWrite(WHEEL_ENA_PIN, LOW);

    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR,   INPUT_PULLUP);

    zaxis.setMaxSpeed(MOVE_SPEED_SCALED);
    zaxis.setAcceleration(LEAD_ACCEL);
    yaxis.setMaxSpeed(WHEEL_MAX_SPEED);
    yaxis.setAcceleration(WHEEL_ACCEL);

    motorTimer.begin(motorTick, MOTOR_TICK_INTERVAL);

    status_msg.data.data     = s_buf;
    status_msg.data.size     = 12;
    status_msg.data.capacity = 12;
    cmd_msg.data.data        = c_buf;
    cmd_msg.data.size        = 0;
    cmd_msg.data.capacity    = 17;

    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "ze_node", "", &support);

    const rosidl_message_type_support_t * ts_float =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    const rosidl_message_type_support_t * ts_bool =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);

    rclc_subscription_init_default(&subscriber,    &node, ts_float, "z_command");
    rclc_subscription_init_default(&ir_subscriber, &node, ts_bool,  "ir_signal");
    rclc_publisher_init_default(&publisher,        &node, ts_float, "ze_status");

    // 2 subscriptions
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber,    &cmd_msg, &subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &ir_subscriber, &ir_msg,  &ir_callback,           ON_NEW_DATA);
}

// ---------------------------------------------------------------------------
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    bool limitClose = (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH);
    bool limitFar   = (digitalRead(LIMIT_SWITCH_FAR)   == HIGH);

    updateLightStack(live_state);
    digitalWrite(BUZZER_STACK, buzzer_req ? HIGH : LOW);

    long targetStepsZ = (long)(live_z_target * STEPS_PER_MM);
    long targetStepsY = (long)(live_y_target * STEPS_PER_MM_Y);

    switch (live_state) {

        case 0: { // idle -- manual jog
            zaxis.setMaxSpeed(MOVE_SPEED_SCALED / 2.0f);
            bool jog_active = manual_active ||
                              (millis() - last_jog_cmd_time < JOG_TIMEOUT_MS);

            if (jog_active) {
                zaxis.moveTo(targetStepsZ);
            } else {
                zaxis.setCurrentPosition(zaxis.currentPosition());
            }

            yaxis.setCurrentPosition(yaxis.currentPosition());
            break;
            }


        case 1: // homing
            zaxis.setMaxSpeed(MOVE_SPEED_SCALED);

            if (buzzer_req) {
                zaxis.setCurrentPosition(zaxis.currentPosition());
            } else {
                if (!limitClose && !homing_complete) {
                    zaxis.setMaxSpeed(HOMING_SPEED_SCALED);
                    zaxis.moveTo(-1000000);
                } else {
                    zaxis.setCurrentPosition(zaxis.currentPosition());
                    if (limitClose && !homing_complete) {
                        zaxis.setCurrentPosition(0);
                        homing_complete = true;
                    }
                }
            }
            yaxis.setCurrentPosition(0);
            yaxis.stop();
            break;

        case 98: // paused
            zaxis.setCurrentPosition(zaxis.currentPosition());
            yaxis.setCurrentPosition(yaxis.currentPosition());
            break;

        case 99: // safety stop
            zaxis.setCurrentPosition(zaxis.currentPosition());
            yaxis.stop();
            break;

        default:
            zaxis.moveTo(targetStepsZ);
            yaxis.moveTo(targetStepsY);
            break;
    }

    // limit protection
    if ((zaxis.distanceToGo() > 0 && limitFar) ||
        (zaxis.distanceToGo() < 0 && limitClose)) {
        zaxis.setCurrentPosition(zaxis.currentPosition());
    }

    // status publish @ 20hz
    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = 4.0f;
        status_msg.data.data[STAT_POS_ALPHA]      = (float)zaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[STAT_POS_BETA]       = (float)yaxis.currentPosition() / STEPS_PER_MM_Y;
        status_msg.data.data[STAT_GRIPPER_DETECT] = 0.0f;
        status_msg.data.data[STAT_PROX_SENSOR]    = 0.0f;
        status_msg.data.data[STAT_TASK_COMPLETE]  = (zaxis.distanceToGo() == 0 &&
                                                     yaxis.distanceToGo()  == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT]  = limitClose ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MAX_HIT]  = limitFar   ? 1.0f : 0.0f;
        status_msg.data.data[STAT_ESTOP_HIT]      = (digitalRead(ESTOP_PIN) == LOW) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_RAMP_CONTACT]   = 0.0f;
        status_msg.data.data[STAT_BRICK_PRESENT]  = 0.0f;
        status_msg.data.data[STAT_IR_SIGNAL]      = 0.0f;
        (void)rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}