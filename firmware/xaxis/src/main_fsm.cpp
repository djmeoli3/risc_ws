#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <AccelStepper.h>
#include "config.h"
#include "risc_bus.h"

// ---------------------------------------------------------------------------
// objects
// ---------------------------------------------------------------------------
AccelStepper xaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
AccelStepper conveyor(AccelStepper::DRIVER, CONVEYOR_STEP_PIN, CONVEYOR_DIR_PIN);
AccelStepper pump(AccelStepper::DRIVER, PUMP_STEP_PIN, PUMP_DIR_PIN);
IntervalTimer motorTimer;

// ros objects
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray cmd_msg;
std_msgs__msg__Float32MultiArray status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static float s_buf[12];
static float c_buf[17];

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------
volatile float live_x_target   = 0.0;
volatile int   live_state       = 0;
volatile bool  buzzer_req       = false;
volatile bool  live_adhesive_on = false;

bool     homing_complete        = false;
bool     conveyor_latch         = false;
bool     brickPresent           = false;

uint32_t x_brick_detected_time  = 0;
uint32_t brick_lost_time        = 0;
bool     x_waiting_for_delay    = false;

static uint32_t state13_entry   = 0;
static uint32_t pump_delay      = 0;
static uint32_t manual_retract_start  = 0;
static bool     manual_retract_active = false;
static bool     pump_latch            = false;  // toggle latch

static bool     pump_running_fwd  = false;
volatile int    pump_debug_state    = 0;

// manual jog
volatile bool  manual_active    = false;

// ---------------------------------------------------------------------------
// ultrasonic distance
// ---------------------------------------------------------------------------
float getXProxDistance() {
    digitalWrite(PROX_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(PROX_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(PROX_TRIG_PIN, LOW);
    long duration = pulseIn(PROX_ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 999.0;
    return (duration * 0.034f / 2.0f);
}

// ---------------------------------------------------------------------------
// motor isr
// ---------------------------------------------------------------------------
void motorTick() {
    xaxis.run();
    conveyor.run();
    pump.run();
}

// ---------------------------------------------------------------------------
// ros callback
// ---------------------------------------------------------------------------
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < 15) return;

    int   new_state = (int)msg->data.data[CMD_STATE_REQUEST];
    float x_cmd     = msg->data.data[CMD_X_LEAD_TARGET];

    // manual jog -- falls false on release
    manual_active = (new_state == 0 && x_cmd != 0.0f);

    // state transition housekeeping
    if (new_state != live_state) {
        if (new_state == 1) homing_complete = false;
        if (new_state != 2) conveyor_latch  = false;
    }

    live_state    = new_state;
    live_x_target = x_cmd;
    buzzer_req    = (msg->data.data[CMD_TRIGGER_HOMING] > 0.5f);

    // pump latch -- edge detect only
    bool adhesive_cmd = (msg->data.data[CMD_ADHESIVE_ON] > 0.5f);
    if (adhesive_cmd != live_adhesive_on) {
        live_adhesive_on = adhesive_cmd;
        if (!adhesive_cmd) {
    
            manual_retract_active = true;
            manual_retract_start  = 0;
        }
    }
    if (adhesive_cmd) pump_latch = true;
}

// ---------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN,           OUTPUT);
    pinMode(LEAD_ENA_PIN,      OUTPUT);
    pinMode(CONVEYOR_ENA_PIN,  OUTPUT);
    pinMode(PUMP_ENA_PIN,      OUTPUT);
    pinMode(IR_PIN,            INPUT);
    digitalWrite(LEAD_ENA_PIN,     LOW);
    digitalWrite(CONVEYOR_ENA_PIN, LOW);
    digitalWrite(PUMP_ENA_PIN,     LOW);

    pinMode(LIMIT_SWITCH_RAMP,  INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR,   INPUT_PULLUP);
    pinMode(PROX_TRIG_PIN,      OUTPUT);
    pinMode(PROX_ECHO_PIN,      INPUT);

    // full speed; case 0 halves for jog
    xaxis.setMaxSpeed(MOVE_SPEED_SCALED);
    xaxis.setAcceleration(LEAD_ACCEL);
    xaxis.setPinsInverted(true, false, false);

    conveyor.setMaxSpeed(CONVEYOR_SPEED_STEPS);
    conveyor.setAcceleration(LEAD_ACCEL);

    pump.setMaxSpeed(PUMP_SPEED_STEPS);
    pump.setAcceleration(999999);
    pump.setPinsInverted(false, false, false);

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
    rclc_node_init_default(&node, "xax_node", "", &support);

    const rosidl_message_type_support_t * ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, "xaxis_command");
    rclc_publisher_init_default(&publisher,     &node, ts, "xaxis_status");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg,
                                   &subscription_callback, ON_NEW_DATA);
}

// ---------------------------------------------------------------------------
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // read hardware
    bool limitClose  = (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH);
    bool limitFar    = (digitalRead(LIMIT_SWITCH_FAR)   == HIGH);
    bool rampContact = (digitalRead(LIMIT_SWITCH_RAMP)  == HIGH);
    bool irHigh      = (digitalRead(IR_PIN) == HIGH);

    // mirror IR on LED
    digitalWrite(LED_PIN, irHigh ? HIGH : LOW);

    // brick presence debounce
    float current_dist = getXProxDistance();
    bool  brickDetect  = (current_dist > 0.5f && current_dist < 6.0f);

    if (brickDetect && !x_waiting_for_delay) {
        x_brick_detected_time = millis();
        x_waiting_for_delay   = true;
    }

    if (x_waiting_for_delay) {
        uint32_t timeOn = millis() - x_brick_detected_time;

        if (timeOn >= 500 && brickDetect) {
            brickPresent  = true;
            brick_lost_time = 0;
        }

        if (!brickDetect) {
            if (brick_lost_time == 0) brick_lost_time = millis();
            if (millis() - brick_lost_time >= 500) {
                brickPresent        = false;
                x_waiting_for_delay = false;
                brick_lost_time     = 0;
            }
        } else {
            brick_lost_time = 0;
        }

        if (timeOn > 10000 && !brickDetect) x_waiting_for_delay = false;

    } else {
        brickPresent    = false;
        brick_lost_time = 0;
    }


    long targetStepsX = (long)(live_x_target * STEPS_PER_MM);


    switch (live_state) {

        case 0: // idle -- manual jog
            xaxis.setMaxSpeed(MOVE_SPEED_SCALED / 2.0f);

            if (manual_active) {
                xaxis.moveTo(targetStepsX);
            } else {
                xaxis.setCurrentPosition(xaxis.currentPosition());
            }

            conveyor.stop();

            // manual pump toggle
            if (pump_latch && live_adhesive_on) {
                pump_debug_state = 1;
                if (!pump_running_fwd) {
                    pump.setCurrentPosition(0);
                    pump.setMaxSpeed(PUMP_SPEED_STEPS);
                    pump.moveTo(1000000);
                    pump_running_fwd = true;
                }
            } else if (manual_retract_active) {
                pump_running_fwd = false;
                pump_debug_state = 2;
                if (manual_retract_start == 0) {
                    manual_retract_start = millis();
                    pump.setCurrentPosition(0);
                    pump.setMaxSpeed(PUMP_SPEED_STEPS * 0.75f);
                    pump.moveTo(-1000000);
                }
                if (millis() - manual_retract_start >= 1000) {
                    pump.stop();
                    pump_latch            = false;
                    manual_retract_active = false;
                    manual_retract_start  = 0;
                }
            } else {
                pump_running_fwd = false;
                pump_debug_state = 0;
                pump.stop();
            }
            break;

        case 1: // homing
            xaxis.setMaxSpeed(MOVE_SPEED_SCALED);

            if (buzzer_req) {
                xaxis.setCurrentPosition(xaxis.currentPosition());
            } else {
                if (!limitClose && !homing_complete) {
                    xaxis.setMaxSpeed(HOMING_SPEED_SCALED);
                    xaxis.moveTo(-1000000);
                } else {
                    xaxis.setCurrentPosition(xaxis.currentPosition());
                    if (limitClose && !homing_complete) {
                        xaxis.setCurrentPosition(0);
                        homing_complete = true;
                    }
                }
            }
            conveyor.stop();
            break;

        case 2: // NAV_AND_FEED
            xaxis.moveTo(targetStepsX);
            if (brickPresent) conveyor_latch = true;

            if (conveyor_latch) {
                conveyor.setMaxSpeed(CONVEYOR_SPEED_STEPS);
                conveyor.moveTo(conveyor.currentPosition() + 5000);
            } else {
                conveyor.stop();
            }
            break;

        case 3: // GRIP_ENGAGE
            xaxis.moveTo(targetStepsX);
            conveyor.stop();
            conveyor_latch = false;
            break;

        case 12: // PUMP_DELIVERY
            xaxis.moveTo(targetStepsX);
            conveyor.stop();
            state13_entry = 0;
            if (live_adhesive_on) {
                if (pump_delay == 0) pump_delay = millis();
                if (millis() - pump_delay > 1500) {
                    pump.setMaxSpeed(PUMP_SPEED_STEPS*0.75f);
                    pump.moveTo(pump.currentPosition() < 0 ? 1000000 : pump.currentPosition() + 1000000);
                }
            } else {
                pump.stop();
                pump_delay = 0;
            }
            break;

        case 13: // swing_to_final -- pump retract
            xaxis.moveTo(targetStepsX);
            pump_delay = 0;
            conveyor.stop();
            if (state13_entry == 0) {
                state13_entry = millis();
                pump.setMaxSpeed(PUMP_SPEED_STEPS * 0.75f);
                pump.moveTo(pump.currentPosition() - 1000000);
            }
            if (millis() - state13_entry >= 1000) pump.stop();
            break;

        case 98: // paused
            xaxis.setCurrentPosition(xaxis.currentPosition());
            conveyor.stop();
            pump.stop();
            break;

        case 99: // safety stop
            xaxis.setCurrentPosition(xaxis.currentPosition());
            conveyor.stop();
            pump.stop();
            conveyor_latch = false;
            break;

        default:
            xaxis.moveTo(targetStepsX);
            conveyor.stop();
            if (millis() - state13_entry > 500) pump.stop();
            conveyor_latch = false;
            break;
    }

    // limit protection
    if ((xaxis.distanceToGo() > 0 && limitFar) ||
        (xaxis.distanceToGo() < 0 && limitClose)) {
        xaxis.setCurrentPosition(xaxis.currentPosition());
    }

    // status publish @ 20hz
    static uint32_t lastP = 0;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID]         = 2.0f;
        status_msg.data.data[STAT_POS_ALPHA]      = (float)xaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[STAT_POS_BETA]       = (float)pump_debug_state;
        status_msg.data.data[STAT_GRIPPER_DETECT] = 0.0f;
        status_msg.data.data[STAT_PROX_SENSOR]    = conveyor_latch ? 1.0f : 0.0f;
        status_msg.data.data[STAT_TASK_COMPLETE]  = (xaxis.distanceToGo() == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT]  = limitClose  ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MAX_HIT]  = limitFar    ? 1.0f : 0.0f;
        status_msg.data.data[STAT_ESTOP_HIT]      = 0.0f;
        status_msg.data.data[STAT_RAMP_CONTACT]   = rampContact ? 1.0f : 0.0f;
        status_msg.data.data[STAT_BRICK_PRESENT]  = brickPresent ? 1.0f : 0.0f;
        status_msg.data.data[STAT_IR_SIGNAL]      = !irHigh ? 1.0f : 0.0f;
        (void)rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}