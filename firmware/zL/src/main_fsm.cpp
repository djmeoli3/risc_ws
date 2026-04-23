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
AccelStepper zaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
AccelStepper yaxis(AccelStepper::DRIVER, WHEEL_STEP_PIN, WHEEL_DIR_PIN); 
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

volatile float live_z_target = 0.0;
volatile float live_y_target = 0.0;
volatile int live_state = 0;
volatile bool buzzer_req = false;
uint32_t brick_detected_time = 0;
bool waiting_for_lift_delay = false;
uint32_t lift_lost_time = 0;
uint32_t stage_timer = 0;

// safety flags
volatile bool safe_to_descend = false;
volatile bool ramp_closed = false;
volatile bool stage_pause = false;
uint32_t lift_accel = 0;
uint32_t last_ramp_time = 0;
const uint32_t RAMP_INTERVAL = 10;

bool homing_complete = false;

volatile bool    manual_active     = false;
static uint32_t last_jog_cmd_time = 0;
const uint32_t  JOG_TIMEOUT_MS    = 200;
volatile float manual_z_target = 0.0;

// ---------------------------------------------------------------------------
// lift state machine
// ---------------------------------------------------------------------------
enum LiftInternalState { LIFT_HOME, LIFT_IDLE, LIFT_MOVING_UP, LIFT_STAGED, LIFT_MOVING_DOWN };
LiftInternalState carriage_state = LIFT_IDLE;
bool ramp_cleared = false;

void driveLift(int speed) { 
    bool atLower = (digitalRead(LIFT_LIMIT_LOWER) == HIGH); 
    bool atUpper = (digitalRead(LIFT_LIMIT_UPPER) == HIGH);

    if (speed > 0 && atUpper) speed = 0;
    if (speed < 0 && atLower) speed = 0;

    if (speed == 0) {
        analogWrite(LIFT_L_PWM, 0); 
        analogWrite(LIFT_R_PWM, 0);
    } else if (speed > 0) {
        analogWrite(LIFT_R_PWM, 0);
        delayMicroseconds(H_BRIDGE_DEAD_TIME * 10);
        analogWrite(LIFT_L_PWM, speed);
    } else {
        analogWrite(LIFT_L_PWM, 0);
        delayMicroseconds(H_BRIDGE_DEAD_TIME * 10);
        analogWrite(LIFT_R_PWM, abs(speed));
    }
}

float getDistance() { 
    digitalWrite(LIFT_TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(LIFT_TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(LIFT_TRIG_PIN, LOW);
    long duration = pulseIn(LIFT_ECHO_PIN, HIGH, 30000); 
    if (duration == 0) return 999.0;
    return (duration * 0.034 / 2);
}

void motorTick() { zaxis.run(); yaxis.run(); }

// ---------------------------------------------------------------------------
// ros callback
// ---------------------------------------------------------------------------
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size < 17) return;

    int   new_state = (int)msg->data.data[CMD_STATE_REQUEST];
    float z_cmd     = msg->data.data[CMD_ZL_TARGET];

    // manual jog -- falls false on release
    manual_active = (new_state == 0 && z_cmd != 0.0f);
    if (manual_active) last_jog_cmd_time = millis();

    if (new_state != live_state && new_state == 1)
        homing_complete = false;

    live_state    = new_state;
    live_z_target = z_cmd;
    live_y_target = msg->data.data[CMD_WHEEL_FE_VEL];
    buzzer_req    = (msg->data.data[CMD_TRIGGER_HOMING] > 0.5f);

    // handshake flags
    safe_to_descend = (msg->data.data[CMD_CONV_CHECK]      > 0.5f);
    ramp_closed     = (msg->data.data[CMD_LIFT_RAMP_READY] > 0.5f);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LIFT_L_PWM, OUTPUT); pinMode(LIFT_R_PWM, OUTPUT);
    pinMode(LIFT_L_ENABLE, OUTPUT); pinMode(LIFT_R_ENABLE, OUTPUT);
    pinMode(LIFT_LIMIT_LOWER, INPUT_PULLUP);
    pinMode(LIFT_LIMIT_UPPER, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP); 
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);   
    pinMode(LIFT_TRIG_PIN, OUTPUT); pinMode(LIFT_ECHO_PIN, INPUT);
    pinMode(LEAD_ENA_PIN, OUTPUT); pinMode(WHEEL_ENA_PIN, OUTPUT);

    digitalWrite(LEAD_ENA_PIN, LOW); digitalWrite(WHEEL_ENA_PIN, LOW);
    digitalWrite(LIFT_R_ENABLE, HIGH); 
    digitalWrite(LIFT_L_ENABLE, HIGH); 
    
    zaxis.setMaxSpeed(MOVE_SPEED_SCALED); zaxis.setAcceleration(LEAD_ACCEL);
    yaxis.setMaxSpeed(WHEEL_MAX_SPEED); yaxis.setAcceleration(WHEEL_ACCEL);
    yaxis.setPinsInverted(true, false, false); 

    motorTimer.begin(motorTick, MOTOR_TICK_INTERVAL); 

    status_msg.data.data = s_buf;
    status_msg.data.size = 12;
    status_msg.data.capacity = 12;
    cmd_msg.data.data = c_buf;
    cmd_msg.data.size = 0;
    cmd_msg.data.capacity = 17;

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
    bool atLower = (digitalRead(LIFT_LIMIT_LOWER) == HIGH);
    bool atUpper = (digitalRead(LIFT_LIMIT_UPPER) == HIGH);
    
    // lift state machine
    switch(carriage_state) {
        case LIFT_HOME:
            driveLift(-LIFT_SPEED_PWM);
            if (atLower) {
                driveLift(0);
                carriage_state = LIFT_IDLE;
            }
            break;

        case LIFT_IDLE:
            driveLift(0);
            if (atLower && homing_complete && live_state != 0) {
                float dist = getDistance();
                bool seeingBrick = (dist > 0.5 && dist < 6.0);

                // initial trigger
                if (seeingBrick && !waiting_for_lift_delay) {
                    brick_detected_time = millis();
                    waiting_for_lift_delay = true;
                    digitalWrite(LED_PIN, HIGH);
                    lift_lost_time = 0;
                }

                if (waiting_for_lift_delay) {
                    uint32_t timeOn = millis() - brick_detected_time;

                    // 2s turn-on verify
                    if (timeOn >= 2000) {
                        if (seeingBrick && safe_to_descend) {
                            carriage_state = LIFT_MOVING_UP;
                            ramp_cleared = false;
                            waiting_for_lift_delay = false;
                            lift_lost_time = 0;
                            digitalWrite(LED_PIN, LOW);
                        }
                    }

                    // 0.5s grace period
                    if (!seeingBrick) {

                        if (lift_lost_time == 0) {
                            lift_lost_time = millis();
                        }
                        

                        if (millis() - lift_lost_time >= 500) {
                            waiting_for_lift_delay = false;
                            lift_lost_time = 0;
                            digitalWrite(LED_PIN, LOW);
                        }
                    } else {
                        lift_lost_time = 0;
                    }
                    
                    // hard timeout
                    if (timeOn > 4000 && !seeingBrick) {
                        waiting_for_lift_delay = false;
                        digitalWrite(LED_PIN, LOW);
                    }
                }
            }
            break;

        case LIFT_MOVING_UP:
            // ramp up
            if (millis() - last_ramp_time >= RAMP_INTERVAL) {
                if (lift_accel < LIFT_SPEED_PWM) lift_accel++;
                last_ramp_time = millis();
            }

            driveLift(lift_accel);

            if (atUpper) { 
                driveLift(0); 
                lift_accel = 0;
                carriage_state = LIFT_STAGED; 
            } 
            else {
                if (!ramp_closed) { ramp_cleared = true; } 
                if (ramp_cleared && ramp_closed) {
                    driveLift(0);
                    lift_accel = 0;
                    carriage_state = LIFT_STAGED;
                }
            }
            break;

        case LIFT_STAGED:
            driveLift(0);
            lift_accel = 0;
            if (!stage_pause) {
                stage_timer = millis();
                stage_pause = true;
            }
            else if (millis() - stage_timer >= 1000 && safe_to_descend && stage_pause) {
                stage_pause = false;
                carriage_state = LIFT_MOVING_DOWN;
            }
            break;

        case LIFT_MOVING_DOWN:
            // ramp down
            if (millis() - last_ramp_time >= RAMP_INTERVAL) {
                if (lift_accel < LIFT_SPEED_PWM) lift_accel++;
                last_ramp_time = millis();
            }

            if (atLower) { 
                driveLift(0); 
                lift_accel = 0;
                carriage_state = LIFT_IDLE; 
            } else { 
                driveLift(-lift_accel); 
            }
            break;
        }

    // ---------------------------------------------------------------------------
    // stepper fsm
    // ---------------------------------------------------------------------------
    switch (live_state) {
        case 0: { // idle -- manual jog
            zaxis.setMaxSpeed(MOVE_SPEED_SCALED / 2.0f);


            bool jog_active = manual_active ||
                              (millis() - last_jog_cmd_time < JOG_TIMEOUT_MS);

            if (jog_active) {
                zaxis.moveTo((long)(live_z_target * STEPS_PER_MM));
            } else {
                zaxis.setCurrentPosition(zaxis.currentPosition());
            }

            yaxis.setCurrentPosition(yaxis.currentPosition());
            // force lift idle
            carriage_state = LIFT_IDLE;
            driveLift(0);
            waiting_for_lift_delay = false;
            break;
            }


        case 1: // homing
            zaxis.setMaxSpeed(MOVE_SPEED_SCALED);

            if (buzzer_req) { // buzzer phase
                zaxis.setCurrentPosition(zaxis.currentPosition());
            } else { // homing phase
                carriage_state = LIFT_HOME;
                if (!limitClose && !homing_complete) {
                    zaxis.setMaxSpeed(HOMING_SPEED_SCALED);
                    zaxis.moveTo(-1000000);
                } else {
                    zaxis.setCurrentPosition(zaxis.currentPosition());
                    if (limitClose && !homing_complete) {
                        motorTimer.end();
                        zaxis.setCurrentPosition(0);
                        yaxis.setCurrentPosition(0);
                        motorTimer.begin(motorTick, MOTOR_TICK_INTERVAL);
                        homing_complete = true;
                    }
                }
            }
            break;
        case 98: // paused
            zaxis.setCurrentPosition(zaxis.currentPosition());
            yaxis.setCurrentPosition(yaxis.currentPosition());
            driveLift(0);
            break;

        case 99: // safety stop
            zaxis.setCurrentPosition(zaxis.currentPosition());
            driveLift(0);
            break;
        default:
            {
                long targetStepsZ = (long)(live_z_target * STEPS_PER_MM);
                long targetStepsY = (long)(live_y_target * STEPS_PER_MM_Y);
                zaxis.moveTo(targetStepsZ);
                yaxis.moveTo(targetStepsY);
            }
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
        status_msg.data.data[STAT_HW_ID]         = 3.0f;
        status_msg.data.data[STAT_POS_ALPHA]      = (float)zaxis.currentPosition() / STEPS_PER_MM;
        status_msg.data.data[STAT_POS_BETA]       = (float)yaxis.currentPosition() / STEPS_PER_MM_Y;
        status_msg.data.data[STAT_GRIPPER_DETECT] = 0.0f;
        status_msg.data.data[STAT_PROX_SENSOR]    = (float)getDistance();
        status_msg.data.data[STAT_TASK_COMPLETE]  = (zaxis.distanceToGo() == 0 &&
                                                     yaxis.distanceToGo()  == 0) ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MIN_HIT]  = limitClose ? 1.0f : 0.0f;
        status_msg.data.data[STAT_LIMIT_MAX_HIT]  = limitFar   ? 1.0f : 0.0f;
        status_msg.data.data[STAT_ESTOP_HIT]      = 0.0f;
        status_msg.data.data[STAT_RAMP_CONTACT]   = (float)atLower;
        status_msg.data.data[STAT_BRICK_PRESENT]  = 0.0f;
        status_msg.data.data[STAT_IR_SIGNAL]      = 0.0f;
        (void)rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}