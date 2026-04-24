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

// ---------------------------------------------------------------------------
// objects
// ---------------------------------------------------------------------------
AccelStepper swing(AccelStepper::DRIVER, SWING_STEP_PIN, SWING_DIR_PIN);
AccelStepper rotate(AccelStepper::DRIVER, ROTATE_STEP_PIN, ROTATE_DIR_PIN);
Servo gripper;
Servo valve_servo;
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
volatile float live_swing_target = 0.0; 
volatile float live_rot_target = 0.0;
volatile int live_state = -1;
volatile bool live_grip_open = false;
volatile bool live_extractor_on = false;
volatile bool buzzer_req = false;

float currentSwingRaw = 0, currentRotateRaw = 0;
float sErr = 0, rErr = 0;
bool rotHomed = false;
bool swingHomed = false;

bool swing_at_target = false;
float last_swing_target = -1.0;

// coordination timers
uint32_t swing_dwell_start = 0;
uint32_t retract_dwell_start = 0;
uint32_t valve_delay_start = 0;

const int FILTER_WINDOW = 5;
float swing_history[FILTER_WINDOW] = {0};
int filter_idx = 0;
float smoothedSwingRaw = 0;

// ---------------------------------------------------------------------------
// motor isr
// ---------------------------------------------------------------------------
void motorTick() {
    swing.runSpeed();
    rotate.runSpeed();
}

// ---------------------------------------------------------------------------
// encoder helper
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// startup homing
// ---------------------------------------------------------------------------
void performStartupHome() {

    swing.setMaxSpeed(800 * MICROSTEPS); 
    rotate.setMaxSpeed(800 * MICROSTEPS);
    
    digitalWrite(SWING_ENA_PIN, HIGH);
    digitalWrite(ROTATE_ENA_PIN, HIGH);
    
    if (!rotHomed || !swingHomed) {
        float tempS = readRawEncoder(Wire1);
        float tempR = readRawEncoder(Wire);
        
        if (tempS != -999) {
            float sError = SWING_UP_RAW - tempS;
            if (SWING_UP_RAW > 180 && tempS < 100) sError = (SWING_UP_RAW - 360) - tempS;
            
            if (abs(sError) > TOLERANCE) {
                float dynamicOffset = 180.0;
                if (abs(sError) < 20.0) dynamicOffset = abs(sError) * 18.0; 
                
                float sSpeed = (sError * 30 + (sError > 0 ? dynamicOffset : -dynamicOffset)) * MICROSTEPS * 2;
                swing.setSpeed(constrain(sSpeed, -7000*MICROSTEPS, 7000*MICROSTEPS));
                swing.runSpeed();
            } else { 
                swing.setSpeed(0); swing.runSpeed();
                swingHomed = true; 
            }
        }

        if (tempR != -999) {
            float hErr = ROTATE_HOME_RAW - tempR;
            if (hErr > 180) hErr -= 360;
            if (hErr < -180) hErr += 360;

            if (abs(hErr) > TOLERANCE) {
                float dynamicOffset = 180.0;
                if (abs(hErr) < 10.0) dynamicOffset = abs(hErr) * 18.0;

                float rSpeed = -(hErr * 55 + (hErr > 0 ? dynamicOffset : -dynamicOffset)) * MICROSTEPS * 2;
                rotate.setSpeed(constrain(rSpeed, -7000*MICROSTEPS, 7000*MICROSTEPS));
                rotate.runSpeed();
            } else {
                rotate.setSpeed(0); rotate.runSpeed();
                rotate.setCurrentPosition(0);
                rotHomed = true;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ros callback
// ---------------------------------------------------------------------------
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size >= 15) {
        live_state        = (int)msg->data.data[CMD_STATE_REQUEST];
        live_rot_target   = msg->data.data[CMD_TOOL_ROT_TARGET];
        live_swing_target = msg->data.data[CMD_TOOL_SWING_TARGET];
        live_grip_open    = (msg->data.data[CMD_GRIP_OPEN] > 0.5f);
        live_extractor_on = (msg->data.data[CMD_EXTRACTOR_ON] > 0.5f);
        buzzer_req        = (msg->data.data[CMD_TRIGGER_HOMING] > 0.5f);
    }
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    pinMode(SWING_ENA_PIN, OUTPUT); pinMode(ROTATE_ENA_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_GRIPPER, INPUT_PULLUP);
    pinMode(EXTRACTOR_PIN, OUTPUT);
    
    Wire.begin(); Wire1.begin();
    swing.setMaxSpeed(800 * MICROSTEPS); rotate.setMaxSpeed(800 * MICROSTEPS);
    gripper.attach(GRIPPER_SERVO_PIN);
    valve_servo.attach(ADHESIVE_VALVE_PIN);
    valve_servo.write(VALVE_CLOSED_POS);
    motorTimer.begin(motorTick, 100); 

    status_msg.data.data = s_buf; status_msg.data.size = 12; status_msg.data.capacity = 12;
    cmd_msg.data.data = c_buf; cmd_msg.data.size = 0; cmd_msg.data.capacity = 17;

    set_microros_transports();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "toolhead_node", "", &support);

    const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray);
    rclc_subscription_init_default(&subscriber, &node, ts, "risc_command");
    rclc_publisher_init_default(&publisher, &node, ts, "toolhead_status");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA);

}

// ---------------------------------------------------------------------------
// main loop
// ---------------------------------------------------------------------------
void loop() {

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    
    // sensor read + filter
    static uint32_t lastSensorRead = 0;
    if (millis() - lastSensorRead > 20) { 
        float tempS = readRawEncoder(Wire1); 
        if (tempS != -999) {
            // snap history on 360 wrap
            float lastVal = swing_history[filter_idx == 0 ? FILTER_WINDOW-1 : filter_idx-1];
            if (abs(tempS - lastVal) > 200.0) {
                for(int i = 0; i < FILTER_WINDOW; i++) swing_history[i] = tempS;
            }

            swing_history[filter_idx] = tempS;
            filter_idx = (filter_idx + 1) % FILTER_WINDOW;

            float sum = 0;
            for(int i = 0; i < FILTER_WINDOW; i++) sum += swing_history[i];
            smoothedSwingRaw = sum / FILTER_WINDOW;
            currentSwingRaw = tempS; 
        }
        
        float tempR = readRawEncoder(Wire); 
        if (tempR != -999) currentRotateRaw = tempR;
        lastSensorRead = millis();
    }
    // dynamic speed limits
    float currentMaxSpeed = 3500.0;
    float speedMultiplier = 1.0;

    if (live_state == 12) {
        currentMaxSpeed = 50.0;
        speedMultiplier = 0.2;
    }

    switch (live_state) { 
        case 0:
            performStartupHome();
            break;
            
        case 1:
            if (buzzer_req) {
                swing.stop();
                rotate.stop();
            } else {
                performStartupHome();
            }
            break;
        case 4: case 12: case 13:
            rotate.setSpeed(0);
            break;
            
        case 7: case 8: 
            gripper.write(145);
            break;

        case 99: case -1: // safety stop
            swing.setSpeed(0);
            rotate.setSpeed(0);
            break;

        default:
            break;
    }

    if (live_state != -1 && live_state != 99 && !buzzer_req) {
        // latch reset
        static int last_processed_state = -1;
        if (abs(live_swing_target - last_swing_target) > 2.0 || live_state != last_processed_state) {
            swing_at_target = false;
            last_swing_target = live_swing_target;
            last_processed_state = live_state;
        }

        float effective_target = live_swing_target;

        // dwell coordination
        if (live_state == 12) {
            if (abs(smoothedSwingRaw - 266.5) < TOLERANCE && swing_dwell_start == 0) swing_dwell_start = millis();

            if (swing_dwell_start > 0 && (millis() - swing_dwell_start < 1000)) effective_target = smoothedSwingRaw;
        } else { swing_dwell_start = 0; }

        if (live_state == 13) {
            if (retract_dwell_start == 0) retract_dwell_start = millis();
            if (millis() - retract_dwell_start < 1000) effective_target = smoothedSwingRaw; 
        } else { retract_dwell_start = 0; }

        // error math
        sErr = effective_target - smoothedSwingRaw;

        // force long-way around hard stop
        if (effective_target > 200.0 && smoothedSwingRaw < 100.0) {

            sErr = (effective_target - 360.0) - smoothedSwingRaw;
        }
        else if (effective_target < 100.0 && smoothedSwingRaw > 200.0) {
            sErr = (effective_target + 360.0) - smoothedSwingRaw;
        }

        if (!swing_at_target) {
            if (abs(sErr) > TOLERANCE) {
                float dynamicOffset = (abs(sErr) < 10.0) ? abs(sErr) * 18.0 : 180.0; 
                float sSpeed = (sErr * 55 + (sErr > 0 ? dynamicOffset : -dynamicOffset)) * MICROSTEPS * 2 * speedMultiplier;
                swing.setSpeed(constrain(sSpeed, -currentMaxSpeed * MICROSTEPS * 2, currentMaxSpeed * MICROSTEPS * 2));
            } else { 
                swing.setSpeed(0);
                // no latch during dwell
                bool dwelling = (live_state == 12 && (millis() - swing_dwell_start < 1000)) || 
                                (live_state == 13 && (millis() - retract_dwell_start < 1000));
                if (!dwelling) swing_at_target = true; 
            }
        } else {
            swing.setSpeed(0);
        }
    }

    // ---------------------------------------------------------------------------
    // rotation control
    // ---------------------------------------------------------------------------
    if (live_state != -1 && live_state != 99 && !buzzer_req) {
        if (live_rot_target == ROTATE_HOME_RAW) {
            float homeErr = ROTATE_HOME_RAW - currentRotateRaw;
            if (homeErr > 180) homeErr -= 360;
            if (homeErr < -180) homeErr += 360;

            if (abs(homeErr) > TOLERANCE) {
                float dynamicOffset = (abs(homeErr) < 10.0) ? abs(homeErr) * 18.0 : 180.0;
                float rSpeed = -(homeErr * 55 + (homeErr > 0 ? dynamicOffset : -dynamicOffset)) * MICROSTEPS * 2;
                rotate.setSpeed(constrain(rSpeed, -7000*MICROSTEPS, 7000*MICROSTEPS));
                rotHomed = false; 
            } else {
                rotate.setSpeed(0);
                if (!rotHomed) {
                    rotate.setCurrentPosition(0);
                    rotHomed = true;
                }
                rErr = 0;
            }
        } else { // step-based brick theta
            float currentDeg = (float)rotate.currentPosition() / ROTATION_STEPS_PER_DEGREE;
            rErr = live_rot_target - currentDeg;
            
            float dynamicOffset = (abs(rErr) < 10.0) ? abs(rErr) * 18.0 : 180.0;
            float rSpeed = (rErr * 55 + (rErr > 0 ? dynamicOffset : -dynamicOffset)) * MICROSTEPS * 2;
            rotate.setSpeed(constrain(rSpeed, -7000*MICROSTEPS, 7000*MICROSTEPS));
            rotHomed = false; 
        }
    }

    // ---------------------------------------------------------------------------
    // actuators
    // ---------------------------------------------------------------------------
    if (live_state != 7 && live_state != 8)
        gripper.write(live_grip_open ? 200 : 122);
    
    // adhesive valve
    if (live_state == 12 || live_state == 13) {

        if (valve_delay_start == 0) {
            valve_delay_start = millis();
        }


        if (millis() - valve_delay_start >= 0) {
            valve_servo.write(VALVE_OPEN_POS);
        }
    } else if (live_state == 0) {
        valve_servo.write(VALVE_OPEN_POS);
    } else {

        valve_servo.write(VALVE_CLOSED_POS);
        valve_delay_start = 0;
    }

    analogWrite(EXTRACTOR_PIN, live_extractor_on ? 200 : 0);

    // ---------------------------------------------------------------------------
    // status publish @ 20hz
    // ---------------------------------------------------------------------------
    static uint32_t lastP = 0;
    static int last_state = -1;
    if (millis() - lastP > 50) {
        status_msg.data.data[STAT_HW_ID] = 1.0f;
        status_msg.data.data[STAT_POS_ALPHA] = currentSwingRaw;
        status_msg.data.data[STAT_POS_BETA] = currentRotateRaw;
        status_msg.data.data[STAT_GRIPPER_DETECT] = (digitalRead(LIMIT_SWITCH_GRIPPER) == HIGH) ? 1.0f : 0.0f;
        
        const float READY_TOLERANCE = 2.0;
        bool atGoal = (abs(sErr) <= READY_TOLERANCE);

        bool currently_dwelling = (live_state == 12 && (millis() - swing_dwell_start < 1000)) || 
                                  (live_state == 13 && (millis() - retract_dwell_start < 0));
        
        if (currently_dwelling) atGoal = false;
        
        if (live_state != last_state) {
            status_msg.data.data[STAT_TASK_COMPLETE] = 0.0f;
            last_state = live_state;
        } else {
            status_msg.data.data[STAT_TASK_COMPLETE] = (atGoal || swing_at_target) ? 1.0f : 0.0f;
        }

        rcl_publish(&publisher, &status_msg, NULL);
        lastP = millis();
    }
}