#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SYSTEM CONFIG ---
const int MICROSTEPS = 4;
#define LED_PIN 13

// --- PHYSICAL CONSTANTS ---
const float ROTATION_GEAR_RATIO = 7.2;         
//const float SWING_GEAR_RATIO    = 5.3333;
const float ROTATION_STEPS_PER_DEGREE = 64.0;
//const float SWING_STEPS_PER_DEGREE    = 47.4074;

// --- STEPPER PINS ---
#define SWING_STEP_PIN 5
#define SWING_DIR_PIN  6
#define SWING_ENA_PIN  7 

#define ROTATE_STEP_PIN 8
#define ROTATE_DIR_PIN  9 
#define ROTATE_ENA_PIN  10 

// --- PERIPHERALS ---
#define EXTRACTOR_PIN 14 
#define GRIPPER_SERVO_PIN 15
#define ADHESIVE_VALVE_PIN 20
#define LIMIT_SWITCH_GRIPPER 23


// --- ENCODER CONFIG ---
#define AS5600_ADDR 0x36

// --- MOTION LIMITS ---
const float MAX_SPEED_STEPPER = 4000.0;
const float MAX_SPEED_CONSTRAIN = 3500.0;
const int MOTOR_TICK_INTERVAL = 100;

// --- GRIPPER SERVO POSITIONS ---
const int GRIP_OPEN_POS = 179;
const int GRIP_CLOSED_POS = 80;

// --- STEPPER TUNING ---
const float ROTATE_ACCEL = 1000.0;

// --- CONTROL GAINS (PID-ISH) ---
const float SWING_GAIN = 55.0;
const float ROTATE_GAIN = 55.0;
const float STARTUP_SWING_GAIN = 30.0;
const float BASE_OFFSET = 180.0;
const float DECEL_THRESHOLD = 10.0;

// --- CALIBRATED LIMITS ---
const float ROTATE_HOME_RAW = 268.0;
const float SWING_UP_RAW    = 248.0;
const float SWING_DOWN_RAW  = 68.9;
const float TOLERANCE       = 1.0; 
const int VALVE_OPEN_POS   = 40;
const int VALVE_CLOSED_POS = 120;
const float SWING_PUMP_START = 266.0; 
const float SWING_PUMP_END   = 275.5;

#endif