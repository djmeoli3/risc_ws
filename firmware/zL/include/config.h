#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SYSTEM CONFIG ---
const int MICROSTEPS = 4;
#define LED_PIN 13
const int MOTOR_TICK_INTERVAL = 100; 

// --- STEPPER PINS (Z-Lead and Wheels) ---
#define LEAD_STEP_PIN 2
#define LEAD_DIR_PIN  3
#define LEAD_ENA_PIN  4
#define WHEEL_STEP_PIN 5
#define WHEEL_DIR_PIN 6
#define WHEEL_ENA_PIN 7

// --- CARRIAGE LIFT (H-BRIDGE) ---
#define LIFT_L_PWM    12
#define LIFT_R_PWM    11
#define LIFT_L_ENABLE 10
#define LIFT_R_ENABLE 9

// --- LIMIT SWITCHES ---
#define LIMIT_SWITCH_CLOSE 14 // Z-Axis Homing (Lead Screw)
#define LIMIT_SWITCH_FAR   15 // Z-Axis Max Travel (Lead Screw)

// Carriage Lift Limits
#define LIFT_LIMIT_LOWER  16
#define LIFT_LIMIT_UPPER  17 // Physical Hard Stop

// --- ULTRASONIC SENSOR (Carriage Detection) ---
#define LIFT_TRIG_PIN 19
#define LIFT_ECHO_PIN 20
const unsigned long BRICK_CONFIRM_DELAY = 2000; // 2 seconds debounce
const float BRICK_DETECT_DIST_CM = 10.0;        // Threshold for "Brick Present"

// --- H-BRIDGE SAFETY & SPEED ---
const int LIFT_SPEED_PWM = 150;    // 0-255 range (approx 70% speed)
const int H_BRIDGE_DEAD_TIME = 50; // ms to wait during direction changes

// --- PHYSICAL CONSTANTS ---
const float BASE_STEPS_PER_MM = 78.740157; 
const float STEPS_PER_MM = BASE_STEPS_PER_MM * (float)MICROSTEPS;
const float STEPS_PER_MM_Y = 3.7594; 

// --- MOTION PROFILES ---
const float MOVE_SPEED_SCALED    = 1000.0 * (float)MICROSTEPS;
const float HOMING_SPEED_SCALED  = 1000.0 * (float)MICROSTEPS;
const float LEAD_ACCEL           = 1500.0 * (float)MICROSTEPS;
const float WHEEL_MAX_SPEED      = 20.0   * (float)MICROSTEPS;
const float WHEEL_ACCEL          = 5.0   * (float)MICROSTEPS;

// --- INVERSION SETTINGS ---
const bool INVERT_Y_DIR = true; 

#endif