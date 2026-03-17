#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SYSTEM CONFIG ---
const int MICROSTEPS = 4;

// --- STEPPER PINS (Consistent with X-Axis) ---
#define LEAD_STEP_PIN 2
#define LEAD_DIR_PIN  3
#define LEAD_ENA_PIN  4

// --- LIMIT SWITCHES (Moved for Z-Axis) ---
#define LIMIT_SWITCH_CLOSE 14
#define LIMIT_SWITCH_FAR   15 

#define LED_PIN 13

// --- PHYSICAL CONSTANTS ---
const float BASE_STEPS_PER_MM = 78.740157; 
const float STEPS_PER_MM = BASE_STEPS_PER_MM * (float)MICROSTEPS;

// --- SPEEDS ---
const float MOVE_SPEED_SCALED    = 1000.0 * (float)MICROSTEPS;
const float HOMING_SPEED_SCALED  = 400.0 * (float)MICROSTEPS;

#endif