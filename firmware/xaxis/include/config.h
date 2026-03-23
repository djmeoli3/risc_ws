#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SYSTEM CONFIG ---
const int MICROSTEPS = 4;

// --- STEPPER PINS ---
#define LEAD_STEP_PIN 2
#define LEAD_DIR_PIN  3
#define LEAD_ENA_PIN  4

#define CONVEYOR_STEP_PIN 5
#define CONVEYOR_DIR_PIN  6
#define CONVEYOR_ENA_PIN  7

#define ENCODER_A_PIN 14 
#define ENCODER_B_PIN 15
#define ENCODER_Z_PIN 8

// --- PUMP PINS  ---
#define PUMP_STEP_PIN 18  
#define PUMP_DIR_PIN  19
#define PUMP_ENA_PIN  17

#define PROX_TRIG_PIN 20
#define PROX_ECHO_PIN 21

#define LIMIT_SWITCH_CLOSE 22
#define LIMIT_SWITCH_FAR   23 

#define LED_PIN 13

// --- PHYSICAL CONSTANTS ---
const float BASE_STEPS_PER_MM = 78.740157; 
const float STEPS_PER_MM = BASE_STEPS_PER_MM * (float)MICROSTEPS;

// --- SPEEDS ---
const float MOVE_SPEED_SCALED    = 1000.0 * (float)MICROSTEPS;
const float HOMING_SPEED_SCALED  = 1000.0 * (float)MICROSTEPS;
const float CONVEYOR_SPEED_STEPS = 400.0 * (float)MICROSTEPS;
const float PUMP_SPEED_STEPS     = 200.0 * (float)MICROSTEPS; //initial placeholder

#endif