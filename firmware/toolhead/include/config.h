#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- SYSTEM CONFIG ---
const int MICROSTEPS = 8;
#define LED_PIN 13

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


// --- LIMIT SWITCH DETECTION ---
#define LIMIT_SWITCH_GRIPPER 23

// --- ENCODER CONFIG ---
#define AS5600_ADDR 0x36

// --- CALIBRATED LIMITS ---
const float ROTATE_HOME_RAW = 267.0;
const float SWING_UP_RAW    = 248.0;
const float SWING_DOWN_RAW  = 68.9;
const float TOLERANCE       = 1.0; 

#endif