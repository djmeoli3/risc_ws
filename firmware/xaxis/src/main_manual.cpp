#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <AccelStepper.h>
#include "config.h"

// MOTOR OBJECTS
AccelStepper xaxis(AccelStepper::DRIVER, LEAD_STEP_PIN, LEAD_DIR_PIN);
AccelStepper conveyor(AccelStepper::DRIVER, CONVEYOR_STEP_PIN, CONVEYOR_DIR_PIN);
AccelStepper pump(AccelStepper::DRIVER, PUMP_STEP_PIN, PUMP_DIR_PIN);

IntervalTimer motorTimer; 

// STATE & ENCODER
enum State { IDLE, HOMING_SEARCH, MOVING, BOUNCING };
volatile State currentState = IDLE;
volatile bool isFeeding = false;
volatile bool isPumping = false;
volatile long encoderTicks = 0;
int bounceDirection = 1;

// ROS 2 OBJECTS
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String msg_sub;
std_msgs__msg__String msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
bool micro_ros_connected = false;

// ENCODER INTERRUPT
void readEncoder() {
    if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) encoderTicks++;
    else encoderTicks--;
}

// HIGH SPEED MOTOR INTERRUPT
void motorTick() {
    if (isFeeding) conveyor.runSpeed();
    if (isPumping) pump.runSpeed();
    
    if (currentState == BOUNCING) xaxis.runSpeed();
    else if (currentState == HOMING_SEARCH || currentState == MOVING) xaxis.run();
}

void applyHardStop() {
    isFeeding = false;
    isPumping = false;
    conveyor.setSpeed(0);
    pump.setSpeed(0);
    xaxis.setSpeed(0);
    xaxis.moveTo(xaxis.currentPosition());
    
    float backoff = 4.0 * STEPS_PER_MM;
    if (digitalRead(LIMIT_SWITCH_FAR) == HIGH) xaxis.moveTo(xaxis.currentPosition() + (long)round(backoff)); 
    else if (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) xaxis.moveTo(xaxis.currentPosition() - (long)round(backoff));
    
    xaxis.setMaxSpeed(HOMING_SPEED_SCALED);
    while(xaxis.distanceToGo() != 0) {
        xaxis.run();
    }
    
    xaxis.setCurrentPosition(0);
    encoderTicks = 0; 
    currentState = IDLE;
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__String * msg_received = (const std_msgs__msg__String *)msgin;
    if (msg_received->data.data == NULL) return;
    
    String input = String(msg_received->data.data);
    input.trim();
    
    char cmd = input.charAt(0);
    switch (cmd) {
        case 'h': 
            xaxis.setMaxSpeed(HOMING_SPEED_SCALED);
            xaxis.moveTo(1000000); 
            currentState = HOMING_SEARCH; 
            break;
        case 'm': 
            {
                float targetMM = input.substring(1).toFloat();
                xaxis.setMaxSpeed(MOVE_SPEED_SCALED);
                xaxis.moveTo((long)round(-targetMM * STEPS_PER_MM));
                currentState = MOVING;
            }
            break;
        case 'b': currentState = BOUNCING; break;
        case 'f': 
            isFeeding = !isFeeding; 
            conveyor.setSpeed(isFeeding ? CONVEYOR_SPEED_STEPS : 0);
            break;
        case 'p': //not implmented
            isPumping = !isPumping;
            pump.setSpeed(isPumping ? PUMP_SPEED_STEPS : 0);
            break;
        case 'x': 
            currentState = IDLE;
            isFeeding = false; isPumping = false;
            conveyor.setSpeed(0); pump.setSpeed(0);
            xaxis.moveTo(xaxis.currentPosition());
            if (digitalRead(LIMIT_SWITCH_FAR) == HIGH || digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) applyHardStop();
            break;
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_FAR, INPUT_PULLUP);
    pinMode(LEAD_ENA_PIN, OUTPUT); 
    pinMode(CONVEYOR_ENA_PIN, OUTPUT);
    pinMode(PUMP_ENA_PIN, OUTPUT);
    pinMode(PROX_TRIG_PIN, OUTPUT);
    pinMode(PROX_ECHO_PIN, INPUT);
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);

    digitalWrite(LEAD_ENA_PIN, LOW); 
    digitalWrite(CONVEYOR_ENA_PIN, LOW);
    digitalWrite(PUMP_ENA_PIN, LOW);

    xaxis.setMaxSpeed(MOVE_SPEED_SCALED); 
    xaxis.setAcceleration(500.0 * (float)MICROSTEPS); 
    conveyor.setMaxSpeed(CONVEYOR_SPEED_STEPS);
    pump.setMaxSpeed(PUMP_SPEED_STEPS);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), readEncoder, CHANGE);
    motorTimer.begin(motorTick, 100); 

    set_microros_transports();
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        rclc_node_init_default(&node, "xaxis_node", "", &support);
        msg_sub.data.capacity = 100;
        msg_sub.data.data = (char*) malloc(msg_sub.data.capacity);

        rclc_subscription_init_default(&subscriber, &node, 
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "xaxis_cmd");

        rclc_publisher_init_default(&publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "xaxis_status");
        
        msg_pub.data.capacity = 100;
        msg_pub.data.data = (char*) malloc(msg_pub.data.capacity);

        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
        micro_ros_connected = true;
    }
}

void loop() {
    if (!micro_ros_connected) return;

    switch (currentState) {
        case HOMING_SEARCH:
            if (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) applyHardStop();
            break;
        case MOVING:
            if (digitalRead(LIMIT_SWITCH_FAR) == HIGH || digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) applyHardStop();
            else if (xaxis.distanceToGo() == 0) currentState = IDLE;
            break;
        case BOUNCING:
            if (bounceDirection == 1 && digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) bounceDirection = -1;
            if (bounceDirection == -1 && digitalRead(LIMIT_SWITCH_FAR) == HIGH) bounceDirection = 1;
            xaxis.setSpeed(MOVE_SPEED_SCALED * (float)bounceDirection);
            break;
        case IDLE: break;
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    static uint32_t lastPub = 0;
    if (millis() - lastPub > 100) {
        digitalWrite(PROX_TRIG_PIN, LOW); delayMicroseconds(2);
        digitalWrite(PROX_TRIG_PIN, HIGH); delayMicroseconds(10);
        digitalWrite(PROX_TRIG_PIN, LOW);
        long dur = pulseIn(PROX_ECHO_PIN, HIGH, 10000); 
        float dist = (dur > 0) ? (dur * 0.034 / 2) : -1.0;

        String status = "ST:" + String(currentState) + " X:" + String(xaxis.currentPosition()) + " E:" + String(encoderTicks) + " P:" + String(dist, 1);
        snprintf(msg_pub.data.data, msg_pub.data.capacity, "%s", status.c_str());
        msg_pub.data.size = strlen(msg_pub.data.data);
        rcl_publish(&publisher, &msg_pub, NULL);
        lastPub = millis();
    }
}