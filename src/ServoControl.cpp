#include "ServoControl.h"
#include <Arduino.h>
#include <ESP32Servo.h>  // Use the Servo library for easier control

#define SERVO_PIN 6
#define CENTER_ANGLE 90
#define LEFT_ANGLE 180
#define RIGHT_ANGLE 0
#define MOVE_DELAY 15  // Short delay between angle steps for smooth movement
#define STEP_SIZE 1     // How much the servo moves per step

ServoControl::ServoControl() : currentAngle(CENTER_ANGLE) {}  // Initialize with center angle

// Initialize the servo and set it to the center position
void ServoControl::setup() {
    servo.attach(SERVO_PIN);  // Attach the servo to the pin
    servo.write(CENTER_ANGLE);  // Move servo to center position
    currentAngle = CENTER_ANGLE;  // Keep track of the current angle
    Serial.println("Servo initialized at center position.");
}

// Function to move the servo to a specified angle
void ServoControl::moveServo(int targetAngle) {
    // Ensure target angle is within valid range
    if (targetAngle < RIGHT_ANGLE || targetAngle > LEFT_ANGLE) {
        Serial.println("Invalid angle. Must be between 0 and 180.");
        return;
    }

    Serial.printf("Moving servo to: %d\n", targetAngle);

    // Gradually move the servo to the target angle
    if (targetAngle > currentAngle) {
        for (int angle = currentAngle; angle <= targetAngle; angle += STEP_SIZE) {
            servo.write(angle);
            delay(MOVE_DELAY);
        }
    } else {
        for (int angle = currentAngle; angle >= targetAngle; angle -= STEP_SIZE) {
            servo.write(angle);
            delay(MOVE_DELAY);
        }
    }

    // Update current angle
    currentAngle = targetAngle;
}

// Function to move the servo to the leftmost position
void ServoControl::moveLeft() {
    moveServo(LEFT_ANGLE);
}

// Function to move the servo to the rightmost position
void ServoControl::moveRight() {
    moveServo(RIGHT_ANGLE);
}

// Function to center the servo
void ServoControl::center() {
    moveServo(CENTER_ANGLE);
}

// Function to stop the servo
void ServoControl::stopServo() {
    servo.detach();  // Detach the servo to stop it
    Serial.println("Servo stopped.");
}

// Function to attach the servo
void ServoControl::attachServo() {
    servo.attach(SERVO_PIN);  // Attach the servo to the pin
    servo.write(currentAngle);  // Restore the last known position
    Serial.println("Servo attached.");
}
