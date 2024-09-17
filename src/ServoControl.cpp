#include "ServoControl.h"
#include <Arduino.h>
#include <ESP32Servo.h>  // Use the Servo library for easier control

#define SERVO_PIN 6
#define CENTER_ANGLE 90
#define LEFT_ANGLE 0
#define RIGHT_ANGLE 180
#define MOVE_DELAY 15  // Short delay between angle steps for smooth movement
#define STEP_SIZE 1     // How much the servo moves per step

ServoControl::ServoControl() {}

// Initialize the servo and set it to the center position
void ServoControl::setup() {
    servo.attach(SERVO_PIN);  // Attach the servo to the pin
    servo.write(CENTER_ANGLE);  // Move servo to center position
    Serial.println("Servo initialized at center position.");
}

// Function to move the servo to a specified angle
void ServoControl::moveServo(int targetAngle) {
    if (targetAngle < LEFT_ANGLE || targetAngle > RIGHT_ANGLE) {
        Serial.println("Invalid angle. Must be between 0 and 180.");
        return;
    }

    Serial.print("Moving servo to: ");
    Serial.println(targetAngle);

    // Gradually move the servo to the target angle
    int currentAngle = servo.read();
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

    Serial.print("Servo moved to: ");
    Serial.println(targetAngle);
}

// Function to move the servo to the leftmost position
void ServoControl::moveLeft() {
    Serial.println("Moving servo to the left...");
    moveServo(LEFT_ANGLE);
}

// Function to move the servo to the rightmost position
void ServoControl::moveRight() {
    Serial.println("Moving servo to the right...");
    moveServo(RIGHT_ANGLE);
}

// Function to center the servo
void ServoControl::center() {
    Serial.println("Centering servo...");
    moveServo(CENTER_ANGLE);
}
