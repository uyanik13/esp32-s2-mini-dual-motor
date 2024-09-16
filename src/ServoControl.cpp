#include "ServoControl.h"
#include <Arduino.h>

#define SERVO_PIN 4
#define CENTER_ANGLE 90
#define LEFT_ANGLE 0
#define RIGHT_ANGLE 180
#define MOVE_DELAY 15  // Short delay between angle steps for smooth movement
#define STEP_SIZE 1     // How much the servo moves per step

Servo servo;

void ServoControl::setup() {
    servo.attach(SERVO_PIN);
    servo.write(CENTER_ANGLE); // Center position
    Serial.begin(9600);
    Serial.println("Servo initialized at center position.");
}

void ServoControl::moveServo(int targetAngle) {
    if (targetAngle < LEFT_ANGLE || targetAngle > RIGHT_ANGLE) {
        Serial.println("Invalid angle. Must be between 0 and 180.");
        return;
    }

    int currentAngle = servo.read();  // Read the current position of the servo

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

    Serial.print("Servo moved to: ");
    Serial.println(targetAngle);
}

void ServoControl::moveLeft() {
    Serial.println("Moving servo to the left...");
    moveServo(LEFT_ANGLE);
}

void ServoControl::moveRight() {
    Serial.println("Moving servo to the right...");
    moveServo(RIGHT_ANGLE);
}

void ServoControl::center() {
    Serial.println("Centering servo...");
    moveServo(CENTER_ANGLE);
}
