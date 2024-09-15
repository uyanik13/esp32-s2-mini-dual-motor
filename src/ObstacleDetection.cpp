#include "ObstacleDetection.h"
#include "MotorControl.h"
#include <Arduino.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 1

extern MotorControl motorControl;
extern bool obstacleDetected;

void ObstacleDetection::setup() {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

unsigned int ObstacleDetection::getDistance() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    unsigned int distance = duration * 0.034 / 2;
    return distance;
}

bool ObstacleDetection::detectObstacle() {
    unsigned int distance = getDistance();
    if (distance < 10) {
        obstacleDetected = true;
        Serial.println("Obstacle detected! Changing direction.");
        motorControl.stopAll();
        autonomousChangeDirection(); // Autonomous direction change logic
        return true;
    }
    obstacleDetected = false;
    return false;
}

void ObstacleDetection::autonomousChangeDirection() {
    // motorControl.moveBackward(200); // Move backward slightly
    // delay(500);
    // motorControl.turnLeft(150); // Turn left to avoid obstacle
    // delay(500);
    // motorControl.moveForward(200); // Move forward after avoiding
    // delay(500);
}
