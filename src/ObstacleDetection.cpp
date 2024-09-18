#include "ObstacleDetection.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include <Arduino.h>
#include <NewPing.h>

extern ServoControl servoControl;
extern MotorControl motorControl;

#define TRIGGER_PIN 2
#define ECHO_PIN 1
#define MAX_DISTANCE 200  // Max distance for obstacle detection

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

ObstacleDetection::ObstacleDetection() {}

void ObstacleDetection::setup() {
    Serial.begin(9600);
    servoControl.setup();
}

unsigned int ObstacleDetection::getDistance() {
    delay(70);
    unsigned int distance = sonar.ping_cm();
    return distance == 0 ? MAX_DISTANCE : distance;
}

bool ObstacleDetection::detectObstacle() {
    unsigned int distance = getDistance();
    if (distance < 15) {
        Serial.println("Obstacle detected! Stopping all motors and scanning for alternatives.");
        motorControl.stopAll();  // Stop all motors when an obstacle is detected
        autonomousChangeDirection();  // Scan for alternative paths
        return true;
    }
    return false;
}

void ObstacleDetection::autonomousChangeDirection() {
    servoControl.attachServo();  // Attach the servo before scanning

    // Alternate between scanning left and right based on last direction
    if (lastScanDirection == 90 || lastScanDirection == 180) {
        // First, scan left
        if (scanWithServo(0)) {  // Scan left
            Serial.println("Clear path found on the left. Returning to center and allowing manual control.");
            lastScanDirection = 0;  // Update the last direction scanned
            servoControl.center();  // Move servo back to center position
            delay(500);  // Wait for the servo to reach the center

            servoControl.stopServo();  // Stop servo after scanning
            return;  // Exit if left is clear
        }
    }

    // Next, scan right
    if (lastScanDirection == 0 || lastScanDirection == 90) {
        if (scanWithServo(180)) {  // Scan right
            Serial.println("Clear path found on the right. Returning to center and allowing manual control.");
            lastScanDirection = 180;  // Update the last direction scanned
            servoControl.center();  // Move servo back to center position
            delay(500);  // Wait for the servo to reach the center

            servoControl.stopServo();  // Stop servo after scanning
            return;  // Exit if right is clear
        }
    }

    // Lastly, scan center if no side is clear
    if (scanWithServo(90)) {  // Scan center
        Serial.println("Clear path found in the center. Stopping servo and allowing manual control.");
        lastScanDirection = 90;  // Update the last direction scanned
        delay(500);  // Wait for the servo to stabilize
        servoControl.stopServo();  // Stop servo after scanning
        return;  // Exit if center is clear
    }

    // If no clear path is found
    Serial.println("No clear path found. Waiting for user command.");
    motorControl.stopAll();  // Keep the motors stopped if no clear path is found
    servoControl.stopServo();  // Stop the servo even if no path is clear
}

bool ObstacleDetection::scanWithServo(int angle) {
    servoControl.moveServo(angle);  // Move the servo to the specified angle
    delay(500);  // Wait for the servo to reach the position
    unsigned int distance = getDistance();
    Serial.printf("Distance at angle %d: %d cm\n", angle, distance);
    return distance > 30;  // Return true if the path is clear (distance > 15cm)
}
