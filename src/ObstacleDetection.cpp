#include "ObstacleDetection.h"
#include "MotorControl.h"
#include <Arduino.h>
#include <NewPing.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 1
#define MAX_DISTANCE 200 // Set maximum distance for ping (in cm)

extern MotorControl motorControl;
extern bool obstacleDetected;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Initialize NewPing with trigger, echo, and max distance

void ObstacleDetection::setup() {
    // No need for pinMode as NewPing handles it internally
}

unsigned int ObstacleDetection::getDistance() {
    delay(70);
    unsigned int distance = sonar.ping_cm(); // Get distance in cm using NewPing
    return distance == 0 ? MAX_DISTANCE : distance; // If no ping detected, return max distance
}

bool ObstacleDetection::detectObstacle() {
    unsigned int distance = getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
    if (distance < 15) { // Threshold for detecting obstacles
        obstacleDetected = true;
        Serial.println("Obstacle detected! Changing direction.");
        autonomousChangeDirection(); // Autonomous direction change logic
        return true;
    }
    obstacleDetected = false;
    return false;
}

void ObstacleDetection::autonomousChangeDirection() {
    Serial.println("Autonomous direction change logic...");

    // motorControl.moveBackward(200); // Move backward slightly
    // delay(500);
    // motorControl.turnLeft(150); // Turn left to avoid obstacle
    // delay(500);
    // motorControl.moveForward(200); // Move forward after avoiding
    // delay(500);
}

