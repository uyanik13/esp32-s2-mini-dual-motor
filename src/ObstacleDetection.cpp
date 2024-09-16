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
        Serial.println("Obstacle detected! Changing direction.");
        autonomousChangeDirection();
        return true;
    }
    return false;
}

void ObstacleDetection::autonomousChangeDirection() {
    motorControl.stopAll();
    
    bool clearLeft = scanWithServo(0);
    bool clearCenter = scanWithServo(90);
    bool clearRight = scanWithServo(180);

    if (clearLeft) {
        motorControl.turnFrontMotor(true, MAX_FRONT_MOTOR_SPEED);  // Now it's recognized
    } else if (clearCenter) {
        motorControl.move(true, MAX_BACK_MOTOR_SPEED);  // Now it's recognized
    } else if (clearRight) {
        motorControl.turnFrontMotor(false, MAX_FRONT_MOTOR_SPEED);  // Now it's recognized
    } else {
        motorControl.stopAll();
    }
}

bool ObstacleDetection::scanWithServo(int angle) {
    servoControl.moveServo(angle);
    delay(500);
    unsigned int distance = getDistance();
    return distance > 20;  // Return true if the path is clear
}
