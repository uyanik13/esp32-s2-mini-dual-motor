#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>  // Include the Servo library for easier control

class ServoControl {
public:
    ServoControl();  // Constructor
    void setup();    // Initialize the servo
    void moveServo(int angle);  // Move the servo to a specified angle
    void moveLeft();            // Move the servo to the leftmost position
    void moveRight();           // Move the servo to the rightmost position
    void center();              // Center the servo

private:
    Servo servo;  // Servo object for controlling the servo motor

    // Define constants for controlling the servo
    static const int SERVO_PIN = 6;      // Pin connected to the servo
    static const int LEFT_ANGLE = 0;     // Minimum servo angle
    static const int RIGHT_ANGLE = 180;  // Maximum servo angle
    static const int CENTER_ANGLE = 90;  // Center angle for the servo
};

#endif
