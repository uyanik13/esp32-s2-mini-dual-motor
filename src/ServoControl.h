#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Arduino.h>

class ServoControl {
public:
    ServoControl();  // Constructor
    void setup();
    void moveServo(int angle);  // Move the servo to a specified angle
    void moveLeft();            // Move the servo to the leftmost position
    void moveRight();           // Move the servo to the rightmost position
    void center();              // Center the servo

private:
    // Helper function to map the angle to PWM duty cycle
    int mapAngleToDutyCycle(int angle);

    // Define constants for controlling the servo
    static const int SERVO_PIN = 6;      // Pin connected to the servo
    static const int LEFT_ANGLE = 0;     // Minimum servo angle
    static const int RIGHT_ANGLE = 180;  // Maximum servo angle
    static const int CENTER_ANGLE = 90;  // Center angle for the servo
    static const int PWM_CHANNEL_SERVO = 2; // PWM channel for servo
    static const int PWM_FREQUENCY = 50; // Frequency for PWM (50Hz for servos)
    static const int PWM_RESOLUTION = 16; // 16-bit PWM resolution
    static const int MIN_PULSE_WIDTH = 544;  // Minimum pulse width in microseconds
    static const int MAX_PULSE_WIDTH = 2400; // Maximum pulse width in microseconds
};

#endif
