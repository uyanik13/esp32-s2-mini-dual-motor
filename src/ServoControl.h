#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <ESP32Servo.h>

class ServoControl {
public:
    ServoControl();  // Constructor
    void setup();
    void moveServo(int angle);
    void moveLeft();
    void moveRight();
    void center();

private:
    Servo servo;  // Servo object for controlling the servo motor
};

#endif
