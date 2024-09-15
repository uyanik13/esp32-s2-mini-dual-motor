#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <ESP32Servo.h>

class ServoControl {
public:
    void setup();
    void moveServo(int angle);
};

#endif
