#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <ESP32Servo.h>

class ServoControl {
public:
    ServoControl();
    void setup();
    void moveServo(int targetAngle);
    void moveLeft();
    void moveRight();
    void center();
    void stopServo();
    void attachServo();

private:
    Servo servo;
    int currentAngle;  // Track the current angle of the servo
};

#endif // SERVOCONTROL_H
