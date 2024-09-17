#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>  // Include Arduino for String

#define MAX_BACK_MOTOR_SPEED 128
#define MAX_FRONT_MOTOR_SPEED 120

class MotorControl {
public:
    MotorControl();
    void setup();
    void handleWebSocketInput(const String& direction, int speed, float angle);  
    void processMotorControl(const String& direction, int backMotorSpeed, int frontMotorSpeed, float angle);
    void stopAll();
    void move(bool forward, int speed);
    String getLastDirection();

    // Make this method public so ObstacleDetection can call it
    void turnFrontMotor(bool left, int speed);  

private:
    void setupPins();
    void setupPWM();
    void checkMotorStatus();
    void stopFrontMotor();
    void setFrontMotorSpeed(int speed);
    void setBackMotorSpeed(int speed);
    void setFrontMotorDirection(int dir1, int dir2);
    void setBackMotorDirection(int dir1, int dir2);
    void setMotorStandby(bool standby);
    int whichMotorRuns(const String& direction);

    // Variables to store motor states
    bool frontMotorActive;
    bool backMotorActive;
    String lastDirection;
    bool motorTurning;
    unsigned long lastTurnTime;
    unsigned long lastMotorCommandTime;
    int currentSpeed;
    int lastSpeed;
    int softStartDelay;
    String lastTurnDirection;
};

#endif
