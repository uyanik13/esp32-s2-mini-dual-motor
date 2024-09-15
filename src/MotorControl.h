#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl();
    void setup();  
    void setupPins();  
    void handleWebSocketInput(const String& direction, int speed, float angle);
    void processMotorControl(const String& direction, int backMotorSpeed, int frontMotorSpeed, float angle);
    void setMotorStandby(bool standby);  
    void stopAll();  
    void setFrontMotorDirection(int dir1, int dir2);  // Corrected method name
    void setBackMotorDirection(int dir1, int dir2);   // Corrected method name
    void setFrontMotorSpeed(int speed);               // Corrected method name
    void setBackMotorSpeed(int speed);                // Corrected method name
    void move(bool forward, int speed);
    void turnFrontMotor(bool left, int speed);  
    void validateDirectionAndAngle(const String& direction, float angle); 
    void stopFrontMotor();  
    void adjustFrontMotorByJoystick(int speed, float angle);  
    void setupPWM();  
    void checkMotorStatus();  
    int whichMotorRuns(const String& direction);

private:
    int currentSpeed;  
    int softStartDelay;
    unsigned long lastMotorCommandTime;
    bool frontMotorActive;
    bool backMotorActive;
    bool motorTurning;
    unsigned long lastTurnTime;
    String lastTurnDirection;
    String lastDirection;
    int lastSpeed;
};

#endif // MOTOR_CONTROL_H
