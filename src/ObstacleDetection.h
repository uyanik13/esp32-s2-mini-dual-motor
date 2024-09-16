#ifndef OBSTACLEDETECTION_H
#define OBSTACLEDETECTION_H

#include <Arduino.h> 

class ObstacleDetection {
public:
    void setup();
    bool detectObstacle();
    void autonomousChangeDirection();

private:
    unsigned int getDistance();
    bool scanWithServo(int angle);
    bool scanLastDirection(String lastDirection);  // Now String will be recognized
};

#endif
