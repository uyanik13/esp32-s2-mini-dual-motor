#ifndef OBSTACLEDETECTION_H
#define OBSTACLEDETECTION_H

class ObstacleDetection {
public:
    ObstacleDetection();
    void setup();
    unsigned int getDistance();
    bool detectObstacle();
    void autonomousChangeDirection();

private:
    bool scanWithServo(int angle);
    int lastScanDirection = 90;  // Start with center as default
};

#endif
