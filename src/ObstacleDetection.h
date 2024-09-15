#ifndef OBSTACLEDETECTION_H
#define OBSTACLEDETECTION_H

class ObstacleDetection {
public:
    void setup();
    bool detectObstacle();
private:
    unsigned int getDistance();
    void autonomousChangeDirection();
};

#endif
