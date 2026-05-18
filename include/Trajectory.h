#pragma once

#include <Arduino.h>

struct TrajectoryPoint {
    float thetaDeg; // yaw
    float phiDeg;   // tilt
};

void initSquareTrajectory();
TrajectoryPoint getCurrentSquarePoint();
void advanceSquarePoint();
int getSquareIndex();