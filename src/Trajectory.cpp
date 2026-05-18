#include "Trajectory.h"

static const TrajectoryPoint squarePoints[] = {
    {-8.0f, 55.0f},
    { 8.0f, 55.0f},
    { 8.0f, 45.0f},
    {-8.0f, 45.0f}
};

static const int squarePointCount = 4;
static int squareIndex = 0;

void initSquareTrajectory() {
    squareIndex = 0;
}

TrajectoryPoint getCurrentSquarePoint() {
    return squarePoints[squareIndex];
}

void advanceSquarePoint() {
    squareIndex++;

    if (squareIndex >= squarePointCount) {
        squareIndex = 0;
    }
}

int getSquareIndex() {
    return squareIndex;
}