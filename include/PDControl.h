#pragma once

#include <Arduino.h>

struct PDController {
    float kp;
    float kd;

    int pwmMax;
    int pwmMin;

    long allowableErrorCounts;

    long prevCounts;
    float velFilt;
};

void initPD(PDController& pd, float kp, float kd, int pwmMax, int pwmMin, long allowableErrorCounts);

int updatePD(PDController& pd, long targetCounts, long actualCounts, float dtSeconds);

bool atTarget(PDController& pd, long targetCounts, long actualCounts);