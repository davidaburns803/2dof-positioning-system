#pragma once

#include <Arduino.h>

enum MotorID {
    MOTOR_SIX,
    MOTOR_TWELVE
};

void initMotors();

void setMotorPWM(MotorID motor, int pwmSigned);
void stopMotor(MotorID motor);
void stopAllMotors();