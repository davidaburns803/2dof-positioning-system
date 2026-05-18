#pragma once

#include <Arduino.h>

enum CommandType {
    CMD_NONE,
    CMD_TARGET,
    CMD_ZERO_ALL,
    CMD_ZERO_SIX,
    CMD_ZERO_TWELVE,
    CMD_LASER_ON,
    CMD_LASER_OFF
};

struct SerialCommand {
    CommandType type;
    float thetaDeg;
    float phiDeg;
};

void initSerialComms();
SerialCommand readSerialCommand();
void printHelp();