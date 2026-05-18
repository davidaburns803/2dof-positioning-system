#include "SerialComms.h"

void initSerialComms() {
    Serial.begin(115200);
    delay(500);
    printHelp();
}

void printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  theta phi      example: 45 120");
    Serial.println("  r theta phi    example: 1 45 120   r ignored");
    Serial.println("  zero           zero both encoders");
    Serial.println("  zero6          zero tilt encoder");
    Serial.println("  zero12         zero yaw encoder");
    Serial.println("  laser on");
    Serial.println("  laser off");
    Serial.println();
}

SerialCommand readSerialCommand() {
    SerialCommand cmd;
    cmd.type = CMD_NONE;
    cmd.thetaDeg = 0.0f;
    cmd.phiDeg = 0.0f;

    if (!Serial.available()) {
        return cmd;
    }

    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.length() == 0) {
        return cmd;
    }

    if (s.equalsIgnoreCase("zero")) {
        cmd.type = CMD_ZERO_ALL;
        return cmd;
    }

    if (s.equalsIgnoreCase("zero6")) {
        cmd.type = CMD_ZERO_SIX;
        return cmd;
    }

    if (s.equalsIgnoreCase("zero12")) {
        cmd.type = CMD_ZERO_TWELVE;
        return cmd;
    }

    if (s.equalsIgnoreCase("laser on")) {
        cmd.type = CMD_LASER_ON;
        return cmd;
    }

    if (s.equalsIgnoreCase("laser off")) {
        cmd.type = CMD_LASER_OFF;
        return cmd;
    }

    s.replace(",", " ");

    float a, b, c;
    int parsed = sscanf(s.c_str(), "%f %f %f", &a, &b, &c);

    if (parsed == 2) {
        cmd.type = CMD_TARGET;
        cmd.thetaDeg = a;
        cmd.phiDeg = b;
        return cmd;
    }

    if (parsed == 3) {
        cmd.type = CMD_TARGET;
        cmd.thetaDeg = b;
        cmd.phiDeg = c;
        return cmd;
    }

    Serial.println("Invalid command.");
    printHelp();

    return cmd;
}