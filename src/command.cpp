#include <Arduino.h>

#include "Pins.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include "PDControl.h"
#include "LaserSwitch.h"
#include "SerialComms.h"

PDController yawPD;
PDController tiltPD;

bool haveTarget = false;

long yawTargetCounts = 0;
long tiltTargetCounts = 0;

int lastYawDirection = 0;
int lastTiltDirection = 0;

long yawDegToCounts(float deg) {
    return (long)lroundf(deg * COUNTS_PER_DEG_12V);
}

long tiltDegToCounts(float deg) {
    return (long)lroundf(deg * COUNTS_PER_DEG_6V);
}

float yawCountsToDeg(long counts) {
    return (float)counts / COUNTS_PER_DEG_12V;
}

float tiltCountsToDeg(long counts) {
    return (float)counts / COUNTS_PER_DEG_6V;
}

int signOf(long value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

long applyBacklashOnReversal(long targetCounts, long actualCounts, long backlashCounts, int& lastDirection) {
    long err = targetCounts - actualCounts;
    int currentDirection = signOf(err);

    if (currentDirection == 0) {
        return targetCounts;
    }

    if (lastDirection != 0 && currentDirection != lastDirection) {
        targetCounts += currentDirection * backlashCounts;
    }

    lastDirection = currentDirection;

    return targetCounts;
}

void handleCommand(const SerialCommand& cmd) {
    if (cmd.type == CMD_NONE) return;

    if (cmd.type == CMD_ZERO_ALL) {
        writeSixCountsAtomic(0);
        writeTwelveCountsAtomic(0);
        haveTarget = false;
        stopAllMotors();
        Serial.println("Zeroed both encoders.");
        return;
    }

    if (cmd.type == CMD_ZERO_SIX) {
        writeSixCountsAtomic(0);
        haveTarget = false;
        stopMotor(MOTOR_SIX);
        Serial.println("Zeroed 6V tilt encoder.");
        return;
    }

    if (cmd.type == CMD_ZERO_TWELVE) {
        writeTwelveCountsAtomic(0);
        haveTarget = false;
        stopMotor(MOTOR_TWELVE);
        Serial.println("Zeroed 12V yaw encoder.");
        return;
    }

    if (cmd.type == CMD_LASER_ON) {
        laserOn();
        Serial.println("Laser ON.");
        return;
    }

    if (cmd.type == CMD_LASER_OFF) {
        laserOff();
        Serial.println("Laser OFF.");
        return;
    }

    if (cmd.type == CMD_TARGET) {
        float theta = constrain(cmd.thetaDeg, YAW_MIN_DEG, YAW_MAX_DEG);
        float phi = constrain(cmd.phiDeg, TILT_MIN_DEG, TILT_MAX_DEG);

        long yawRawTarget = yawDegToCounts(theta);
        long tiltRawTarget = tiltDegToCounts(phi);

        long yawActual = readTwelveCountsAtomic();
        long tiltActual = readSixCountsAtomic();

        yawTargetCounts = applyBacklashOnReversal(
            yawRawTarget,
            yawActual,
            BACKLASH_12V_COUNTS,
            lastYawDirection
        );

        tiltTargetCounts = applyBacklashOnReversal(
            tiltRawTarget,
            tiltActual,
            BACKLASH_6V_COUNTS,
            lastTiltDirection
        );

        haveTarget = true;

        Serial.print("Target theta yaw deg: ");
        Serial.print(theta, 2);
        Serial.print("  counts: ");
        Serial.println(yawTargetCounts);

        Serial.print("Target phi tilt deg: ");
        Serial.print(phi, 2);
        Serial.print("  counts: ");
        Serial.println(tiltTargetCounts);

        return;
    }
}

void setup() {
    initSerialComms();

    initMotors();
    initEncoders();
    initLaser();

    initPD(yawPD, 0.35f, 0.002f, 180, 35, 12);
    initPD(tiltPD, 0.30f, 0.002f, 180, 35, 20);

    laserOn();

    Serial.println("Command mode ready.");
}

void loop() {
    SerialCommand cmd = readSerialCommand();
    handleCommand(cmd);

    static unsigned long lastT = 0;
    unsigned long now = millis();
    unsigned long dtMs = now - lastT;

    if (dtMs < 10) return;

    lastT = now;
    float dtSeconds = dtMs / 1000.0f;

    long yawActual = readTwelveCountsAtomic();
    long tiltActual = readSixCountsAtomic();

    if (!haveTarget) {
        stopAllMotors();
        return;
    }

    bool yawDone = atTarget(yawPD, yawTargetCounts, yawActual);
    bool tiltDone = atTarget(tiltPD, tiltTargetCounts, tiltActual);

    if (yawDone) {
        stopMotor(MOTOR_TWELVE);
    } else {
        int yawPWM = updatePD(yawPD, yawTargetCounts, yawActual, dtSeconds);
        setMotorPWM(MOTOR_TWELVE, yawPWM);
    }

    if (tiltDone) {
        stopMotor(MOTOR_SIX);
    } else {
        int tiltPWM = updatePD(tiltPD, tiltTargetCounts, tiltActual, dtSeconds);
        setMotorPWM(MOTOR_SIX, tiltPWM);
    }

    if (yawDone && tiltDone) {
        stopAllMotors();

        Serial.print("Reached target. Yaw deg: ");
        Serial.print(yawCountsToDeg(yawActual), 2);
        Serial.print("  Tilt deg: ");
        Serial.println(tiltCountsToDeg(tiltActual), 2);

        haveTarget = false;
    }
}