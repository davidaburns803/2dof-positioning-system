#include <Arduino.h>

#include "Pins.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include "PDControl.h"
#include "LaserSwitch.h"
#include "Trajectory.h"

PDController yawPD;
PDController tiltPD;

long yawTargetCounts = 0;
long tiltTargetCounts = 0;

long yawDegToCounts(float deg) {
    return (long)lroundf(deg * COUNTS_PER_DEG_12V);
}

long tiltDegToCounts(float deg) {
    return (long)lroundf(deg * COUNTS_PER_DEG_6V);
}

void loadCurrentSquareTarget() {
    TrajectoryPoint pt = getCurrentSquarePoint();

    yawTargetCounts = yawDegToCounts(pt.thetaDeg);
    tiltTargetCounts = tiltDegToCounts(pt.phiDeg);
}

void setup() {
    initMotors(); 
    delay(3000);

    initEncoders();
    initLaser();
    initSquareTrajectory();

    initPD(yawPD, 0.35f, 0.002f, 180, 38, 12);
    initPD(tiltPD, 0.30f, 0.002f, 180, 45, 20);

    laserOn();

    delay(1000);

    loadCurrentSquareTarget();
}

void loop() {
    static unsigned long lastT = 0;
    unsigned long now = millis();
    unsigned long dtMs = now - lastT;

    if (dtMs < 10) return;

    lastT = now;
    float dtSeconds = dtMs / 1000.0f;

    long yawActual = readTwelveCountsAtomic();
    long tiltActual = readSixCountsAtomic();

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

        delay(500);

        advanceSquarePoint();
        loadCurrentSquareTarget();
    }
}