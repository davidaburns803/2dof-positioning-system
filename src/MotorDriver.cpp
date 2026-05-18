#include "MotorDriver.h"
#include "Pins.h"

struct MotorPins {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
    uint8_t stby;
    uint8_t pwmChannel;
};

static MotorPins sixMotor = {
    sixPWM,
    sixIN1,
    sixIN2,
    sixSTBY,
    0
};

static MotorPins twelveMotor = {
    twelvePWM,
    twelveIN1,
    twelveIN2,
    twelveSTBY,
    1
};

const int pwmFreq = 20000;
const int pwmResBits = 8;

static int lastSixPWM = 0;
static int lastTwelvePWM = 0;

const int RAMP_STEP = 8;

static MotorPins* getMotor(MotorID motor) {
    if (motor == MOTOR_SIX) return &sixMotor;
    return &twelveMotor;
}

static int* getLastPWM(MotorID motor) {
    if (motor == MOTOR_SIX) return &lastSixPWM;
    return &lastTwelvePWM;
}

static int rampPWM(int desired, int current) {
    if (desired > current + RAMP_STEP) return current + RAMP_STEP;
    if (desired < current - RAMP_STEP) return current - RAMP_STEP;
    return desired;
}

void initMotors() {
    pinMode(sixIN1, OUTPUT);
    pinMode(sixIN2, OUTPUT);
    pinMode(sixSTBY, OUTPUT);

    pinMode(twelveIN1, OUTPUT);
    pinMode(twelveIN2, OUTPUT);
    pinMode(twelveSTBY, OUTPUT);

    digitalWrite(sixSTBY, HIGH);
    digitalWrite(twelveSTBY, HIGH);

    ledcSetup(sixMotor.pwmChannel, pwmFreq, pwmResBits);
    ledcAttachPin(sixPWM, sixMotor.pwmChannel);

    ledcSetup(twelveMotor.pwmChannel, pwmFreq, pwmResBits);
    ledcAttachPin(twelvePWM, twelveMotor.pwmChannel);

    stopAllMotors();
}

void setMotorPWM(MotorID motor, int pwmSigned) {
    MotorPins* m = getMotor(motor);
    int* lastPWM = getLastPWM(motor);

    pwmSigned = constrain(pwmSigned, -255, 255);

    int rampedPWM = rampPWM(pwmSigned, *lastPWM);
    *lastPWM = rampedPWM;

    int pwmMag = abs(rampedPWM);

    if (rampedPWM > 0) {
        digitalWrite(m->in1, HIGH);
        digitalWrite(m->in2, LOW);
    } else if (rampedPWM < 0) {
        digitalWrite(m->in1, LOW);
        digitalWrite(m->in2, HIGH);
    } else {
        digitalWrite(m->in1, LOW);
        digitalWrite(m->in2, LOW);
    }

    ledcWrite(m->pwmChannel, pwmMag);
}

void stopMotor(MotorID motor) {
    MotorPins* m = getMotor(motor);
    int* lastPWM = getLastPWM(motor);

    *lastPWM = 0;

    ledcWrite(m->pwmChannel, 0);
    digitalWrite(m->in1, LOW);
    digitalWrite(m->in2, LOW);
}

void stopAllMotors() {
    stopMotor(MOTOR_SIX);
    stopMotor(MOTOR_TWELVE);
}