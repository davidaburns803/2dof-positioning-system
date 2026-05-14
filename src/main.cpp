#include <Arduino.h>
#include "driver/gpio.h"

// Motor driver pins TB6612
#define PWMA 27
#define AIN1 26
#define AIN2 25
#define STBY 14

// Encoder pins
#define ENCA 34
#define ENCB 35

// ===================== ENCODER CALIBRATION =====================
// If 1809 was from your OLD method (A rising only), then FULL quadrature will be ~4x that.
const float COUNTS_PER_REV = 4281.0f;   // FULL quadrature
// const float COUNTS_PER_REV = 1809.0f;       // If you already measured using full quadrature
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0f;

// Control parameters
float Kp = 0.35f;
float Kd = 0.0f;
int pwmMax = 200;
int pwmMin = 35;
long allowableErrorCounts = 5;   // if you go 4x CPR, you may want ~20 here

// ===================== BACKLASH / DIRECTION OFFSET =====================
const long BACKLASH_OFFSET_COUNTS = -5; // start here, flip sign if needed

static inline long applyBacklashComp(long rawCounts, long errCounts) {
  // Apply offset when approaching target in negative direction (err < 0).
  // If your mismatch is on the other approach direction, change to (errCounts > 0).
  if (errCounts < 0) return rawCounts + BACKLASH_OFFSET_COUNTS;
  return rawCounts;
}

// PWM setup for ESP32 LEDC
const int pwmChannel = 0;
const int pwmFreq = 20000;
const int pwmResBits = 8;

// ===================== ROBUST QUADRATURE DECODER =====================
// We reject impossible transitions. Only one bit may change at a time.
volatile long positionCounts = 0;
volatile uint8_t prevAB = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

// index = (prev<<2) | curr
// Valid transitions produce +/-1, illegal jumps produce 0
static const int8_t TRANSITION[16] = {
  0, +1, -1,  0,   // prev 00 -> 00,01,10,11
 -1,  0,  0, +1,   // prev 01 -> 00,01,10,11
 +1,  0,  0, -1,   // prev 10 -> 00,01,10,11
  0, -1, +1,  0    // prev 11 -> 00,01,10,11
};

static inline uint8_t readAB_fast() {
  uint8_t a = gpio_get_level((gpio_num_t)ENCA);
  uint8_t b = gpio_get_level((gpio_num_t)ENCB);
  return (a << 1) | b; // 0..3
}

void IRAM_ATTR onEncoderChange() {
  uint8_t curr = readAB_fast();
  uint8_t idx = (prevAB << 2) | curr;
  int8_t delta = TRANSITION[idx];

  prevAB = curr;

  if (delta != 0) {
    portENTER_CRITICAL_ISR(&encMux);
    positionCounts += delta;
    portEXIT_CRITICAL_ISR(&encMux);
  }
}

static inline long readCountsAtomic() {
  long p;
  portENTER_CRITICAL(&encMux);
  p = positionCounts;
  portEXIT_CRITICAL(&encMux);
  return p;
}

static inline void writeCountsAtomic(long v) {
  portENTER_CRITICAL(&encMux);
  positionCounts = v;
  portEXIT_CRITICAL(&encMux);
}

// Forward declarations
void setMotor(int pwmSigned);
long angleDegToCounts(float angleDeg);
float countsToAngleDeg(long counts);

void setup() {
  Serial.begin(115200);

  // Motor driver setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // PWM setup using LEDC
  ledcSetup(pwmChannel, pwmFreq, pwmResBits);
  ledcAttachPin(PWMA, pwmChannel);
  ledcWrite(pwmChannel, 0);

  // Encoder pins
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  // Init AB state, then attach CHANGE interrupts on BOTH pins
  prevAB = (uint8_t)((digitalRead(ENCA) << 1) | digitalRead(ENCB));
  attachInterrupt(digitalPinToInterrupt(ENCA), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), onEncoderChange, CHANGE);

  Serial.println("Type a target angle in degrees, example 90 or -45, then press Enter.");
  Serial.println("Type 'zero' to reset counts.");
}

void loop() {
  static bool haveTarget = false;
  static long targetCounts = 0;

  // Read serial commands
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.length() > 0) {
      if (s.equalsIgnoreCase("zero")) {
        writeCountsAtomic(0);
        haveTarget = false;
        setMotor(0);
        Serial.println("Zeroed position.");
      } else {
        float targetAngleDeg = s.toFloat();
        targetCounts = angleDegToCounts(targetAngleDeg);
        haveTarget = true;

        Serial.print("Target angle deg ");
        Serial.print(targetAngleDeg, 2);
        Serial.print("  targetCounts ");
        Serial.println(targetCounts);
      }
    }
  }

  // Control loop at fixed rate
  static unsigned long lastT = 0;
  static long lastPosRaw = 0;

  unsigned long now = millis();
  unsigned long dt_ms = now - lastT;
  if (dt_ms < 10) return;   // 100 Hz
  lastT = now;

  // Read raw encoder counts once
  long p_raw = readCountsAtomic();

  // Velocity from RAW counts only (do not apply backlash offset to velocity)
  long deltaPos = p_raw - lastPosRaw;
  lastPosRaw = p_raw;

  float dt_s = dt_ms / 1000.0f;
  float velocity_counts_per_seconds = deltaPos / dt_s;

  if (!haveTarget) {
    setMotor(0);
    return;
  }

  long err_raw = targetCounts - p_raw;

  // Backlash compensation applied to position only
  long p_comp = applyBacklashComp(p_raw, err_raw);
  long err = targetCounts - p_comp;

  float angleNow = countsToAngleDeg(p_comp);

  float vel = velocity_counts_per_seconds;
  static float velFilt = 0.0f;
  velFilt = 0.8f * velFilt + 0.2f * vel;

  // Stop when close enough
  if (labs(err) <= allowableErrorCounts) {
    setMotor(0);
    Serial.print("At target. angle deg ");
    Serial.print(angleNow, 2);
    Serial.print("  raw ");
    Serial.print(p_raw);
    Serial.print("  comp ");
    Serial.println(p_comp);
    return;
  }

  // PD control
  float u = (Kp * (float)err) - (Kd * velFilt);
  int pwm = (int)lroundf(u);

  // Clamp and enforce minimum effort
  if (pwm > pwmMax) pwm = pwmMax;
  if (pwm < -pwmMax) pwm = -pwmMax;

  if (pwm > 0 && pwm < pwmMin) pwm = pwmMin;
  if (pwm < 0 && pwm > -pwmMin) pwm = -pwmMin;

  setMotor(pwm);

  
}

long angleDegToCounts(float angleDeg) {
  return (long)lroundf(angleDeg * COUNTS_PER_DEG);
}

float countsToAngleDeg(long counts) {
  return (float)counts / COUNTS_PER_DEG;
}

void setMotor(int pwmSigned) {
  int pwm = abs(pwmSigned);
  pwm = constrain(pwm, 0, 255);

  if (pwmSigned > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmSigned < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  ledcWrite(pwmChannel, pwm);
}