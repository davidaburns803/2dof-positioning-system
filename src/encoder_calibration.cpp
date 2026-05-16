#include <Arduino.h>
#include "driver/gpio.h"

// Motor driver pins TB6612
#define twelvePWM 13
#define twelveIN1 12
#define twelveIN2 32 //32
#define twelveSTBY 5

// Encoder pins
#define twelveENCA 36  //36
#define twelveENCB 39

const float COUNTS_PER_REV_12V = 4250.0f;
const float Counts_per_degree_12V = COUNTS_PER_REV_12V/ 360.0f;
const long targetcounts = (long)(1.0f * COUNTS_PER_REV_12V);


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
  uint8_t a = gpio_get_level((gpio_num_t)twelveENCA);
  uint8_t b = gpio_get_level((gpio_num_t)twelveENCB);
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


void setup () {
pinMode(twelveIN1, OUTPUT);
pinMode(twelveIN2, OUTPUT);
pinMode(twelveSTBY, OUTPUT);
pinMode(twelvePWM, OUTPUT);
digitalWrite(twelveSTBY, HIGH);

pinMode(twelveENCA, INPUT);
pinMode(twelveENCB, INPUT);

delay(1000);

// Init AB state, then attach CHANGE interrupts on BOTH pins
  prevAB = (uint8_t)((digitalRead(twelveENCA) << 1) | digitalRead(twelveENCB));
  writeCountsAtomic(0);
  attachInterrupt(digitalPinToInterrupt(twelveENCA), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(twelveENCB), onEncoderChange, CHANGE);
}

void loop() {
    
    long p_loop = readCountsAtomic();

    digitalWrite(twelveIN1, LOW);
    digitalWrite(twelveIN2, HIGH);

    while(p_loop > -targetcounts) {
        digitalWrite(twelvePWM, HIGH);
        p_loop = readCountsAtomic();

    }
    
    digitalWrite(twelvePWM, LOW);
}