#include "Encoder.h"
#include "driver/gpio.h"

// Encoder pins
#define twelveENCA 36
#define twelveENCB 39

volatile long positionCounts = 0;
volatile uint8_t prevAB = 0;

portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

static const int8_t TRANSITION[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

static inline uint8_t readAB_fast() {
    uint8_t a = gpio_get_level((gpio_num_t)twelveENCA);
    uint8_t b = gpio_get_level((gpio_num_t)twelveENCB);

    return (a << 1) | b;
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

long readCountsAtomic() {
    long p;

    portENTER_CRITICAL(&encMux);
    p = positionCounts;
    portEXIT_CRITICAL(&encMux);

    return p;
}

void writeCountsAtomic(long v) {
    portENTER_CRITICAL(&encMux);
    positionCounts = v;
    portEXIT_CRITICAL(&encMux);
}

void initEncoder() {
    pinMode(twelveENCA, INPUT);
    pinMode(twelveENCB, INPUT);

    prevAB = (uint8_t)((digitalRead(twelveENCA) << 1) | digitalRead(twelveENCB));
    writeCountsAtomic(0);
    attachInterrupt(digitalPinToInterrupt(twelveENCA), onEncoderChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(twelveENCB), onEncoderChange, CHANGE);
}