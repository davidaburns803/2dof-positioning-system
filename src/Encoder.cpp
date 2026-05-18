#include "Encoder.h"
#include "Pins.h"
#include "driver/gpio.h"

struct EncoderData {   
    volatile long positionCounts;
    volatile uint8_t prevAB;
    portMUX_TYPE mux;
    uint8_t pinA;
    uint8_t pinB;
};

static EncoderData sixEncoder = {
    0,
    0,
    portMUX_INITIALIZER_UNLOCKED,
    sixENCA,
    sixENCB
};

static EncoderData twelveEncoder = {
    0,
    0,
    portMUX_INITIALIZER_UNLOCKED,
    twelveENCA,
    twelveENCB
};

static const int8_t TRANSITION[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static inline uint8_t readAB_fast(uint8_t pinA, uint8_t pinB) {
    uint8_t a = gpio_get_level((gpio_num_t)pinA);
    uint8_t b = gpio_get_level((gpio_num_t)pinB);
    return (a << 1) | b;
}

static inline void updateEncoderISR(EncoderData* enc) {
    uint8_t curr = readAB_fast(enc->pinA, enc->pinB);
    uint8_t idx = (enc->prevAB << 2) | curr;
    int8_t delta = TRANSITION[idx];

    enc->prevAB = curr;

    if (delta != 0) {
        portENTER_CRITICAL_ISR(&enc->mux);
        enc->positionCounts += delta;
        portEXIT_CRITICAL_ISR(&enc->mux);
    }
}

void IRAM_ATTR onSixEncoderChange() {
    updateEncoderISR(&sixEncoder);
}

void IRAM_ATTR onTwelveEncoderChange() {
    updateEncoderISR(&twelveEncoder);
}

static long readCountsAtomic(EncoderData* enc) {
    long p;
    portENTER_CRITICAL(&enc->mux);
    p = enc->positionCounts;
    portEXIT_CRITICAL(&enc->mux);
    return p;
}

static void writeCountsAtomic(EncoderData* enc, long v) {
    portENTER_CRITICAL(&enc->mux);
    enc->positionCounts = v;
    portEXIT_CRITICAL(&enc->mux);
}

long readSixCountsAtomic() {
    return readCountsAtomic(&sixEncoder);
}

long readTwelveCountsAtomic() {
    return readCountsAtomic(&twelveEncoder);
}

void writeSixCountsAtomic(long v) {
    writeCountsAtomic(&sixEncoder, v);
}

void writeTwelveCountsAtomic(long v) {
    writeCountsAtomic(&twelveEncoder, v);
}

void initEncoders() {
    pinMode(sixENCA, INPUT);
    pinMode(sixENCB, INPUT);
    pinMode(twelveENCA, INPUT);
    pinMode(twelveENCB, INPUT);

    sixEncoder.prevAB = (uint8_t)((digitalRead(sixENCA) << 1) | digitalRead(sixENCB));
    twelveEncoder.prevAB = (uint8_t)((digitalRead(twelveENCA) << 1) | digitalRead(twelveENCB));

    writeSixCountsAtomic(0);
    writeTwelveCountsAtomic(0);

    attachInterrupt(digitalPinToInterrupt(sixENCA), onSixEncoderChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sixENCB), onSixEncoderChange, CHANGE);

    attachInterrupt(digitalPinToInterrupt(twelveENCA), onTwelveEncoderChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(twelveENCB), onTwelveEncoderChange, CHANGE);
}