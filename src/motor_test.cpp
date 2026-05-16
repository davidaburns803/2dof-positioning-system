#include <Arduino.h>
#include "driver/gpio.h"

// Motor driver pins TB6612
#define sixPWM 27
#define twelvePWM 13
#define sixIN1 26
#define sixIN2 25
#define twelveIN1 12
#define twelveIN2 32
#define twelveSTBY 5
#define sixSTBY 14

// Encoder pins
#define sixENCA 34
#define sixENCB 35
#define twelveENCA 36
#define twelveENCB 39

// Laser Pointer Transistor Switch
#define laserswitch 33


// Control Parameters
const float Kp_12V = 0.3f ;   //try .2 first since 12V more powerful then 6V and .35 worked well
const float Kd_12V = 0.0f ;
const float Ki_12V = 0.0f ;
const float Kp_6V = .35f ;
const float Kd_6V = 0.0f ;
const float Ki_6V =  0.0f;

// Counts per Rev
const float COUNTS_PER_REV_12V = 4250.0f;
const float COUNTS_PER_REV_6V = 1809.0f * 4.0f;

const float COUNTS_PER_DEG_12V = COUNTS_PER_REV_12V / 360.0f;
const float COUNTS_PER_DEG_6V = COUNTS_PER_REV_6V / 360.0f;

