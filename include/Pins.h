#pragma once

// 6V Tilt Motor
#define sixPWM 27
#define sixIN1 26
#define sixIN2 25
#define sixSTBY 14

// 12V Yaw Motor
#define twelvePWM 13
#define twelveIN1 12
#define twelveIN2 32
#define twelveSTBY 5

// 6V Encoder
#define sixENCA 34
#define sixENCB 35

// 12V Encoder
#define twelveENCA 36
#define twelveENCB 39

// Laser
#define laserswitch 33

// Encoder calibration
const float COUNTS_PER_REV_12V = 4250.0f;
const float COUNTS_PER_REV_6V  = 1809.0f * 4.0f;

const float COUNTS_PER_DEG_12V = COUNTS_PER_REV_12V / 360.0f;
const float COUNTS_PER_DEG_6V  = COUNTS_PER_REV_6V / 360.0f;

// rotational limits

const float YAW_MIN_DEG = -180.0f;
const float YAW_MAX_DEG =  180.0f;

const float TILT_MIN_DEG = 0.0f;
const float TILT_MAX_DEG = 270.0f;

//  Backlash constants

const long BACKLASH_12V_COUNTS = -7;
const long BACKLASH_6V_COUNTS  = 5;