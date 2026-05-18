#pragma once

#include <Arduino.h>

void initEncoders();

long readSixCountsAtomic();
long readTwelveCountsAtomic();

void writeSixCountsAtomic(long v);
void writeTwelveCountsAtomic(long v);