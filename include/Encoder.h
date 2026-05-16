#pragma once

#include <Arduino.h>

void initEncoder();
long readCountsAtomic();
void writeCountsAtomic(long v);