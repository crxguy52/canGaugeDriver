#pragma once
#include "Arduino.h"
#include "inttypes.h"

float interp1f(float x, const float* xValues, const float* yValues, uint8_t numPoints);
bool is_due(uint16_t loopCount, uint16_t *count_end, uint16_t interval);


