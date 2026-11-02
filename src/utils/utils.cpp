
#include "utils.h"

float interp1f(float x, const float* xValues, const float* yValues, uint8_t numPoints) {
  // Clamp to table bounds
  if (x <= xValues[0]) return yValues[0];
  if (x >= xValues[numPoints - 1]) return yValues[numPoints - 1];

  // Find the bracketing segment (linear search — fine for small tables;
  // use binary search below if numPoints is large)
  uint8_t i = 0;
  while (x > xValues[i + 1]) i++;

  // Linear interpolation between the two bracketing points
  float x0 = xValues[i], x1 = xValues[i + 1];
  float y0 = yValues[i], y1 = yValues[i + 1];

  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}


bool is_due(uint16_t loopCount, uint16_t *count_end, uint16_t interval) {
  // True once loopCount reaches *count_end, then schedules the next event one
  // interval later. The unsigned subtraction handles loopCount rollover.
  if ((uint16_t)(loopCount - *count_end) < interval) {
    *count_end += interval;
    return true;
  }
  return false;
}
