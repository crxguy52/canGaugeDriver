
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


