#include "derivative.h"
#include <Arduino.h>

derivative::derivative(float loopTime) {
  vLoopTime = loopTime;
}

float derivative::update(float data) {

  if (firstRun) {
    lastInput = data;
    firstRun = false;
    return 0.0f;  // No derivative on first call
  }

  float derivativeValue = (data - lastInput) / vLoopTime;
  lastInput = data;

  return derivativeValue;
}

float derivative::update(float data, float dt) {
  if (firstRun) {
    lastInput = data;
    firstRun = false;
    return 0.0f;  // No derivative on first call
  }

  float derivativeValue = (data - lastInput) / dt;
  lastInput = data;

  return derivativeValue;

}