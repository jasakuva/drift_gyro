#include "lpfilter.h"

#include <Arduino.h>

lpfilter::lpfilter(float cutoffFrequency, float loopTime) {
  p_loopTime = loopTime;
  setCutoff(cutoffFrequency);
  //float o = 2.0f * PI * cutoffFrequency * loopTime;
  //k = o / (o + 1.0f);
}

void lpfilter::setCutoff(float cutoffFrequency) {
  if (cutoffFrequency == 0) {
    k = 1;
  } else {
    float o = 2.0f * PI * cutoffFrequency * p_loopTime;
    k = o / (o + 1.0f);
  }
}

float lpfilter::get() {
  return lastOutput;
}

float lpfilter::update(float data) {
  lastOutput = lastOutput + k * (data - lastOutput);
  return lastOutput;
}