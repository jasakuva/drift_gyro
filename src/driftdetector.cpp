#include "DriftDetector.h"
#include <Arduino.h>

DriftDetector::DriftDetector(float loopTime, float min_steering, float min_yaw)
{
  init(loopTime, min_steering, min_yaw);
}

float DriftDetector::update(float steering, float yaw)
{
  // avoid division by zero / nonsense loop time
  if (v_loopTime <= 0.0f) return drift_value;

  const bool drifting =
    ((steering >  v_min_steering && yaw < -v_min_yaw) ||
     (steering < -v_min_steering && yaw >  v_min_yaw));

  if(drifting) {
    drift_value = drift_value + (0.5f * v_loopTime);
  } else {
    drift_value = drift_value - (2.0f * v_loopTime);
  }
  
  drift_value = constrain(drift_value, 0.0f, 1.0f);

  last_steering = steering;
  last_yaw = yaw;

  return drift_value;
}

float DriftDetector::get()
{
  return drift_value;
}

void DriftDetector::init(float loopTime, float min_steering, float min_yaw)
{
  v_loopTime = loopTime;
  v_min_steering = min_steering;
  v_min_yaw = min_yaw;

  // optional: reset state when re-initializing
  // drift_value = 0.0f;
}