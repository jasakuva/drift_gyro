#include "DriftDetector.h"
#include <Arduino.h>

DriftDetector::DriftDetector(float loopTime, float min_steering, float min_yaw)
{
  init(loopTime, min_steering, min_yaw);
}

float DriftDetector::update(float steering, float yaw, float gain)
{
  // avoid division by zero / nonsense loop time
  if (v_loopTime <= 0.0f) return drift_value;

  //if (cp.gain > 0) {
    //  drift_value = driftd.update(normSteering, 0 - yawRateFilt);
    //} else {
    //  drift_value = driftd.update(normSteering, yawRateFilt);
    //}
  bool drifting;
  if (gain < 0) {
    drifting =
    ((steering >  v_min_steering && yaw < -v_min_yaw) ||
     (steering < -v_min_steering && yaw >  v_min_yaw));
  } else {
    drifting =
    ((steering >  v_min_steering && 0-yaw < -v_min_yaw) ||
     (steering < -v_min_steering && 0-yaw >  v_min_yaw));
  }

  if(drifting) {
    drift_value = drift_value + (1.0f * v_loopTime);
  } else {
    drift_value = drift_value - (4.0f * v_loopTime);
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