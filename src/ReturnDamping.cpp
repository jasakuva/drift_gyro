#include "ReturnDamping.h"
#include <Arduino.h>
#include <math.h>

ReturnDamping::ReturnDamping(float up_increment, float down_increment, float return_increment, float input_derivative_trigger)
{
  init(up_increment, down_increment, return_increment, input_derivative_trigger);
}

void ReturnDamping::init(float up_increment, float down_increment, float return_increment, float input_derivative_trigger)
{
  v_up_increment=up_increment;
  v_down_increment=down_increment;
  v_return_increment=return_increment;
  v_input_derivative_trigger=input_derivative_trigger;
}

float ReturnDamping::getDamping(float current_corr, float input_derivative)
{
  if(fabsf(current_corr)-1 > fabsf(v_last_correction)) {
    v_activate_value = 0;
    v_triggered = true;
  }
  else
  {
    if(v_triggered) {
      v_triggered = false;
      v_activate_value = 1.0f;
    } else
    {
      v_activate_value = v_activate_value-v_down_increment;
    }
  }
  if(v_activate_value > 1.0f) {v_activate_value=1.0f; }
  if(v_activate_value < 0.0f) {v_activate_value=0.0f; }

  v_last_correction = current_corr;

  return v_activate_value;

}