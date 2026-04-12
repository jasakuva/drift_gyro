#include "ReturnDamping.h"
#include <Arduino.h>

ReturnDamping::ReturnDamping(float up_increment, float down_increment, float return_increment, float input_derivative_trigger)
{
  init(float up_increment, float down_increment, float return_increment, float input_derivative_trigger);
}

ReturnDampin::init(float up_increment, float down_increment, float return_increment, float input_derivative_trigger)
{
  v_up_increment=up_increment;
  v_down_increment=down_increment;
  v_return_increment=return_increment;
  v_input_derivative_trigger=input_derivative_trigger;
}

ReturnDamping::getDamping(float current_corr, float input_derivative)
{
  
}