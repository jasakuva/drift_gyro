class ReturnDamping {
  public:
    ReturnDamping(float up_increment, float down_increment, float return_increment, float input_derivative_trigger);
    float getDamping(float current_corr, float input_derivative);
    void init(float up_increment, float down_increment, float return_increment, float input_derivative_trigger);

  private:
    
    float v_last_correction;
    float v_activate_value;
    float v_up_increment;
    float v_down_increment;
    float v_return_increment;
    float v_input_derivative_trigger;
};