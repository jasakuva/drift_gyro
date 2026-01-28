class SteeringMap {
public:
  explicit SteeringMap(int16_t epa_low, int16_t epa_high, int16_t epa_center) {
    SetGyroReserve(10);
    setEpaValues(epa_low, epa_high, epa_center);
    
  }

  void setEpaValues(int16_t epa_low, int16_t epa_high, int16_t epa_center) {
    p_epa_low = epa_low;
    p_epa_high = epa_high;
    p_epa_center = epa_center;
  }

  void SetGyroReserve(int16_t gyro_reserve) {
    p_gyro_reserve = gyro_reserve;
  }

  float getNormalized(int16_t ms_value) {
    if (ms_value > p_epa_center) {
      return (float)(ms_value - p_epa_center) / (float)(p_epa_high - p_epa_center);
    } else {
      return 0 - (float)(p_epa_center - ms_value) / (float)(p_epa_center - p_epa_low);
    }
  }

  int16_t getServoMsValue(float normalized) {
    if (normalized >= 0.0f) {
      return p_epa_center + normalized * (p_steering_epa_high - p_epa_center);
    } else {
      return p_epa_center - normalized * (p_epa_center - p_steering_epa_low);
    }
  }

  
private:

  int16_t p_epa_low;
  int16_t p_epa_high;
  int16_t p_epa_center;
  int16_t p_gyro_reserve;
  int16_t p_steering_epa_low =  p_epa_center - (int16_t)((p_epa_center - p_epa_low) * (1.0f - p_gyro_reserve / 100.0f));
  int16_t p_steering_epa_high =  p_epa_center + (int16_t)((p_epa_high - p_epa_center) * (1.0f - p_gyro_reserve / 100.0f));
  
};