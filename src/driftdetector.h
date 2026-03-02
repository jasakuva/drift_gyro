class DriftDetector {
  public:
    DriftDetector(float loopTime, float min_steering, float min_yaw);
    float update(float steering, float yaw);
    float get();
    void init(float loopTime, float min_steering, float min_yaw);

  private:
    
    float v_loopTime;
    float v_min_steering;
    float v_min_yaw;
    float last_steering;
    float last_yaw;
    float drift_value;
};