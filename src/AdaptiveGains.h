struct AdaptiveGains {
  float Kp0;
  float Kp;

  // limits
  float kpMinMul = 0.55f;
  
  // rates
  float cutPerSecP = 1.2f;   // 120%/s toward min when wobbling (fast)
  float cutPerSecI = 1.0f;
  float recPerSec  = 0.25f;  // 25%/s back to base (slow)

  void init(float kp) {
    Kp0=kp;
    Kp=kp;
  }

  void setBaseGain(float bGain) {
    Kp0 = bGain;
  }

  float get() {
    return Kp;
  }

  void update(bool wobbling, float dt) {
    float kpMin = kpMinMul * Kp0;
    

    if (wobbling) {
      // move gains down toward minimum
      Kp += (kpMin - Kp) * (cutPerSecP * dt);
      
    } else {
      // move gains up toward base
      Kp += (Kp0 - Kp) * (recPerSec * dt);
      
    }
  }
};