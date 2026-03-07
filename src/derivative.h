class derivative {
  public:
    derivative(float loopTime);
    float update(float data);
    float update(float data, unsigned long dt);

  private:
    float vLoopTime;
    float lastInput = 0.0f;
    bool firstRun = true;
};