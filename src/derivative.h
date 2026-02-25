class derivative {
  public:
    derivative(float loopTime);
    float update(float data);

  private:
    float vLoopTime;
    float lastInput = 0.0f;
    bool firstRun = true;
};