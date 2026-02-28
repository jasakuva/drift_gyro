#pragma once

class lpfilter {
public:
  lpfilter(float cutoffFrequency, float loopTime);
  void setCutoff(float cutoffFrequency);
  float update(float data);
  float get();

private:
  float p_loopTime;
  float k;
  float lastOutput;
};