#include <Arduino.h>
#include <math.h>
class NotchFilter {
public:
  NotchFilter(float sampleRate, float notchFreq, float Q) {
    fs = sampleRate;
    f0 = notchFreq;
    this->Q = Q;
    setup();
  }

  float process(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2
                        - a1 * y1 - a2 * y2;

    // shift history
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;

    return y;
  }

private:
  float fs, f0, Q;
  float b0, b1, b2, a1, a2;

  float x1 = 0, x2 = 0;
  float y1 = 0, y2 = 0;

  void setup() {
    float w0 = 2.0 * PI * f0 / fs;
    float alpha = sin(w0) / (2.0 * Q);
    float cosw0 = cos(w0);

    float a0 = 1.0 + alpha;

    b0 = 1.0 / a0;
    b1 = -2.0 * cosw0 / a0;
    b2 = 1.0 / a0;
    a1 = -2.0 * cosw0 / a0;
    a2 = (1.0 - alpha) / a0;
  }
};