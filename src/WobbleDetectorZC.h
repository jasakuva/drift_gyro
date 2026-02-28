#ifndef WOBBLE_DETECTOR_ZC_H
#define WOBBLE_DETECTOR_ZC_H

#include <Arduino.h>

class WobbleDetectorZC {
public:
  // Loop config
  static const int FS = 300;

  // 0.8 s window -> 240 samples
  static const int WIN = 240;

  // Tune for your system (deg/s error units)
  float minAmp = 1.5f;          // must be above this average |error|
  int   zcOnMin = 5;            // wobble ON if zc >= this within window
  int   zcOffMax = 3;           // wobble OFF if zc <= this within window
  int   holdOnWindows = 2;      // require N consecutive windows to assert wobble
  int   holdOffWindows = 2;     // require N consecutive windows to clear wobble

  bool wobbling = false;

  void reset() {
    i = 0;
    sumAbs = 0;
    zc = 0;
    prevSign = 0;
    onCount = 0;
    offCount = 0;
    wobbling = false;
  }

  // Call each loop with yawRateError (setpoint - measured)
  void update(float e) {
    int s = signWithDeadband(e, 0.1f); // ignore tiny sign flips near zero

    sumAbs += fabsf(e);

    if (prevSign != 0 && s != 0 && s != prevSign) {
      zc++;
    }
    if (s != 0) prevSign = s;

    i++;

    if (i >= WIN) {
      float amp = sumAbs / (float)WIN;

      bool candidateOn  = (amp >= minAmp) && (zc >= zcOnMin);
      bool candidateOff = (amp <  minAmp) || (zc <= zcOffMax);

      if (!wobbling) {
        onCount  = candidateOn ? (onCount + 1) : 0;
        if (onCount >= holdOnWindows) {
          wobbling = true;
          onCount = 0;
        }
      } else {
        offCount = candidateOff ? (offCount + 1) : 0;
        if (offCount >= holdOffWindows) {
          wobbling = false;
          offCount = 0;
        }
      }

      // reset window accumulators
      i = 0;
      sumAbs = 0;
      zc = 0;
      prevSign = 0;
    }
  }

  int lastZeroCrossings() const { return zc; }

private:
  int i = 0;
  float sumAbs = 0;
  int zc = 0;
  int prevSign = 0;
  int onCount = 0, offCount = 0;

  static int signWithDeadband(float x, float db) {
    if (x >  db) return  1;
    if (x < -db) return -1;
    return 0;
  }
};

#endif