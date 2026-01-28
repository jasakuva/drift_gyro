#include <Arduino.h>
#include <math.h>

// ---- Settings ----
static const uint32_t SAMPLE_PERIOD_MS = 100;   // 10 Hz
static const uint32_t TIMEOUT_MS       = 20000; // 20 seconds
static const uint8_t  WINDOW_N         = 20;    // 20 measurements
static const float    STDEV_THRESH     = 0.5f;  // <-- tune: gyro units (e.g. deg/s)

// ---- State ----
float samples[WINDOW_N];
uint8_t count = 0;
uint8_t idx = 0;

bool finished = false;
float recordedMean = 0.0f;

// ---- Replace this with your real gyro reading ----
// Return gyro rate (e.g., deg/s) or raw units—just be consistent with STDEV_THRESH.
float readGyroZ() {
  // Example stub:
  // return mpu.getGyroZ_dps();
  return 0.0f;
}

void computeMeanStd(const float* x, uint8_t n, float &mean, float &stdev) {
  // Mean
  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) sum += x[i];
  mean = sum / (float)n;

  // Variance (sample variance)
  float var = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    float d = x[i] - mean;
    var += d * d;
  }
  var /= (float)(n - 1);      // sample variance
  stdev = sqrtf(var);
}

float getAndDoCalibration(&myCodeCell) {
  static uint32_t startMs = millis();
  static uint32_t lastSampleMs = 0;

  if (finished) {
    // Calibration done; keep running or halt as you prefer
    // while(true) { delay(1000); }
    return recordedMean;
  }

  uint32_t now = millis();

  // Timeout after ~20 seconds
  if (now - startMs >= TIMEOUT_MS) {
    Serial.println("Timeout: did not reach required stability.");
    finished = true;
    return -999.0f;
  }

  // Sample at 10 Hz
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;

  float g = readGyroZ();

  // Store in ring buffer
  samples[idx] = g;
  idx = (idx + 1) % WINDOW_N;
  if (count < WINDOW_N) count++;

  // Wait until we have 20 samples
  if (count < WINDOW_N) {
    Serial.print("Filling buffer: ");
    Serial.print(count);
    Serial.print("/");
    Serial.println(WINDOW_N);
    return;
  }

  // Compute mean/stdev of the current window
  float mean, stdev;
  computeMeanStd(samples, WINDOW_N, mean, stdev);

  Serial.print("mean=");
  Serial.print(mean, 6);
  Serial.print("  stdev=");
  Serial.println(stdev, 6);

  // Check stability condition
  if (stdev <= STDEV_THRESH) {
    recordedMean = mean;
    finished = true;

    Serial.println("Stable! Recording average.");
    Serial.print("Recorded gyro mean = ");
    Serial.println(recordedMean, 6);

    // You can store recordedMean as your gyro offset/bias here.
  }
}