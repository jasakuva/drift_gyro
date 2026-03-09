#include <ESP32Servo.h>
#include "DerivativeN.h"
#include "SteeringMap.h"
#include "settings.h"
#include <Preferences.h>
#include <Arduino.h>
#include "myMovingAverage.h"
#include "lpfilter.h"
#include "derivative.h"
#include "medianfilter.h"
#include "ControlParams.h"
#include "WobbleDetectorZC.h"
#include "AdaptiveGains.h"
#include "DriftDetector.h"

WobbleDetectorZC wob;
AdaptiveGains ag;

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN D4
#define SCL_PIN D5

// ---- Pins ----
const uint8_t PIN_STEER_IN  = D0;
const uint8_t PIN_GAIN_IN   = D1;
const uint8_t PIN_SERVO_OUT = D3;

const unsigned long LOOP_PERIOD_US = 5000;   // 5 ms = 200 Hz
const float LOOP_PERIOD_MS = LOOP_PERIOD_US / 1000.0f;
const float LOOP_PERIOD_S  = LOOP_PERIOD_MS / 1000.0f;
unsigned long nextLoopTime;

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define BUFFER_SIZE 30

ControlParams cp;

float Roll;
float Pitch;
float Yaw;
float PrewYaw;
volatile float yawRate_dps;
float corr;
float gyrodps;
float gyro_correction;
float drift_value;
float filtered_yaw_derivative;

float lastGyroCorrection = 0.0f;

Preferences prefs_2;

Adafruit_MPU6050 mpu;

Servo steerServo;
lpfilter gyro_lp(20.0, LOOP_PERIOD_S);
lpfilter derivative_lp(20.0, LOOP_PERIOD_S);
lpfilter servoin_lp(20.0, LOOP_PERIOD_S);
lpfilter servoout_lp(20.0, LOOP_PERIOD_S);
lpfilter corr_return_lp(5.0, LOOP_PERIOD_S);
lpfilter correction_long_lp(0.2, LOOP_PERIOD_S);

derivative dYawRate(LOOP_PERIOD_S);
derivative dSteering(LOOP_PERIOD_S);

myMovingAverage<10> drift_value_avg;

DriftDetector driftd(LOOP_PERIOD_S, 0.05f, 10.0f);



// ---- RC ranges ----
const int16_t RC_MIN = 1000;
const int16_t RC_MID = 1500;
const int16_t RC_MAX = 2000;

int epa_low_us_2    = 1100;
int epa_center_us_2 = 1500;
int epa_high_us_2   = 1900;

int to_settings_counter;

SteeringMap mySteeringMap(1000, 2000, 1500);

// ---- ISR pulse capture ----
volatile uint32_t s_rise = 0, g_rise = 0;
volatile uint16_t s_pw   = 1700, g_pw = RC_MID;

float buffer_gyro[BUFFER_SIZE] = {0};
float sum_gyro = 0.0f;
int bufIndex_gyro = 0;
int count_gyro = 0;
float yawRateFilt;

float off_drift_damper;

// Gyro bias
float gyroZOffset_dps = 0.0f;

void IRAM_ATTR isrSteer() {
  if (digitalRead(PIN_STEER_IN)) {
    portENTER_CRITICAL_ISR(&s_mux);
    s_rise = micros();
    portEXIT_CRITICAL_ISR(&s_mux);
  } else {
    uint32_t w = micros() - s_rise;
    if (w >= 800 && w <= 2200) {
      portENTER_CRITICAL_ISR(&s_mux);
      s_pw = (uint16_t)w;
      portEXIT_CRITICAL_ISR(&s_mux);
    }
  }
}

void IRAM_ATTR isrGain() {
  if (digitalRead(PIN_GAIN_IN)) {
    portENTER_CRITICAL_ISR(&s_mux);
    g_rise = micros();
    portEXIT_CRITICAL_ISR(&s_mux);
  } else {
    uint32_t w = micros() - g_rise;
    if (w >= 800 && w <= 2200) {
      portENTER_CRITICAL_ISR(&s_mux);
      g_pw = (uint16_t)w;
      portEXIT_CRITICAL_ISR(&s_mux);
    }
  }
}

static inline int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float lpf(float prev, float x, float alpha) {
  return prev + alpha * (x - prev);
}

static void loadSettings_2() {
  delay(50);
  prefs_2.begin("rc", true);
  epa_low_us_2    = prefs_2.getInt("epaL", 1100);
  epa_center_us_2 = prefs_2.getInt("epaC", 1500);
  epa_high_us_2   = prefs_2.getInt("epaH", 1900);

  cp.gain             = prefs_2.getFloat("gain_main", 1.0);
  cp.gyro_avg         = prefs_2.getInt("gyro_avg", 6);
  cp.steering_prio    = prefs_2.getFloat("s_p", 1.0);
  cp.correction_exp   = prefs_2.getFloat("g_exp", 1.0);
  cp.gyro_lp_hz       = prefs_2.getInt("gyro_lp_hz", 20);
  cp.derivative_lp_hz = prefs_2.getInt("d_lp_hz", 20);
  cp.steer_in_lp_hz   = prefs_2.getInt("s_in_lp", 20);
  cp.steer_out_lp_hz  = prefs_2.getInt("s_out_lp", 20);
  cp.pid_p            = prefs_2.getFloat("pid_p", 5.0);
  cp.pid_d            = prefs_2.getFloat("pid_d", 1.0);
  cp.wobble_det_a     = prefs_2.getFloat("wobble_det_a", 0.2);
  cp.max_d_corr       = prefs_2.getFloat("max_d_corr", 0.05);

  gyro_lp.setCutoff(cp.gyro_lp_hz);
  derivative_lp.setCutoff(cp.derivative_lp_hz);
  servoin_lp.setCutoff(cp.steer_in_lp_hz);
  servoout_lp.setCutoff(cp.steer_out_lp_hz);

  wob.setAmplitude(cp.wobble_det_a);
  driftd.init(LOOP_PERIOD_S, cp.dd_min_steer, cp.dd_min_yaw);

  prefs_2.end();
}

static void calibrateGyroOffset() {
  const int samples = 500;
  float sum = 0.0f;

  sensors_event_t a, g, t;

  Serial.println("Calibrating MPU6050 gyro offset... keep car still");
  delay(500);

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z * 57.2957795f;  // rad/s -> deg/s
    delay(2);
  }

  gyroZOffset_dps = sum / samples;
  Serial.print("gyroZOffset_dps=");
  Serial.println(gyroZOffset_dps);
}

void setup() {
  Serial.begin(115200);

  loadSettings_2();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 found");

  // Good starting point for RC drift gyro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyroOffset();

  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);
  delay(500);

  steerServo.attach(PIN_SERVO_OUT);
  steerServo.setTimerWidth(14);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  delay(200);

  if (s_pw < 1300 || s_pw > 1700) {
    makeSettings();
  }

  loadSettings_2();

  delay(500);
  nextLoopTime = micros();
}

float moving_average_gyro(float new_value, int avg_size) {
  buffer_gyro[bufIndex_gyro] = new_value;
  bufIndex_gyro = (bufIndex_gyro + 1) % BUFFER_SIZE;

  if (count_gyro < BUFFER_SIZE) {
    count_gyro++;
  }

  float sum = 0.0f;
  int samples = (count_gyro < avg_size) ? count_gyro : avg_size;

  for (int i = 0; i < samples; i++) {
    int idx = (bufIndex_gyro - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
    sum += buffer_gyro[idx];
  }

  return sum / samples;
}

void loop() {
  // Read RC pulses atomically
  uint16_t steerIn, gainIn;
  portENTER_CRITICAL(&s_mux);
  steerIn = s_pw;
  gainIn  = g_pw;
  portEXIT_CRITICAL(&s_mux);

  float gain = ((float)gainIn - 1000.0f) / 1000.0f;
  gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;

  steerIn = servoin_lp.update(steerIn);
  float normSteering = mySteeringMap.getNormalized(steerIn);

  // Read latest MPU6050 data
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Z gyro in deg/s
  gyrodps = (g.gyro.z * 57.2957795f) - gyroZOffset_dps;
  Yaw = gyrodps;

  // Go to settings mode if steer near EPA and no yaw rate
  if ((steerIn < epa_low_us_2 + 50 || steerIn > epa_high_us_2 - 50) && abs(gyrodps) < 10) {
    to_settings_counter++;
    if (to_settings_counter > 3.0f * (1.0f / LOOP_PERIOD_S)) {
      makeSettings();
      loadSettings_2();
      to_settings_counter = 0;
    }
  } else {
    to_settings_counter = 0;
  }

  yawRateFilt = gyro_lp.update(gyrodps);
  filtered_yaw_derivative = derivative_lp.update(dYawRate.update(yawRateFilt));

  // PID
  gyro_correction = cp.gain * gain *
                    (cp.pid_p * yawRateFilt + cp.pid_d * filtered_yaw_derivative) / 150.0f;

  corr = gyro_correction /
         (1.0f + (abs(dSteering.update(normSteering)) / 2.0f) * cp.steering_prio);

  if (corr != 0.0f) {
    corr = (corr > 0.0f ? 1.0f : -1.0f) * powf(fabs(corr), cp.correction_exp);
  } else {
    corr = 0.0f;
  }

  corr = corr_return_lp.update(corr);
  correction_long_lp.update(corr);

  float delta = corr - lastGyroCorrection;
  if (fabs(delta) > cp.max_d_corr) {
    corr = lastGyroCorrection + (delta > 0 ? cp.max_d_corr : -cp.max_d_corr);
  }

  lastGyroCorrection = corr;

  if (cp.gain > 0) {
    drift_value = driftd.update(normSteering, 0 - yawRateFilt);
  } else {
    drift_value = driftd.update(normSteering, yawRateFilt);
  }

  int16_t out = (int16_t)mySteeringMap.getServoMsValue(normSteering + corr);
  out = clamp16(out, epa_low_us_2, epa_high_us_2);

  out = servoout_lp.update(out);
  steerServo.writeMicroseconds(out);

  long loopTime = micros() - nextLoopTime;

  while ((long)(micros() - nextLoopTime) < 0) {
    yield();
  }
  nextLoopTime += LOOP_PERIOD_US;

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200 && cp.debug_serial == 1) {
    lastPrint = millis();
    Serial.print("corr="); Serial.print(corr);
    Serial.print(" Result_gyro="); Serial.print(gyrodps);
    Serial.print(" gyro_correction="); Serial.print(gyro_correction);
    Serial.print(" yawRateFilt="); Serial.print(yawRateFilt);
    Serial.print(" normSteering="); Serial.print(normSteering);
    Serial.print(" gain="); Serial.print(gain);
    Serial.print(" steerIn="); Serial.print(steerIn);
    Serial.print(" out="); Serial.print(out);
    Serial.print(" filtered_yaw_derivative="); Serial.print(filtered_yaw_derivative);
    Serial.print(" loopTime="); Serial.println(loopTime);
  }
}
