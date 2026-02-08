
#include <ESP32Servo.h>
#include "DerivativeN.h"
#include "SteeringMap.h"
#include <CodeCell.h>
#include "settings.h"
#include <Preferences.h>
#include <Arduino.h>

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define WINDOW_SIZE 5

float Roll;
float Pitch;
float Yaw;
volatile float yawRate_dps;

Preferences prefs_2;

CodeCell myCodeCell;

Servo steerServo;

DerivativeN dYawRate(20);
DerivativeN dSteering(10);

float gz_offset;

// ---- Pins ----
const uint8_t PIN_STEER_IN = 5;   // INT0
const uint8_t PIN_GAIN_IN  = 3;   // INT1 (optional)
const uint8_t PIN_SERVO_OUT = 2;

// ---- RC ranges ----
const int16_t RC_MIN = 1000;
const int16_t RC_MID = 1500;
const int16_t RC_MAX = 2000;

int epa_low_us_2   = 1100;
int epa_center_us_2 = 1500;
int epa_high_us_2  = 1900;

// Replace with your real measurement source (we’ll set it from s_pw snapshot)
//int current_steering_us = 1500;

// ---------- Settings variables (8 parameters) ----------
//int   p1;
//int   p2;
//int   p3;
//int   p4;
//float p5;
//float p6;
//float p7;
//int   p8;

SteeringMap mySteeringMap(1000,2000,1500);

// ---- ISR pulse capture ----
volatile uint32_t s_rise = 0, g_rise = 0;
volatile uint16_t s_pw = RC_MID, g_pw = RC_MID;

float buffer_gyro[WINDOW_SIZE] = {0};
float sum_gyro = 0.0f;
int bufIndex_gyro = 0;
int count_gyro = 0;



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

void isrGain() {
  if (digitalRead(PIN_GAIN_IN)) g_rise = micros();
  else {
    uint32_t w = micros() - g_rise;
    if (w >= 800 && w <= 2200) g_pw = (uint16_t)w;
  }
}

static inline int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Simple low-pass filter helper
static inline float lpf(float prev, float x, float alpha) {
  return prev + alpha * (x - prev);
}

static void loadSettings_2() {
  prefs_2.begin("rc", true);
  epa_low_us_2   = prefs_2.getInt("epaL", 1100);
  epa_center_us_2 = prefs_2.getInt("epaC", 1500);
  epa_high_us_2  = prefs_2.getInt("epaH", 1900);

  //p1 = prefs_2.getInt("p1", 10);
  //p2 = prefs_2.getInt("p2", 20);
  //p3 = prefs_2.getInt("p3", 30);
  //p4 = prefs_2.getInt("p4", 40);
  //p5 = prefs_2.getFloat("p5", 0.5f);
  //p6 = prefs_2.getFloat("p6", 1.0f);
  //p7 = prefs_2.getFloat("p7", 2.5f);
  //p8 = prefs_2.getInt("p8", 1);
  prefs_2.end();
}

void setup() {
  Serial.begin(115200);

  loadSettings_2();

  myCodeCell.Init(MOTION_GYRO);
  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);
  delay(1000);

  steerServo.attach(PIN_SERVO_OUT);
  
  steerServo.setTimerWidth(14);
  
  for (int us = 1700; us >= 1100; us -= 1) {
    steerServo.writeMicroseconds(us);
    delay(20);
  }

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  delay(2000);

  //for (int i = 0; i < 10000; i++) {
  //  steerServo.writeMicroseconds(s_pw);
  //  delay(5);
  //}

  if (s_pw < 1300 || s_pw > 1500)
  {
    makeSettings();
  };

  loadSettings_2();
  //if (calibrateYawAvg(myCodeCell, gz_offset)) {
  //gz_offset = gz_offset;   // store calibration
  //} else {
    Serial.println("Yaw calibration failed");
  //}
  
}

float moving_average_gyro(float new_value)
{
    // Remove the oldest value from sum
    sum_gyro -= buffer_gyro[bufIndex_gyro];

    // Store new value
    buffer_gyro[bufIndex_gyro] = new_value;
    sum_gyro += new_value;

    // Move index circularly
    bufIndex_gyro = (bufIndex_gyro + 1) % WINDOW_SIZE;

    // Count samples (for startup phase)
    if (count_gyro < WINDOW_SIZE) {
        count_gyro++;
    }

    // Return average
    return sum_gyro / count_gyro;   // use WINDOW_SIZE once fully filled
}

void loop() {
  // Read RC pulses atomically
  uint16_t steerIn, gainIn;
  //noInterrupts();
  steerIn = s_pw;
  //steerIn = 1500;
  gainIn  = g_pw;
  //interrupts();

  // Convert steer to signed
  int16_t steer = (int16_t)steerIn - RC_MID; // -500..+500

  float normSteering = mySteeringMap.getNormalized(steerIn);

  // Read gyro
  
  if (myCodeCell.Run(100)) {
    myCodeCell.Motion_GyroRead(Roll, Pitch, Yaw); 
  }

  float Result_gyro = moving_average_gyro(Yaw * 115.0f);

  yawRate_dps = Result_gyro;

  // Low-pass filter yaw rate to reduce twitch
  static float yawRateFilt = 0.0f;
  yawRateFilt = lpf(yawRateFilt, yawRate_dps, 0.2f); // alpha 0.1..0.3 typical

  dYawRate.add(yawRateFilt);
  dSteering.add(normSteering);

  float gain = ((float)gainIn - 1000.0f) / 1000.0f;  // 0..1
  gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;
  
  float gyro_correction = (yawRateFilt / 180.0f)*(gain/3) + (dYawRate.get() / 50.0f)*(gain);
  
  float corr = gyro_correction / (1.0f + abs(dSteering.get())/2.0f);
 
  int16_t out = (int16_t)mySteeringMap.getServoMsValue(normSteering + corr);
  out = clamp16(out, RC_MIN, RC_MAX);

  steerServo.writeMicroseconds(out);

  delay(6);


  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print("corr="); Serial.print(corr);
    Serial.print("Result_gyro="); Serial.print(Result_gyro);
    Serial.print("gyro_correction="); Serial.print(gyro_correction);
    Serial.print("yawRateFilt="); Serial.print(yawRateFilt);
    Serial.print("dYawRate.get()="); Serial.print(dYawRate.get());
    Serial.print("normSteering="); Serial.print(normSteering);
    Serial.print(" yaw(dps)="); Serial.print(yawRateFilt);
    Serial.print(" gain="); Serial.print(gain);
    Serial.print(" steerIn="); Serial.print(steerIn);
    Serial.print(" out="); Serial.println(out);
  }
}

  bool calibrateYawAvg(CodeCell& cc, float& avgOut) {
  constexpr int MAX_READS   = 100;
  constexpr int WINDOW_SIZE_2 = 10;
  constexpr int WAIT_MS     = 20;
  constexpr float STD_EPS   = 1e-6f;  // float tolerance for "0"

  float roll = 0, pitch = 0, yaw = 0;
  float buf[WINDOW_SIZE_2] = {0};

  int idx = 0;
  int filled = 0;

  for (int i = 0; i < MAX_READS; ++i) {
    cc.Motion_GyroRead(roll, pitch, yaw);

    buf[idx] = yaw;
    idx = (idx + 1) % WINDOW_SIZE;
    if (filled < WINDOW_SIZE_2) filled++;

    if (filled == WINDOW_SIZE_2) {
      // Compute average
      float sum = 0.0f;
      for (int j = 0; j < WINDOW_SIZE_2; ++j) sum += buf[j];
      float avg = sum / WINDOW_SIZE_2;

      // Compute std deviation
      float var = 0.0f;
      for (int j = 0; j < WINDOW_SIZE_2; ++j) {
        float d = buf[j] - avg;
        var += d * d;
      }
      var /= WINDOW_SIZE_2;
      float stddev = sqrtf(var);

      if (stddev <= STD_EPS) {
        avgOut = avg;
        return true;   // ✅ success
      }
    }

    delay(WAIT_MS);
  }

  return false;  // ❌ no stable window found
  }
