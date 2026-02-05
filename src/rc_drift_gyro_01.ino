
#include <ESP32Servo.h>
#include "DerivativeN.h"
#include "SteeringMap.h"
#include <CodeCell.h>
#include "settings.h"

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define WINDOW_SIZE 40

float Roll;
float Pitch;
float Yaw;
volatile float yawRate_dps;

CodeCell myCodeCell;

Servo steerServo;

DerivativeN dYawRate(40);
DerivativeN dSteering(20);

float gz_offset;

// ---- Pins ----
const uint8_t PIN_STEER_IN = 1;   // INT0
const uint8_t PIN_GAIN_IN  = 3;   // INT1 (optional)
const uint8_t PIN_SERVO_OUT = 2;

// ---- RC ranges ----
const int16_t RC_MIN = 1000;
const int16_t RC_MID = 1500;
const int16_t RC_MAX = 2000;

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

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  steerServo.attach(PIN_SERVO_OUT);
  steerServo.writeMicroseconds(RC_MID);

  //myCodeCell.Init(MOTION_GYRO | MOTION_ACCELEROMETER);
  myCodeCell.Init(MOTION_GYRO);

  // Let gyro settle, then calibrate offset (keep car still!)
  delay(1000);

  // Quick-and-dirty gyro Z offset calibration
  float sum = 0;
  const int N = 20;
  float Roll;
  float Pitch;
  float Yaw;
  for (int i = 0; i < N; i++) {
    
    myCodeCell.Motion_GyroRead(Roll, Pitch, Yaw);
    sum += Yaw;
    Serial.print("Yaw="); Serial.print(Yaw);
    delay(100);
  }
  gz_offset = (float)sum / (float)N;

  delay(5000);

  // Store in a global via lambda capture trick? simpler: print and manually set
  // We'll keep it as a static in loop using a global.

  Serial.print("Calibrated gz_offset = ");
  Serial.println(gz_offset);

  makeSettings();
  
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
  noInterrupts();
  steerIn = s_pw;
  //steerIn = 1500;
  gainIn  = g_pw;
  interrupts();

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

  // Gain from CH2 knob: map 1000..2000 to 0.0..1.5 (tune)
  float gain = ((float)gainIn - 1000.0f) / 1000.0f;  // 0..1
  gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;
  
  float gyro_correction = (yawRateFilt / 180.0f)*(gain) + (dYawRate.get() / 10.0f)*(gain);
  
  float corr = gyro_correction / (1.0f + abs(dSteering.get())/2.0f);
 
  

  // Optional: deadband to avoid hunting when nearly straight
  if (abs(steer) < 20) steer = 0; // ~20us deadband

  int16_t out = (int16_t)mySteeringMap.getServoMsValue(normSteering + corr);
  out = clamp16(out, RC_MIN, RC_MAX);

  steerServo.writeMicroseconds(out);

  delay(10);


  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
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