
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

WobbleDetectorZC wob;
AdaptiveGains ag;

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 8
#define SCL_PIN 9

const unsigned long LOOP_PERIOD_US = 2500;  // 2.5 ms
const float LOOP_PERIOD_MS = LOOP_PERIOD_US / 1000.0f;
const float LOOP_PERIOD_S = LOOP_PERIOD_MS / 1000.0f;
unsigned long nextLoopTime;

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define BUFFER_SIZE 30

ControlParams cp;

float Roll;
float Pitch;
float Yaw;
float PrewYaw;
volatile float yawRate_dps;

float lastGyroCorrection;

Preferences prefs_2;

Adafruit_BNO08x bno08x;

Servo steerServo;
lpfilter gyro_lp(20.0, LOOP_PERIOD_S);
lpfilter derivative_lp(20.0, LOOP_PERIOD_S);
lpfilter servoin_lp(20.0, LOOP_PERIOD_S);
lpfilter servoout_lp(20.0, LOOP_PERIOD_S);
lpfilter corr_return_lp(5.0, LOOP_PERIOD_S);
lpfilter correction_long_lp(0.2, LOOP_PERIOD_S);

derivative dYawRate(LOOP_PERIOD_S);
derivative dSteering(LOOP_PERIOD_S);

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


// Parameters are stored in ControlParams cp

int to_settings_counter;

SteeringMap mySteeringMap(1000,2000,1500);

// ---- ISR pulse capture ----
volatile uint32_t s_rise = 0, g_rise = 0;
volatile uint16_t s_pw = RC_MID, g_pw = RC_MID;

float buffer_gyro[BUFFER_SIZE] = {0};
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

//void isrGain() {
//  if (digitalRead(PIN_GAIN_IN)) g_rise = micros();
//  else {
//    uint32_t w = micros() - g_rise;
//    if (w >= 800 && w <= 2200) g_pw = (uint16_t)w;
//  }
//}

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

  cp.gain = prefs_2.getFloat("gain_main", 1.0);
  cp.gyro_avg  = prefs_2.getInt("gyro_avg", 6);
  cp.steering_prio = prefs_2.getFloat("s_p", 1.0);
  cp.correction_exp = prefs_2.getFloat("g_exp", 1.0);
  cp.gyro_lp_hz = prefs_2.getInt("gyro_lp_hz", 20);
  cp.derivative_lp_hz = prefs_2.getInt("d_lp_hz", 20);
  cp.steer_in_lp_hz = prefs_2.getInt("s_in_lp", 20);
  cp.steer_out_lp_hz = prefs_2.getInt("s_out_lp", 20);
  cp.pid_p = prefs_2.getFloat("pid_p", 5.0);
  cp.pid_d = prefs_2.getFloat("pid_d", 1.0);

  // Apply filter cutoffs
  gyro_lp.setCutoff(cp.gyro_lp_hz);
  derivative_lp.setCutoff(cp.derivative_lp_hz);
  servoin_lp.setCutoff(cp.steer_in_lp_hz);
  servoout_lp.setCutoff(cp.steer_out_lp_hz);

  prefs_2.end();
}

void setup() {
  Serial.begin(115200);

  loadSettings_2();

  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // 400kHz I2C
  delay(50);
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not found");
    while (1);
  }
  Serial.println("BNO085 found");
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2000);

  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);
  delay(500);

  steerServo.attach(PIN_SERVO_OUT);
  
  steerServo.setTimerWidth(14);
  
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  delay(200);

  //makeSettings();

  if (s_pw < 1300 || s_pw > 1700)
  {
    makeSettings();
  };

  loadSettings_2();

  delay(500);

  nextLoopTime = micros();
  
}

float moving_average_gyro(float new_value, int avg_size)
{
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
  
  long startTime = micros();
  
  // Read RC pulses atomically
  uint16_t steerIn, gainIn;
  portENTER_CRITICAL(&s_mux);
  steerIn = s_pw;
  gainIn  = g_pw;
  portEXIT_CRITICAL(&s_mux);
  
  float gain = ((float)gainIn - 1000.0f) / 1000.0f;  // 0..1
  gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;

  //detect wobbling and reduce gain if wobbling
  ag.setBaseGain(gain);
  ag.update(wob.wobbling, LOOP_PERIOD_S);
  gain = ag.get();
  
  steerIn = servoin_lp.update(steerIn);

  // Convert steer to signed and normalized to -1...0...+1
  //int16_t steer = (int16_t)steerIn - RC_MID; 
  float normSteering = mySteeringMap.getNormalized(steerIn);

  sh2_SensorValue_t sensorValue;

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      Yaw = sensorValue.un.gyroscope.z;
    }
  }

  long loopTime = micros()-nextLoopTime;
  // Wait until next 5ms boundary
  while ((long)(micros() - nextLoopTime) < 0) {
    yield();
  }
  nextLoopTime += LOOP_PERIOD_US;

  float Result_gyro = moving_average_gyro(Yaw * 57.2958f, cp.gyro_avg);

  float gyrodps = Yaw * 57.2958f;

  // go to settings mode if steerin near epa and no yaw rate
  if ((steerIn < epa_low_us_2+50 || steerIn > epa_high_us_2-50) && abs(Result_gyro) < 10) {
    to_settings_counter++;
    if (to_settings_counter > 3*400) {
      makeSettings();
      loadSettings_2();
      to_settings_counter = 0;
    }
  } else {
    to_settings_counter = 0;
  }

  float yawRateFilt = gyro_lp.update(Result_gyro);
  
  float filtered_yaw_derivative = derivative_lp.update(dYawRate.update(yawRateFilt));
 
  float gyro_correction = cp.gain * gain * (cp.pid_p*yawRateFilt + cp.pid_d*filtered_yaw_derivative) / 150.0f;

  float corr = gyro_correction / (1.0f + (abs(dSteering.update(normSteering))/2.0f)*cp.steering_prio);
  
  //float corr_return = corr_return_lp.update(corr);
  
  //if(fabs(corr) < fabs(lastGyroCorrection)) {
  //  corr = corr_return;
  //}
  
  if (corr != 0.0f) {
    corr = (corr > 0.0f ? 1.0f : -1.0f) * powf(fabs(corr), cp.correction_exp);
  } else {
      corr = 0.0f; // avoid powf(0, <=0)
  }

  corr = corr_return_lp.update(corr);
  correction_long_lp.update(corr);

  wob.update(corr-correction_long_lp.get());

  // slow down correction in near center
  //float combined = normSteering + corr;
  //if (fabs(combined) < 0.1f && fabs(norm_output_avg.get()) < 0.05 ) {
  //  corr = (combined)*10.0f*corr;
  //}
  //float scale = fabs(combined) / 0.1f;  // linear from 0 to 1
  //if (scale > 1.0f ) scale = 1.0f;       // clamp to max 1
  //corr *= scale;

  lastGyroCorrection = corr;
  
 
  //dOutput_long.add(normSteering + corr);
  //norm_output_avg.update(normSteering + corr);
 
  int16_t out = (int16_t)mySteeringMap.getServoMsValue(normSteering + corr);
  out = clamp16(out, epa_low_us_2, epa_high_us_2);

  out = servoout_lp.update(out);
  steerServo.writeMicroseconds(out);
 
  //delay(2);

  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200 && cp.debug_serial == 1) {
  //if (millis() - lastPrint > 200) {  
    lastPrint = millis();
    Serial.print("corr="); Serial.print(corr);
    Serial.print("Result_gyro="); Serial.print(gyrodps);
    Serial.print("gyro_correction="); Serial.print(gyro_correction);
    Serial.print("yawRateFilt="); Serial.print(yawRateFilt);
    //Serial.print("dYawRate.get()="); Serial.print(dYawRate.get());
    Serial.print("normSteering="); Serial.print(normSteering);
    Serial.print(" yaw(dps)="); Serial.print(yawRateFilt);
    Serial.print(" gain="); Serial.print(gain);
    Serial.print(" steerIn="); Serial.print(steerIn);
    Serial.print(" out="); Serial.print(out);
    Serial.print(" filtered_yaw_derivative="); Serial.print(filtered_yaw_derivative);
    
    Serial.print(" looptime="); Serial.println(loopTime);

    
  }
}
