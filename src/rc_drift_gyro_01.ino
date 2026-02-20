
#include <ESP32Servo.h>
#include "DerivativeN.h"
#include "SteeringMap.h"
//#include <CodeCell.h>
#include "settings.h"
#include <Preferences.h>
#include <Arduino.h>
#include "myMovingAverage.h"

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 8
#define SCL_PIN 9

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#define BUFFER_SIZE 30

float Roll;
float Pitch;
float Yaw;
volatile float yawRate_dps;

float lastGyroCorrection;

Preferences prefs_2;

//CodeCell myCodeCell;
Adafruit_BNO08x bno08x;

Servo steerServo;

DerivativeN dYawRate(20);
DerivativeN dSteering(10);
DerivativeN dOutput_long(50);
myMovingAverage<50> norm_output_avg;

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

float gain_main_2;
int gyro_avg_2;
int deriv_yaw_window_2;
int deriv_steer_window_2;
float steer_prio_2;
float gyro_dp_2;
int debug_serial_2=0;
int return_damping_2=5;
float gain_exp_2=1;

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

  gain_main_2 = prefs_2.getFloat("gain_main", 1.0);
  gyro_avg_2  = prefs_2.getInt("gyro_avg", 6);
  deriv_yaw_window_2 = prefs_2.getInt("d_y_a", 15);
  deriv_steer_window_2 = prefs_2.getInt("d_y_s", 5);
  steer_prio_2 = prefs_2.getFloat("s_p", 1.0);
  gyro_dp_2 = prefs_2.getFloat("g_dp", 0.5);
  return_damping_2 = prefs_2.getInt("return_damp",5);
  gain_exp_2 = prefs_2.getFloat("g_exp", 1);
  

  Serial.print("gyro_avg_2="); Serial.print(gyro_avg_2);
  Serial.print("deriv_yaw_window_2="); Serial.print(deriv_yaw_window_2);

  prefs_2.end();
}

void setup() {
  Serial.begin(115200);

  loadSettings_2();

  //myCodeCell.Init(MOTION_GYRO);
  Wire.begin(SDA_PIN, SCL_PIN);   // 👈 Set custom I2C pins
  Wire.setClock(400000);   // 400kHz I2C

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not found");
    while (1);
  }
  Serial.println("BNO085 found");
  bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 5000);

  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);
  delay(500);

  steerServo.attach(PIN_SERVO_OUT);
  
  steerServo.setTimerWidth(14);
  
  //for (int us = 1600; us >= 1400; us -= 1) {
  //  steerServo.writeMicroseconds(us);
  //  delay(5);
  //}

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  delay(1000);

  //for (int i = 0; i < 10000; i++) {
  //  steerServo.writeMicroseconds(s_pw);
  //  delay(5);
  //}

  //makeSettings();

  if (s_pw < 1300 || s_pw > 1700)
  {
    makeSettings();
  };

  loadSettings_2();

  delay(5000);

  dYawRate.setWindow(deriv_yaw_window_2);
  dSteering.setWindow(deriv_steer_window_2);

  //if (calibrateYawAvg(myCodeCell, gz_offset)) {
  //gz_offset = gz_offset;   // store calibration
  //} else {
    Serial.println("Yaw calibration failed");
  //}
  
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
  //if (myCodeCell.Run(100)) { 
  //  myCodeCell.Motion_GyroRead(Roll, Pitch, Yaw); 
  //}

  sh2_SensorValue_t sensorValue;

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      Yaw = sensorValue.un.gyroscope.z;
    }
  }

  float Result_gyro = moving_average_gyro(Yaw * 115.0f, gyro_avg_2);

  // go to settings mode if steerin near epa and no yaw rate
  if ((steerIn < epa_low_us_2+50 || steerIn > epa_high_us_2-50) && abs(Result_gyro) < 15) {
    to_settings_counter++;
    if (to_settings_counter > 400) {
      makeSettings();
      loadSettings_2();
      to_settings_counter = 0;
    }
  } else {
    to_settings_counter = 0;
  }

  yawRate_dps = Result_gyro;

  // Low-pass filter yaw rate to reduce twitch
  static float yawRateFilt = 0.0f;
  // yawRateFilt = lpf(yawRateFilt, yawRate_dps, 0.2f); // alpha 0.1..0.3 typical
  yawRateFilt = yawRate_dps;

  dYawRate.add(yawRateFilt);
  dSteering.add(normSteering);

  float gain = ((float)gainIn - 1000.0f) / 1000.0f;  // 0..1
  gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;
  
  float gyro_correction = gain_main_2*((1.0f-gyro_dp_2)*(yawRateFilt / 30.0f)*(gain) + (gyro_dp_2)*(dYawRate.get() / 50.0f)*(gain));
  
  float corr = gyro_correction / (1.0f + (abs(dSteering.get())/2.0f)*steer_prio_2);
  if(fabs(corr) < fabs(lastGyroCorrection)) {
    corr = (corr+static_cast<float>(return_damping_2)*lastGyroCorrection)/static_cast<float>(return_damping_2+1);
  }
  
  if (corr != 0.0f) {
    corr = (corr > 0.0f ? 1.0f : -1.0f) * powf(fabs(corr), gain_exp_2);
  } else {
      corr = 0.0f; // avoid powf(0, <=0)
  }

  

  // slow down correction in near center
  //float combined = normSteering + corr;
  //if (fabs(combined) < 0.1f && fabs(norm_output_avg.get()) < 0.05 ) {
  //  corr = (combined)*10.0f*corr;
  //}
  //float scale = fabs(combined) / 0.1f;  // linear from 0 to 1
  //if (scale > 1.0f ) scale = 1.0f;       // clamp to max 1
  //corr *= scale;

  lastGyroCorrection = corr;

  dOutput_long.add(normSteering + corr);
  norm_output_avg.update(normSteering + corr);

  int16_t out = (int16_t)mySteeringMap.getServoMsValue(normSteering + corr);
  out = clamp16(out, epa_low_us_2, epa_high_us_2);

  steerServo.writeMicroseconds(out);

  delay(2);


  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200 && debug_serial_2 == 1) {
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
