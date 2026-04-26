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
#include "ReturnDamping.h"
#include <math.h>

#include "SCH16T.h"
#include "logging.h"
#include "RollingPeak.h"

//#include <WiFi.h>
//#include <esp_wifi.h>
//#include <WebServer.h>
//#include <WebSocketsServer.h>

WobbleDetectorZC wob;
AdaptiveGains ag;

//#include <Wire.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>

//#define SDA_PIN 4
//#define SCL_PIN 5

#define FILTER_RATE         68.0f      // Hz, LPF0 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12        68.0f
#define FILTER_ACC3         68.0f
#define SENSITIVITY_RATE1   3200.0f     // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2   3200.0f
#define SENSITIVITY_ACC1    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2    3200.0f
#define SENSITIVITY_ACC3    3200.0f     // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE     4          // DEC2, Output sample rate decimation. Nominal output rate of 5.9kHz.
#define DECIMATION_ACC      4

#define SPI_OBJECT          SPI         // Some platforms have additional SPI intefaces under different names (e.g. SPI1)
#define CS_PIN              10
#define RESET_PIN           -1          //leave as -1 if not used

SCH16T_K01 imu(SPI_OBJECT, CS_PIN, RESET_PIN);

char serial_num[15];
int  init_status = SCH16T_ERR_OTHER;
SCH16T_filter         Filter;
SCH16T_sensitivity    Sensitivity;
SCH16T_decimation     Decimation;

SCH16T_raw_data raw;
SCH16T_result result;

float acc_lateral_setpoint;



// ---- Pins ----
const uint8_t PIN_STEER_IN  = 3;
const uint8_t PIN_GAIN_IN   = 4;
const uint8_t PIN_SERVO_OUT = 5;

const unsigned long LOOP_PERIOD_US = 5000;   // 5 ms = 200 Hz
const float LOOP_PERIOD_MS = LOOP_PERIOD_US / 1000.0f;
const float LOOP_PERIOD_S  = LOOP_PERIOD_MS / 1000.0f;
unsigned long nextLoopTime;
uint32_t nextLoopTimeUs = 0;
float dt_s = LOOP_PERIOD_US * 1e-6f;
int16_t out;

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

portMUX_TYPE cp_mux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t wifiTaskHandle = nullptr;
hw_timer_t* controlTimer = nullptr;

#define BUFFER_SIZE 60

ControlParams cp;

float Roll;
float Pitch;
float Yaw;
float PrewYaw;
float yawRate_dps;
float corr;
float gyrodps;
float gyro_correction;
float drift_value;
float filtered_yaw_derivative;
float normSteering;


float lastGyroCorrection = 0.0f;

Preferences prefs_2;

//Adafruit_MPU6050 mpu;

//WebServer server2(80);
//WebSocketsServer ws2(81);   // WebSocket on port 81

Servo steerServo;
lpfilter gyro_lp(20.0, LOOP_PERIOD_S);
lpfilter derivative_lp(20.0, LOOP_PERIOD_S);
lpfilter servoin_lp(20.0, LOOP_PERIOD_S);
lpfilter servoout_lp(20.0, LOOP_PERIOD_S);
lpfilter corr_lp(5.0, LOOP_PERIOD_S);
lpfilter correction_long_lp(0.2, LOOP_PERIOD_S);
lpfilter acc_lat_lp(20, LOOP_PERIOD_S);

derivative dYawRate(LOOP_PERIOD_MS);
derivative dSteering(LOOP_PERIOD_MS);

myMovingAverage<10> drift_value_avg;

DriftDetector driftd(LOOP_PERIOD_S, 0.05f, 10.0f);

ReturnDamping return_damp(0.25f, 0.1f, 0.0f, 0.0f);

RollingPeak rp_corr(17);

// ---- RC ranges ----
const int16_t RC_MIN = 1000;
const int16_t RC_MID = 1500;
const int16_t RC_MAX = 2000;

int epa_low_us_2    = 1100;
int epa_center_us_2 = 1500;
int epa_high_us_2   = 1900;

int to_settings_counter;

int logging_counter = 0;

SteeringMap mySteeringMap(1000, 2000, 1500);

// ---- ISR pulse capture ----
volatile uint32_t s_rise = 0, g_rise = 0;
volatile uint16_t s_pw   = RC_MID, g_pw = RC_MID;

bool settings_changed = false;

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

void IRAM_ATTR onControlTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(controlTaskHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

static inline int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float clampf(float v, float lo, float hi) {
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
  cp.dd_min_steer     = prefs_2.getFloat("dd_min_steer", 0.05);
  cp.dd_min_yaw       = prefs_2.getFloat("dd_min_yaw", 15);
  cp.corr_lp_hz       = prefs_2.getFloat("corr_lp", 5.0);
  cp.acc_lat_multip     = prefs_2.getFloat("acc_lat_multip", 5.0);

  gyro_lp.setCutoff(cp.gyro_lp_hz);
  derivative_lp.setCutoff(cp.derivative_lp_hz);
  servoin_lp.setCutoff(cp.steer_in_lp_hz);
  servoout_lp.setCutoff(cp.steer_out_lp_hz);
  corr_lp.setCutoff(cp.corr_lp_hz);
  acc_lat_lp.setCutoff(20);
  mySteeringMap.setEpaValues(epa_low_us_2, epa_high_us_2, epa_center_us_2);


  wob.setAmplitude(cp.wobble_det_a);
  driftd.init(LOOP_PERIOD_S, cp.dd_min_steer, cp.dd_min_yaw);

  prefs_2.end();
}

static void calibrateGyroOffset() {
  const int samples = 500;
  float sum = 0.0f;

  

  Serial.println("Calibrating MPU6050 gyro offset... keep car still");
  delay(500);

  

  gyroZOffset_dps = sum / samples;
  Serial.print("gyroZOffset_dps=");
  Serial.println(gyroZOffset_dps);
}

void reloadSettings() {
  gyro_lp.setCutoff(cp.gyro_lp_hz);
  derivative_lp.setCutoff(cp.derivative_lp_hz);
  servoin_lp.setCutoff(cp.steer_in_lp_hz);
  servoout_lp.setCutoff(cp.steer_out_lp_hz);
  wob.setAmplitude(cp.wobble_det_a);
  driftd.init(LOOP_PERIOD_S, cp.dd_min_steer, cp.dd_min_yaw);
  corr_lp.setCutoff(cp.corr_lp_hz);

}

void calibrateImu (){
  const int N = 10;
  float readings[N];

  for (int i = 0; i < N; i++) {
    imu.getData(&raw);
    imu.convertData(&raw, &result);
    readings[i] = result.Acc1[SCH16T_AXIS_X];  // Replace with your actual IMU read function
    delay(100);
  }

  // 2. Calculate average
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sum += readings[i];
  }
  float mean = sum / N;

  acc_lateral_setpoint = mean;

  // 3. Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < N; i++) {
    variance += pow(readings[i] - mean, 2);
  }
  variance /= N;

  float stdev = sqrt(variance);
}

void setup() {
  Serial.begin(115200);

  loadSettings_2();

  SPI_OBJECT.begin();

    delay(1000);

    Filter.Rate12 = FILTER_RATE;
    Filter.Acc12  = FILTER_ACC12;
    Filter.Acc3   = FILTER_ACC3;

    Sensitivity.Rate1 = SENSITIVITY_RATE1;
    Sensitivity.Rate2 = SENSITIVITY_RATE2;
    Sensitivity.Acc1  = SENSITIVITY_ACC1;
    Sensitivity.Acc2  = SENSITIVITY_ACC2;
    Sensitivity.Acc3  = SENSITIVITY_ACC3;

    Decimation.Rate2 = DECIMATION_RATE;
    Decimation.Acc2  = DECIMATION_ACC;

    while (init_status != SCH16T_OK)
    {
        init_status = imu.begin(Filter, Sensitivity, Decimation, false);
        if (init_status != SCH16T_OK) {
            Serial.println("ERROR");
            delay(3000);
        }
    }

    // Read serial number from the sensor.
    strcpy(serial_num, imu.getSnbr());
    Serial.print("Serial Number: ");
    Serial.println(serial_num);

  pinMode(PIN_STEER_IN, INPUT);
  pinMode(PIN_GAIN_IN, INPUT);
  delay(500);

  calibrateImu();

  steerServo.attach(PIN_SERVO_OUT);
  steerServo.setTimerWidth(14);

  attachInterrupt(digitalPinToInterrupt(PIN_STEER_IN), isrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_GAIN_IN),  isrGain,  CHANGE);

  delay(200);

  //if (s_pw < 1300 || s_pw > 1700) {
  //  makeSettings();
  //}

  loadSettings_2();

  delay(500);
  
  nextLoopTimeUs = micros() + LOOP_PERIOD_US;

   // Create control task on core 1
  xTaskCreatePinnedToCore(
    controlTask,         // task function
    "controlTask",       // name
    16384,                // stack size
    nullptr,             // parameter
    3,                   // priority
    &controlTaskHandle,  // handle
    1                    // core
  );

  
  xTaskCreatePinnedToCore(
    wifiTask,
    "wifiTask",
    12288,
    nullptr,
    4,
    &wifiTaskHandle,
    0
  );

  controlTimer = timerBegin(1000000);   // 1 MHz timer tick = 1 us
  timerAttachInterrupt(controlTimer, &onControlTimer);
  timerAlarm(controlTimer, LOOP_PERIOD_US, true, 0);   // auto-reload, unlimited

}

void wifiTask(void* pvParameters) {

 
  setupSettings();

  while (makeSettings()) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  wifiTaskHandle = nullptr;
  vTaskDelete(nullptr);
}


void controlTask(void* pvParameters) {
  uint32_t lastTickUs = micros();
  float acc_lat;
  float acc_lat_filt;
  float acc_lat_corr;

  for (;;) {
    // Wait until timer ISR wakes this task
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (settings_changed) {
      reloadSettings();
      settings_changed = false;
    }

    uint32_t nowUs = micros();
    float dt_s = (nowUs - lastTickUs) * 1e-6f;
    lastTickUs = nowUs;
  
    ControlParams cp_local;

    portENTER_CRITICAL(&cp_mux);
    cp_local = cp;
    portEXIT_CRITICAL(&cp_mux);
    
    
    // Read RC pulses atomically
    uint16_t steerIn, gainIn;
    portENTER_CRITICAL(&s_mux);
    steerIn = s_pw;
    gainIn  = g_pw;
    portEXIT_CRITICAL(&s_mux);

    float gain = ((float)gainIn - 1000.0f) / 1000.0f;
    gain = clamp16((int16_t)(gain * 1000), 0, 1000) / 1000.0f;

    steerIn = servoin_lp.update(steerIn);
    normSteering = mySteeringMap.getNormalized(steerIn);

    imu.getData(&raw);
    imu.convertData(&raw, &result);

    gyrodps = result.Rate1[SCH16T_AXIS_Z];
    acc_lat = result.Acc1[SCH16T_AXIS_X] - acc_lateral_setpoint;
    acc_lat_filt = acc_lat_lp.update(acc_lat);

    // Z gyro in deg/s
    //gyrodps = (g.gyro.z * 57.2957795f) - gyroZOffset_dps;
    Yaw = gyrodps;

    yawRateFilt = gyro_lp.update(gyrodps);                                  //   P  value
    float yawDerivativeRaw = dYawRate.update(gyrodps,LOOP_PERIOD_MS);
    filtered_yaw_derivative = derivative_lp.update(yawDerivativeRaw);       //   D  value
    
    // calculate drift propability
    drift_value = driftd.update(normSteering, yawRateFilt, cp.gain);
    float drift_multiplier = 1.0f+(drift_value/10)*cp.dd_multiplier;

    // calculate steering return damping value based on drifting value
    float return_damping = 0;
    return_damping = return_damp.getDamping(corr, 0.0f); 
    //if((yawRateFilt > 0 && filtered_yaw_derivative < 0) || (yawRateFilt < 0 && filtered_yaw_derivative > 0)) {
    //  return_damping = drift_value;
    //}
    
    float curve_gain = 1;
    if(fabsf(out)<100) {
      curve_gain = fabsf(out) * 0.01f;
    }

    // PID
    gyro_correction = curve_gain * drift_multiplier * cp.gain * gain * (cp.pid_p * yawRateFilt + cp.pid_d * filtered_yaw_derivative);

    corr = gyro_correction;

    // steering prio can reduce gyro correction
    corr = gyro_correction /
           (1.0f + (abs(dSteering.update(normSteering)) / 2.0f) * cp.steering_prio);

    // gyro correction exponent
    if (corr != 0.0f) {
      corr = (corr > 0.0f ? 1.0f : -1.0f) * powf(fabs(corr), cp.correction_exp);
    } else {
      corr = 0.0f;
    }

    acc_lat_corr = acc_lat_filt * cp.acc_lat_multip;
    acc_lat_corr = clampf(acc_lat_corr, -50.0f, 50.0f);

    corr = corr + acc_lat_corr;

    float correction_before_damping = corr;

    return_damping = return_damp.getDamping(corr, 0.0f); 

    corr = corr_lp.update(corr);
    //correction_long_lp.update(corr);

    //corr = rp_corr.addGetPeak(corr);

    float delta = corr - lastGyroCorrection;

    // return damping, percentage method
    //delta = delta * (1.0f - 1.0f*sqrt(return_damping));

    // return damping clamp version
    
    /*
    if (fabs(return_damping > 0)) {
      if (delta > 1) {
        delta = 1.0f - 0.9f*sqrt(return_damping);
      }
      
      if (delta < -1) {
        delta = -1.0f + 0.9f*sqrt(return_damping);
      } 
    }
    */
    corr = lastGyroCorrection + delta;
    
    //global damper
    if (fabs(delta) > cp.max_d_corr) {
      corr = lastGyroCorrection + (delta > 0 ? cp.max_d_corr : -cp.max_d_corr);
    }

    //lastGyroCorrection = correction_before_damping;
    lastGyroCorrection = corr;

    out = steerIn + (int16_t)corr;
    out = clamp16(out, epa_low_us_2, epa_high_us_2);

    out = servoout_lp.update(out);
    steerServo.writeMicroseconds(out);

    logging_counter ++;
    if(logging_counter >= 4) {
      float p1 = yawRateFilt;
      float p2 = gyrodps;
      float p3 = corr;
      float p4 = steerIn;
      float p5 = out;
      float p6 = drift_value;
      float p7 = filtered_yaw_derivative;
      float p8 = gain;
      float p9 = result.Acc1[SCH16T_AXIS_X];
      float p10 = result.Acc1[SCH16T_AXIS_Y];
      float p11 = result.Acc1[SCH16T_AXIS_Z];

      LogSample s;
        s.t_us = micros();

        // Replace with your real drift stabilizer parameters
        s.p1 = p1;
        s.p2 = p2;
        s.p3 = p3;
        s.p4 = p4;
        s.p5 = p5;
        s.p6 = p6;
        s.p7 = p7;
        s.p8 = p8;
        s.p9 = p9;
        s.p10 = p10;
        s.p11 = p11;

        loggingEnqueue(s);

        /*
        Serial.print("p1="); Serial.print(yawRateFilt);
        Serial.print("p3="); Serial.print(corr);
        Serial.print("p4="); Serial.print(steerIn);
        Serial.print("p8="); Serial.print(drift_multiplier);
        Serial.print("cp.gain="); Serial.println(cp.gain);
        */
      logging_counter = 0;
    }

    
    // debug printing
    static uint32_t lastPrint = 0;
    //if (millis() - lastPrint > 200 && cp.debug_serial == 1) {
    if ((millis() - lastPrint > 200) && (micros() - s_rise) > 1000000 && cp.debug_serial == 1) {
      lastPrint = millis();
      Serial.print("corr="); Serial.print(corr);
      Serial.print(" Result_gyro="); Serial.print(gyrodps);
      Serial.print(" gyro_correction="); Serial.print(gyro_correction);
      Serial.print(" yawRateFilt="); Serial.print(yawRateFilt);
      Serial.print(" normSteering="); Serial.print(normSteering);
      Serial.print(" gain="); Serial.print(gain);
      Serial.print(" steerIn="); Serial.print(steerIn);
      Serial.print(" out="); Serial.print(out);
      Serial.print(" drift_value="); Serial.print(drift_value);
      Serial.print(" filtered_yaw_derivative="); Serial.println(filtered_yaw_derivative);
      
      //Serial.print(" loopTime="); Serial.println(loopTime);
    }
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}