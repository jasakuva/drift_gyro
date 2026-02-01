#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

extern WebServer server;

extern const char* AP_SSID;
extern const char* AP_PASS;

extern int epa_left_us;
extern int epa_center_us;
extern int epa_right_us;

extern int current_steering_us;

extern int   p1;
extern int   p2;
extern int   p3;
extern int   p4;
extern float p5;
extern float p6;
extern float p7;
extern int   p8;

void setupSettings();
void makeSettings();

void loadSettings();   // read from NVS into p1..p8
//void saveSettings();   // write p1..p8 into NVS