#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// -----------------------------
// Configuration
// -----------------------------
#ifndef LOG_QUEUE_LENGTH
#define LOG_QUEUE_LENGTH 256
#endif

#ifndef LOG_BUFFER_SAMPLES
#define LOG_BUFFER_SAMPLES 11
#endif

// -----------------------------
// Data structure: timestamp + 1 parameters
// -----------------------------
struct LogSample {
    uint32_t t_us;
    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
    float p6;
    float p7;
    float p8;
    float p9;
    float p10;
    float p11;
};

// -----------------------------
// API
// -----------------------------
bool loggingBegin(uint8_t csPin,
                  uint8_t sckPin,
                  uint8_t misoPin,
                  uint8_t mosiPin,
                  const char* filename,
                  uint8_t core);
//bool loggingEnqueue(const LogSample& sample);
bool loggingEnqueue(LogSample sample);
void loggingStop();
void loggingFlush();
bool loggingIsRunning();
uint32_t loggingDroppedSamples();

#endif