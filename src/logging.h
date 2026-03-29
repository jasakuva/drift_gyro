#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// -----------------------------
// Configuration
// -----------------------------
#ifndef LOG_QUEUE_LENGTH
#define LOG_QUEUE_LENGTH 512
#endif

#ifndef LOG_BUFFER_SAMPLES
#define LOG_BUFFER_SAMPLES 128
#endif

// -----------------------------
// Data structure: timestamp + 8 parameters
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
};

// -----------------------------
// API
// -----------------------------
bool loggingBegin(uint8_t csPin, const char* filename = "/driftlog.bin", uint8_t core = 0);
bool loggingEnqueue(const LogSample& sample);
void loggingStop();
void loggingFlush();
bool loggingIsRunning();
uint32_t loggingDroppedSamples();

#endif