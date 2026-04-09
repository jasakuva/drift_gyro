#include "logging.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SD_CS_PIN D7
#define SD_SCK_PIN D8
#define SD_MISO_PIN D9
#define SD_MOSI_PIN D10

// -----------------------------
// Static globals
// -----------------------------
static QueueHandle_t s_logQueue = nullptr;
static TaskHandle_t s_loggerTaskHandle = nullptr;
static File s_logFile;
static volatile bool s_loggerRunning = false;
static volatile uint32_t s_droppedSamples = 0;

//static uint8_t s_sdCsPin   = 5;
//static uint8_t s_sdSckPin  = 7;
//static uint8_t s_sdMisoPin = 8;
//static uint8_t s_sdMosiPin = 9;

static SPIClass s_sdSpi(FSPI);   // ESP32-S3

static LogSample s_writeBuffer[LOG_BUFFER_SAMPLES];
static size_t s_writeBufferCount = 0;

// -----------------------------
// Internal helpers
// -----------------------------
//sckPin, misoPin, mosiPin, csPin)
static bool initSD(uint8_t s_sdSckPin, uint8_t s_sdMisoPin, uint8_t s_sdMosiPin, uint8_t s_sdCsPin)
{
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    Serial.println("spi initialized");

    if (!SD.begin(SD_CS_PIN)) {
    //if (!SD.begin(s_sdCsPin)) {
        Serial.println("SD.begin failed");
        return false;
    }
    Serial.println("sd initialized");
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }

    uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("SD card OK, size: %llu MB\n", cardSizeMB);
    return true;
}

static bool openLogFile(const char* filename)
{
    s_logFile = SD.open(filename, FILE_WRITE);
    if (!s_logFile) {
        Serial.printf("Failed to open %s\n", filename);
        return false;
    }

    Serial.printf("Opened log file: %s\n", filename);
    return true;
}

static void flushWriteBufferInternal()
{
    if (!s_logFile || s_writeBufferCount == 0) {
        return;
    }

    const size_t bytesToWrite = s_writeBufferCount * sizeof(LogSample);
    const size_t bytesWritten = s_logFile.write(
        reinterpret_cast<const uint8_t*>(s_writeBuffer),
        bytesToWrite
    );

    if (bytesWritten != bytesToWrite) {
        Serial.printf("SD write mismatch: wanted %u, wrote %u\n",
                      (unsigned)bytesToWrite,
                      (unsigned)bytesWritten);
    }

    s_writeBufferCount = 0;
}

static void loggerTask(void* parameter)
{
    (void)parameter;

    LogSample incoming;
    uint32_t lastFlushMs = millis();

    while (s_loggerRunning) {
        if (xQueueReceive(s_logQueue, &incoming, pdMS_TO_TICKS(20)) == pdTRUE) {
       //    Serial.printf("RX %lu %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
       // (unsigned long)incoming.t_us,
       // incoming.p1, incoming.p2, incoming.p3, incoming.p4,
       // incoming.p5, incoming.p6, incoming.p7, incoming.p8);
            
            if (s_writeBufferCount >= LOG_BUFFER_SAMPLES) {
                flushWriteBufferInternal();
            }

            s_writeBuffer[s_writeBufferCount++] = incoming;
          
        }
      
        uint32_t nowMs = millis();
        if ((nowMs - lastFlushMs) >= 100) {
            flushWriteBufferInternal();
            if (s_logFile) {
                s_logFile.flush();
            }
            lastFlushMs = nowMs;
        }
      
    }

    while (xQueueReceive(s_logQueue, &incoming, 0) == pdTRUE) {
        if (s_writeBufferCount >= LOG_BUFFER_SAMPLES) {
            flushWriteBufferInternal();
        }

        s_writeBuffer[s_writeBufferCount++] = incoming;
    }

    flushWriteBufferInternal();

    if (s_logFile) {
        s_logFile.flush();
        s_logFile.close();
    }

    s_loggerTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

// -----------------------------
// Public API
// -----------------------------

//loggingBegin(SD_CS_PIN, SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, filename,0);
bool loggingBegin(uint8_t csPin,
                  uint8_t sckPin,
                  uint8_t misoPin,
                  uint8_t mosiPin,
                  const char* filename,
                  uint8_t core)
{
    if (s_loggerRunning) {
        Serial.println("Logger already running");
        return true;
    }

    //s_sdCsPin   = csPin;
    //s_sdSckPin  = sckPin;
    //s_sdMisoPin = misoPin;
    //s_sdMosiPin = mosiPin;

    s_droppedSamples = 0;
    s_writeBufferCount = 0;

    if (!initSD(sckPin, misoPin, mosiPin, csPin)) {
        return false;
    }

    if (!openLogFile(filename)) {
        return false;
    }

    s_logQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogSample));
    if (s_logQueue == nullptr) {
        Serial.println("Failed to create log queue");
        s_logFile.close();
        return false;
    }

    s_loggerRunning = true;

    BaseType_t ok = xTaskCreatePinnedToCore(
        loggerTask,
        "LoggerTask",
        16384,
        nullptr,
        3,
        &s_loggerTaskHandle,
        core
    );

    if (ok != pdPASS) {
        Serial.println("Failed to create logger task");
        s_loggerRunning = false;
        vQueueDelete(s_logQueue);
        s_logQueue = nullptr;
        s_logFile.close();
        return false;
    }

    return true;
}

//bool loggingEnqueue(const LogSample& sample)
bool loggingEnqueue(LogSample sample)
{
    if (!s_loggerRunning || s_logQueue == nullptr) {
        return false;
    }

    BaseType_t ok = xQueueSend(s_logQueue, &sample, 0);
    if (ok != pdTRUE) {
        s_droppedSamples++;
        return false;
    }

    return true;
}

void loggingFlush()
{
    
}

void loggingStop()
{
    if (!s_loggerRunning) {
        return;
    }

    s_loggerRunning = false;

    uint32_t start = millis();
    while (s_loggerTaskHandle != nullptr && (millis() - start) < 2000) {
        delay(10);
    }

    if (s_logQueue != nullptr) {
        vQueueDelete(s_logQueue);
        s_logQueue = nullptr;
    }
}

bool loggingIsRunning()
{
    return s_loggerRunning;
}

uint32_t loggingDroppedSamples()
{
    return s_droppedSamples;
}