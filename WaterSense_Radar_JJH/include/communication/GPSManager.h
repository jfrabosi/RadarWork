#pragma once

#include <HardwareSerial.h>
#include "RTClib.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct GPSData
{
  int hour;
  int minute;
  int second;
  char latitude[32];
  char longitude[32];
  char elevation[16];
};

class GPSManager
{
public:
  static GPSManager &getInstance()
  {
    static GPSManager instance;
    return instance;
  }

  bool initialize(uint8_t rxPin, uint8_t txPin, uint8_t powerPin, RTC_PCF8523 &rtcRef);
  void gpsTask();

  // Power management
  void powerOn();
  void powerOff();

  // GPS data access
  const GPSData &getCurrentLocation() const { return m_currentData; }
  bool hasValidFix() const { return m_hasFix; }

  // Debug control
  void setDebugMode(bool enable) { m_debugMode = enable; }
  bool isDebugEnabled() const { return m_debugMode; }

  // RTC synchronization
  void syncRTCWithGPS();

private:
  GPSManager() : m_isEnabled(false),
                 m_hasFix(false),
                 m_powerPin(0),
                 m_pRTC(nullptr),
                 m_debugMode(false),
                 m_lastDebugPrint(0) {}
  ~GPSManager() = default;

  GPSManager(const GPSManager &) = delete;
  GPSManager &operator=(const GPSManager &) = delete;

  // Internal helper functions
  bool parseGPGGA(const char *sentence);
  void convertDMtoDMS(float decimal_degrees, char *result, size_t size, bool isLat, char direction);
  int8_t getUTCOffset();
  bool isDST(uint16_t year, uint8_t month, uint8_t day, uint8_t hour);
  void processIncomingData();
  void printDebugInfo(const char *sentence);

  // Member variables
  HardwareSerial m_gpsSerial{1}; // Using UART1
  GPSData m_currentData;
  bool m_isEnabled;
  bool m_hasFix;
  uint8_t m_powerPin;
  RTC_PCF8523 *m_pRTC; // Pointer to RTC object
  bool m_debugMode;
  uint32_t m_lastDebugPrint;
  uint32_t m_lastGPSTime;

  static constexpr uint32_t GPS_TIMEOUT = 5 * 60 * 1000;
  static constexpr uint32_t GPS_BAUD_RATE = 9600;
  static constexpr size_t MAX_SENTENCE_LENGTH = 256;
  static constexpr int8_t NUM_GPS_AVERAGES = 10;
  static constexpr int8_t NUM_GPS_WARMUPS = 10;
  static constexpr uint32_t DEBUG_PRINT_INTERVAL = 30000; // Print debug every 1 second
};