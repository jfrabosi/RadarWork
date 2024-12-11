// include/communication/GPSManager.h
#pragma once

#include <HardwareSerial.h>
#include "RTClib.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "communication/BluetoothManager.h"

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
  void powerOn();
  void powerOff();
  const GPSData &getCurrentLocation() const { return m_currentData; }
  bool hasValidFix() const { return m_hasFix; }
  void syncRTCWithGPS();

private:
  GPSManager() : m_isEnabled(false),
                 m_hasFix(false),
                 m_powerPin(0),
                 m_pRTC(nullptr) {}
  ~GPSManager() = default;

  GPSManager(const GPSManager &) = delete;
  GPSManager &operator=(const GPSManager &) = delete;

  bool parseGPGGA(const char *sentence);
  void convertDDtoDMS(float decimal_degrees, char *result, size_t size, bool isLat, char direction);
  void processIncomingData();
  void logStatus(const char *message);

  // member variables
  HardwareSerial m_gpsSerial { 1 }; // using UART1
  GPSData m_currentData;            // current GPS time/lat/long/pos
  bool m_isEnabled;                 // is GPS on?
  bool m_hasFix;                    // does GPS have a good fix? see parseGPGGA
  uint8_t m_powerPin;               // connected to MOSFET (GND switch, see init function)
  RTC_PCF8523 *m_pRTC;              // pointer to RTC object
  uint32_t m_lastGPSTime;           // tracks time since last fix
  BluetoothManager *m_pBT;          // pointer to Bluetooth Manager

  // parameters
  static constexpr uint32_t GPS_TIMEOUT = 5 * 60 * 1000;  // update rate for GPS in ms
  static constexpr uint32_t GPS_BAUD_RATE = 9600;         // baud rate, based on GPS model
  static constexpr size_t MAX_SENTENCE_LENGTH = 256;      // max characters in NMEA message
  static constexpr int8_t NUM_GPS_AVERAGES = 10;          // number of averages to take for lat/long/elev
  static constexpr int8_t NUM_GPS_WARMUPS = 10;           // number of warmup (discarded) readings for lat/long/elev
};