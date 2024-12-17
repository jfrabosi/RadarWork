// include/communication/GPSManager.h
#pragma once

#include <HardwareSerial.h>
#include "storage/SDCardManager.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Command codes
#define RADAR_CMD_REQUEST_CONFIG 0x3F
#define RADAR_CMD_CONFIG_GOOD 0x3E
#define RADAR_CMD_CONFIG_BAD 0x3C
#define RADAR_CMD_START_DATA 0x5B
#define RADAR_CMD_NEW_DATA 0x2A
#define RADAR_CMD_START_TEST 0x54
#define RADAR_CMD_END_TEST 0x74
#define RADAR_CMD_NOISE_ON 0x42
#define RADAR_CMD_NOISE_OFF 0x62
#define RADAR_CMD_STOP_REQUEST 0x58
#define RADAR_CMD_STOP_CONFIRM 0x78
#define RADAR_CMD_CONFIG_STRING 0x24
#define RADAR_CMD_DEBUG_MSG 0x21
#define RADAR_NULL 0x00

// Header bytes
#define RADAR_HEADER_BYTE1 0x4F
#define RADAR_HEADER_BYTE2 0x3A

class RadarManager
{
public:
  static RadarManager &getInstance()
  {
    static RadarManager instance;
    return instance;
  }

  bool initialize(uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 921600);
  void radarTask();

  bool sendConfig(const ConfigSettings &config);
  bool stopDataCollection();
  bool isActive() const { return m_isActive; }
  bool isTesting() const { return m_isTesting; }
  bool isMeasuring() const { return m_measurementInProgress; }
  float getSamplePeriod() const { return m_samplePeriod; }
  bool isSamplingPeriodOver() const { return m_samplePeriodOver; }

private:
  RadarManager() : m_isActive(false),
                   m_isTesting(false),
                   m_noiseBlocking(false),
                   m_testStartTime(0),
                   m_rxPin(0),
                   m_txPin(0),
                   m_measurementInProgress(false),
                   m_samplePeriod(0.0f),
                   m_timingInProgress(false),
                   m_timingStartTick(0),
                   m_sampleCount(0),
                   m_discardCount(0),
                   m_sampleCountMax(1),
                   m_samplePeriodOver(false) {}
  ~RadarManager() = default;

  RadarManager(const RadarManager &) = delete;
  RadarManager &operator=(const RadarManager &) = delete;

  bool sendCommand(uint8_t cmd);
  bool sendCommandWithData(uint8_t cmd, const uint8_t *data, size_t len);
  bool processRadarData();
  void handleDistanceData(const uint8_t *data, size_t len);
  void logStatus(const char *format, ...);
  bool performStopSequence(uint32_t delay_ms, uint32_t timeout_ms);
  float calculateUpdateRate(uint8_t count, uint32_t elapsed_ms);
  bool validateConfigEcho(const uint8_t *received, size_t len);

  HardwareSerial m_serial{2}; // UART2
  bool m_isActive;
  bool m_isTesting;
  bool m_noiseBlocking;
  uint32_t m_testStartTime;
  uint32_t m_lastCommandTime;
  ConfigSettings m_currentConfig;
  uint8_t m_rxPin;  // Added RX pin storage
  uint8_t m_txPin;  // Added TX pin storage
  uint32_t m_lastPrintTime = 0;
  bool m_measurementInProgress;
  float m_samplePeriod;       // Time for one sample in milliseconds
  bool m_timingInProgress;    // Are we currently timing samples?
  uint32_t m_timingStartTick; // When did timing start?
  uint32_t m_sampleCount;     // How many samples received during timing
  uint32_t m_discardCount;    // How many samples to discard
  uint32_t m_sampleCountMax;  // Sample count goal
  bool m_samplePeriodOver;

      static constexpr size_t MAX_DATA_SIZE = 256;
  static constexpr uint32_t CONFIG_TIMEOUT_MS = 2500;
  static constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
  static constexpr uint32_t STOP_TIMEOUT_MS = 3000;
  static constexpr uint8_t MAX_BT_PRINTS_PER_SEC = 11;  // Adjust this value as needed
  static constexpr uint32_t MIN_PRINT_INTERVAL_MS = 1000 / MAX_BT_PRINTS_PER_SEC;
};