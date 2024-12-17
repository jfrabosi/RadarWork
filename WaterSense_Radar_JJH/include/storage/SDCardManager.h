// include/storage/SDCardManager.h
#pragma once

#include <SD.h>
#include "RTClib.h"
#include <string>
#include <queue>
#include <mutex>
#include <atomic>

typedef struct
{
  float start_m;
  float end_m;
  float update_rate;
  uint8_t max_step_length;
  uint8_t max_profile;
  float signal_quality;
  uint8_t reflector_shape;
  float threshold_sensitivity;
  uint8_t testing_update_rate;
  float true_update_rate;
  uint8_t text_width;
  char latitude[32];
  char longitude[32];
  char elevation[16];
} ConfigSettings;

class SDCardManager
{
public:
  // singleton pattern
  static SDCardManager &getInstance(size_t dataBufferLines = 100)
  {
    static SDCardManager instance(dataBufferLines);
    return instance;
  }

  bool initialize(uint8_t chipSelectPin, RTC_PCF8523 &rtc);
  void sdTask();

  void queueData(const char *format, ...);
  void queueDebug(const char *format, ...);
  void updateConfig(const ConfigSettings &config);
  ConfigSettings getConfig();
  void requestNewDataFile() { m_needNewDataFile = true; }

  void flushDebugBuffer();
  void flushDataBuffer();

  bool hasActiveOperations() const { return m_operationInProgress.load(); }

private:
  class OperationGuard
  {
  public:
    OperationGuard(std::atomic<bool> &flag) : m_flag(flag)
    {
      m_flag.store(true);
    }
    ~OperationGuard()
    {
      m_flag.store(false);
    }

  private:
    std::atomic<bool> &m_flag;
  };

  explicit SDCardManager(size_t dataBufferLines)
      : m_isInitialized(false),
        m_pRTC(nullptr),
        m_debugBufferPos(0),
        m_dataBufferPos(0),
        m_lastFlushTime(0),
        m_linesSaved(0),
        m_maxDataBufferLines(dataBufferLines),
        m_currentDebugPath(nullptr),
        m_currentDataPath(nullptr),
        m_needNewDataFile(false),
        m_needConfigSave(false)
  {
    m_debugBuffer[0] = '\0';
    m_dataBuffer[0] = '\0';
  }

  ~SDCardManager()
  {
    if (m_currentDebugPath)
      free(m_currentDebugPath);
    if (m_currentDataPath)
      free(m_currentDataPath);
  }

  // prevent copying
  SDCardManager(const SDCardManager &) = delete;
  SDCardManager &operator=(const SDCardManager &) = delete;

  bool createDirectory(const char *path);
  bool appendToFile(const char *path, const char *message);
  bool deleteFile(const char *path);

  bool startNewDebugFile(char **debugFilePath);
  void appendDebug(const char *format, ...);

  bool startNewDataFile(char **dataFilePath);
  void appendData(const char *format, ...);

  bool readConfig(ConfigSettings *config);
  bool saveConfig(const ConfigSettings *config);
  bool verifyConfig(const ConfigSettings *config);

  void logStatus(const char *format, ...);

  ConfigSettings getDefaultConfig() const
  {
    ConfigSettings config = {
        0.10f,     // start_m
        0.50f,     // end_m
        0.8f,      // update_rate
        1,         // max_step_length
        5,         // max_profile
        20.0f,     // signal_quality
        1,         // reflector_shape
        0.50f,     // threshold_sensitivity
        0,         // testing_update_rate
        10.1f,     // true_update_rate
        40,        // text_width
        "Not Set", // latitude
        "Not Set", // longitude
        "Not Set"  // elevation
    };
    return config;
  }

  // member variables
  bool m_isInitialized;         // is the SD card initialized?
  RTC_PCF8523 *m_pRTC;          // pointer to RTC object
  size_t m_debugBufferPos;      // tracks debug buffer cursor position
  size_t m_dataBufferPos;       // tracks data buffer cursor position
  uint32_t m_lastFlushTime;     // last time that data was appended
  uint32_t m_linesSaved;        // track number of data lines saved
  size_t m_maxDataBufferLines;  // gets set in constructor, max # of data lines before append is forced
  char *m_currentDebugPath;     // track current debug file path
  char *m_currentDataPath;      // track current data file path

  // parameters
  static constexpr size_t DEBUG_BUFFER_SIZE = 4096; // max length for debug lines
  static constexpr size_t MAX_LINE_LENGTH = 256;    // max length for data lines
  char m_debugBuffer[DEBUG_BUFFER_SIZE];            // buffer for 
  char m_dataBuffer[MAX_LINE_LENGTH * 100];         // default max 100 lines
  static constexpr uint32_t FLUSH_INTERVAL = 5000;  // 5 seconds
  static constexpr uint32_t MAX_LINES_PER_FILE = 1000000UL;
  static constexpr const char *CONFIG_FILE_PATH = "/radar_config.txt";

  // Queues for data
  std::queue<std::string> m_dataQueue;
  std::queue<std::string> m_debugQueue;

  // Mutexes for thread safety
  std::mutex m_dataQueueMutex;
  std::mutex m_debugQueueMutex;

  // Control flags
  volatile bool m_needNewDataFile;
  volatile bool m_needConfigSave;

  // Config storage
  ConfigSettings m_currentConfig;
  std::mutex m_configMutex;

  std::atomic<bool> m_operationInProgress{false}; // Track if any operation is running

  static constexpr size_t MAX_QUEUE_SIZE = 100; // Adjust based on needs
};