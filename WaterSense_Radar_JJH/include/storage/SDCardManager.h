#pragma once

#include <SD.h>
#include "RTClib.h"
#include <string>

class SDCardManager
{
public:
  // Singleton pattern with configurable buffer sizes
  static SDCardManager &getInstance(size_t dataBufferLines = 100)
  {
    static SDCardManager instance(dataBufferLines);
    return instance;
  }

  // Initialize SD card with given chip select pin
  bool initialize(uint8_t chipSelectPin, RTC_PCF8523 &rtc);

  // Basic file operations
  bool createDirectory(const char *path);
  bool appendToFile(const char *path, const char *message);
  bool deleteFile(const char *path);

  // Debug file management
  bool startNewDebugFile(char **debugFilePath);
  void flushDebugBuffer();
  void appendDebug(const char *format, ...);

  // Data file management
  bool startNewDataFile(char **dataFilePath);
  void appendData(const char *format, ...);
  void flushDataBuffer();

  // Buffer configuration
  void setDataBufferSize(size_t lines) { m_maxDataBufferLines = lines; }
  size_t getDataBufferSize() const { return m_maxDataBufferLines; }

  // Status checks
  bool isInitialized() const { return m_isInitialized; }
  uint64_t getTotalSpace() const;
  uint64_t getUsedSpace() const;

private:
  // Constructor now takes buffer size parameter
  explicit SDCardManager(size_t dataBufferLines)
      : m_isInitialized(false), m_pRTC(nullptr), m_debugBufferPos(0), m_dataBufferPos(0), m_lastFlushTime(0), m_linesSaved(0), m_maxDataBufferLines(dataBufferLines), m_currentDebugPath(nullptr), m_currentDataPath(nullptr)
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

  // Prevent copying
  SDCardManager(const SDCardManager &) = delete;
  SDCardManager &operator=(const SDCardManager &) = delete;

  bool m_isInitialized;
  RTC_PCF8523 *m_pRTC;
  size_t m_debugBufferPos;
  size_t m_dataBufferPos;
  uint32_t m_lastFlushTime;
  uint32_t m_linesSaved;
  size_t m_maxDataBufferLines;
  char *m_currentDebugPath; // Added to track current debug file path
  char *m_currentDataPath;  // Added to track current data file path

  // Buffers for debug and data messages
  static constexpr size_t DEBUG_BUFFER_SIZE = 4096;
  static constexpr size_t MAX_LINE_LENGTH = 256;
  char m_debugBuffer[DEBUG_BUFFER_SIZE];
  char m_dataBuffer[MAX_LINE_LENGTH * 100]; // Default max 100 lines

  // Constants
  static constexpr uint32_t FLUSH_INTERVAL = 5000; // 5 seconds
  static constexpr uint32_t MAX_LINES_PER_FILE = 1000000UL;
};