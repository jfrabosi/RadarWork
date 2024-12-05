// src/storage/SDCardManager.cpp
#include "storage/SDCardManager.h"
#include <Arduino.h>
#include <stdarg.h>

bool SDCardManager::initialize(uint8_t chipSelectPin, RTC_PCF8523 &rtc)
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelectPin))
  {
    Serial.println("initialization failed!");
    Serial.println("Things to check:");
    Serial.println("1. Is a card inserted?");
    Serial.println("2. Is your wiring correct?");
    Serial.println("3. Did you change the chipSelect pin to match your shield or module?");
    return false;
  }

  m_pRTC = &rtc;
  m_isInitialized = true;

  Serial.println("initialization successful.");

  // Check for and create required directories
  if (!SD.exists("/DEBUG_LOGS") || !SD.exists("/DATA"))
  {
    Serial.println("Creating required directories...");
    createDirectory("/DEBUG_LOGS");
    createDirectory("/DATA");
  }

  // Print card info
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  return true;
}

bool SDCardManager::createDirectory(const char *path)
{
  if (!m_isInitialized)
    return false;

  if (SD.mkdir(path))
  {
    Serial.printf("Created directory: %s\n", path);
    return true;
  }
  Serial.printf("Failed to create directory: %s\n", path);
  return false;
}

bool SDCardManager::appendToFile(const char *path, const char *message)
{
  if (!m_isInitialized)
    return false;

  File file = SD.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.printf("Failed to open file for appending: %s\n", path);
    return false;
  }

  if (!file.print(message))
  {
    Serial.println("Append failed");
    file.close();
    return false;
  }

  file.close();
  return true;
}

bool SDCardManager::deleteFile(const char *path)
{
  if (!m_isInitialized)
    return false;

  if (SD.remove(path))
  {
    Serial.printf("Deleted: %s\n", path);
    return true;
  }
  Serial.printf("Delete failed: %s\n", path);
  return false;
}

bool SDCardManager::startNewDebugFile(char **debugFilePath)
{
  if (!m_isInitialized || !m_pRTC)
    return false;

  static char filename[64];
  DateTime now = m_pRTC->now();

  snprintf(filename, sizeof(filename), "/DEBUG_LOGS/%02d-%02d-%02d_%02d-%02d-%02d_debug.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  if (SD.exists(filename))
  {
    Serial.printf("Debug file already exists: %s\n", filename);
    return false;
  }

  File file = SD.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to create new debug file");
    return false;
  }
  file.close();

  // Update the external pointer if provided
  if (debugFilePath)
  {
    if (*debugFilePath != nullptr)
    {
      free(*debugFilePath);
    }
    *debugFilePath = strdup(filename);
  }

  // Update internal path
  if (m_currentDebugPath != nullptr)
  {
    free(m_currentDebugPath);
  }
  m_currentDebugPath = strdup(filename);

  return true;
}

void SDCardManager::appendDebug(const char *format, ...)
{
  if (!m_isInitialized)
    return;

  va_list args;
  va_start(args, format);

  size_t remaining = DEBUG_BUFFER_SIZE - m_debugBufferPos - 1;
  int written = vsnprintf(m_debugBuffer + m_debugBufferPos, remaining, format, args);

  va_end(args);

  if (written > 0)
  {
    m_debugBufferPos += written;
    // Add newline if there's room
    if (m_debugBufferPos < DEBUG_BUFFER_SIZE - 1)
    {
      m_debugBuffer[m_debugBufferPos++] = '\n';
    }
  }

  // Check if buffer is nearly full or it's time to flush
  if (m_debugBufferPos > DEBUG_BUFFER_SIZE - 256 ||
      (millis() - m_lastFlushTime > FLUSH_INTERVAL))
  {
    flushDebugBuffer();
  }
}

void SDCardManager::flushDebugBuffer()
{
  if (!m_isInitialized || m_debugBufferPos == 0 || !m_currentDebugPath)
    return;

  m_debugBuffer[m_debugBufferPos] = '\0';
  appendToFile(m_currentDebugPath, m_debugBuffer);
  m_debugBufferPos = 0;
  m_lastFlushTime = millis();
}

bool SDCardManager::startNewDataFile(char **dataFilePath)
{
  if (!m_isInitialized || !m_pRTC)
    return false;

  static char filename[64];
  DateTime now = m_pRTC->now();

  snprintf(filename, sizeof(filename), "/DATA/%02d-%02d-%02d_%02d-%02d-%02d_data.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  if (SD.exists(filename))
  {
    Serial.printf("Data file already exists: %s\n", filename);
    return false;
  }

  File file = SD.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to create new data file");
    return false;
  }
  file.close();

  // Update the external pointer if provided
  if (dataFilePath)
  {
    if (*dataFilePath != nullptr)
    {
      free(*dataFilePath);
    }
    *dataFilePath = strdup(filename);
  }

  // Update internal path
  if (m_currentDataPath != nullptr)
  {
    free(m_currentDataPath);
  }
  m_currentDataPath = strdup(filename);

  m_linesSaved = 0;
  return true;
}

void SDCardManager::appendData(const char *format, ...)
{
  if (!m_isInitialized)
    return;

  va_list args;
  va_start(args, format);

  // Calculate position in buffer for new line
  size_t bufferOffset = m_dataBufferPos * MAX_LINE_LENGTH;
  size_t remaining = sizeof(m_dataBuffer) - bufferOffset - 1;

  // Format the message
  int written = vsnprintf(m_dataBuffer + bufferOffset, remaining, format, args);

  va_end(args);

  if (written > 0 && written < remaining)
  {
    // Add newline
    m_dataBuffer[bufferOffset + written] = '\n';
    m_dataBuffer[bufferOffset + written + 1] = '\0'; // Null terminate this line
    m_dataBufferPos++;

    // Check if buffer is full
    if (m_dataBufferPos >= m_maxDataBufferLines)
    {
      flushDataBuffer();
    }
  }
}

void SDCardManager::flushDataBuffer()
{
  if (!m_isInitialized || m_dataBufferPos == 0 || !m_currentDataPath)
    return;

  // Already null-terminated by appendData
  appendToFile(m_currentDataPath, m_dataBuffer);

  // Clear buffer
  m_dataBuffer[0] = '\0';
  m_dataBufferPos = 0;
  m_linesSaved += m_maxDataBufferLines;

  // Check if we need to start a new file
  if (m_linesSaved >= MAX_LINES_PER_FILE)
  {
    startNewDataFile(nullptr); // Only update internal path
  }
}

uint64_t SDCardManager::getTotalSpace() const
{
  return m_isInitialized ? SD.totalBytes() : 0;
}

uint64_t SDCardManager::getUsedSpace() const
{
  return m_isInitialized ? SD.usedBytes() : 0;
}