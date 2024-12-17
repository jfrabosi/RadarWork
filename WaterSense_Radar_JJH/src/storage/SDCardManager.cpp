// src/storage/SDCardManager.cpp
#include "storage/SDCardManager.h"
#include "storage/TimeManager.h"
#include <Arduino.h>
#include <stdarg.h>

/**
 * @brief Initializes the SD Card Manager
 * @param chipSelectPin Pin number for SD card chip select
 * @param rtc Reference to RTC object for timestamping
 * @return true for successful initialization, false if error occurred
 *
 * Initializes SD card, creates required directories, and checks card type.
 * Required directories are /DEBUG_LOGS and /DATA. Debug messages are written
 * during initialization to help troubleshoot any issues.
 */
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

  // Load the initial configuration
  if (!readConfig(&m_currentConfig))
  {
    logStatus("Failed to load initial configuration");
    return false;
  }
  logStatus("Initial configuration loaded from SD card\n");

  return true;
}


/**
 * @brief Main task for SD Card Manager
 * @return none
 *
 * SDTask handles several key operations:
 * - Creates new data files when needed
 * - Processes queued data and debug messages
 * - Saves configuration changes
 * - Flushes buffers periodically
 *
 * All operations use appropriate mutex locks for thread safety
 */
void SDCardManager::sdTask()
{
  if (!m_isInitialized)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }

  while (true)
  {
    // Handle new data file creation if needed
    if (m_needNewDataFile)
    {
      if (startNewDataFile(nullptr))
      {
        m_needNewDataFile = false;
      }
    }

    // Process data queue
    {
      std::lock_guard<std::mutex> lock(m_dataQueueMutex);
      while (!m_dataQueue.empty())
      {
        appendData(m_dataQueue.front().c_str());
        m_dataQueue.pop();
      }
    }

    // Process debug queue
    {
      std::lock_guard<std::mutex> lock(m_debugQueueMutex);
      while (!m_debugQueue.empty())
      {
        appendDebug(m_debugQueue.front().c_str());
        m_debugQueue.pop();
      }
    }

    // Save config if needed
    if (m_needConfigSave)
    {
      {
        std::lock_guard<std::mutex> lock(m_configMutex);
        if (saveConfig(&m_currentConfig))
        {
          m_needConfigSave = false;
        }
      }
    }

    // Ensure buffers are flushed periodically
    if (millis() - m_lastFlushTime > FLUSH_INTERVAL)
    {
      flushDebugBuffer();
      flushDataBuffer();
    }

    // Prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


/**
 * @brief Queues data message for writing to SD card
 * @param format Treat this function like a wrapper for printf
 * @return none
 *
 * Thread-safe function to queue data messages. Messages are buffered and
 * written in batches to improve SD card performance.
 */
void SDCardManager::queueData(const char *format, ...)
{
  OperationGuard guard(m_operationInProgress);
  char buffer[MAX_LINE_LENGTH];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  std::lock_guard<std::mutex> lock(m_dataQueueMutex);
  if (m_dataQueue.size() >= MAX_QUEUE_SIZE)
  {
    flushDataBuffer();
  }
  else
  {
    m_dataQueue.push(std::string(buffer));
  }
}


/**
 * @brief Queues debug message for writing to SD card
 * @param format Treat this function like a wrapper for printf
 * @return none
 *
 * Thread-safe function to queue debug messages. Messages are buffered and
 * written in batches to improve SD card performance.
 */
void SDCardManager::queueDebug(const char *format, ...)
{
  OperationGuard guard(m_operationInProgress);
  char buffer[MAX_LINE_LENGTH];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  std::lock_guard<std::mutex> lock(m_debugQueueMutex);
  if (m_debugQueue.size() >= MAX_QUEUE_SIZE)
  {
    flushDebugBuffer();
  }
  else
  {
    m_debugQueue.push(std::string(buffer));
  }
}


/**
 * @brief Updates system configuration and marks for saving
 * @param config New configuration settings to save
 * @return none
 *
 * Thread-safe function to update configuration. Changes are queued and
 * saved to SD card by the main task.
 */
void SDCardManager::updateConfig(const ConfigSettings &config)
{
  OperationGuard guard(m_operationInProgress);
  std::lock_guard<std::mutex> lock(m_configMutex);
  m_currentConfig = config;
  m_needConfigSave = true;
}


/**
 * @brief Retrieves current system configuration
 * @return Current ConfigSettings struct
 *
 * Thread-safe function to get current configuration settings
 */
ConfigSettings SDCardManager::getConfig()
{
  OperationGuard guard(m_operationInProgress);
  std::lock_guard<std::mutex> lock(m_configMutex);
  return m_currentConfig;
}

/**
 * @brief Creates a new directory on the SD card
 * @param path Directory path to create
 * @return true if directory created successfully, false if error
 */
bool SDCardManager::createDirectory(const char *path)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized)
    return false;

  if (SD.mkdir(path))
  {
    logStatus("Created directory: %s\n", path);
    return true;
  }
  logStatus("Failed to create directory: %s\n", path);
  return false;
}


/**
 * @brief Appends message to specified file
 * @param path File path to append to
 * @param message Message to append
 * @return true if message appended successfully, false if error
 */
bool SDCardManager::appendToFile(const char *path, const char *message)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized)
    return false;

  File file = SD.open(path, FILE_APPEND);
  if (!file)
  {
    logStatus("Failed to open file for appending: %s\n", path);
    return false;
  }

  if (!file.print(message))
  {
    logStatus("Append failed");
    file.close();
    return false;
  }

  file.close();
  return true;
}


/**
 * @brief Deletes specified file from SD card
 * @param path File path to delete
 * @return true if file deleted successfully, false if error
 */
bool SDCardManager::deleteFile(const char *path)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized)
    return false;

  if (SD.remove(path))
  {
    logStatus("Deleted: %s\n", path);
    return true;
  }
  logStatus("Delete failed: %s\n", path);
  return false;
}


/**
 * @brief Creates new debug log file with timestamp name
 * @param debugFilePath Optional pointer to store new file path
 * @return true if file created successfully, false if error
 *
 * Creates debug file with name format: DD-MM-YY_HH-MM-SS_debug.txt
 * Updates both internal path and optional external pointer
 */
bool SDCardManager::startNewDebugFile(char **debugFilePath)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || !m_pRTC)
    return false;

  static char filename[64];
  DateTime now = m_pRTC->now();

  snprintf(filename, sizeof(filename), "/DEBUG_LOGS/%02d-%02d-%02d_%02d-%02d-%02d_debug.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  if (SD.exists(filename))
  {
    logStatus("Debug file already exists: %s\n", filename);
    return false;
  }

  File file = SD.open(filename, FILE_WRITE);
  if (!file)
  {
    logStatus("Failed to create new debug file");
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


/**
 * @brief Appends debug message to current debug file
 * @param format Treat this function like a wrapper for printf
 * @return none
 *
 * Messages are buffered and written in batches. Buffer is flushed when
 * nearly full or after FLUSH_INTERVAL milliseconds
 */
void SDCardManager::appendDebug(const char *format, ...)
{
  OperationGuard guard(m_operationInProgress);
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


/**
 * @brief Flushes debug buffer to SD card
 * @return none
 *
 * Forces writing of buffered debug messages to SD card
 * Called automatically when buffer is full or by timer
 */
void SDCardManager::flushDebugBuffer()
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || m_debugBufferPos == 0)
    return;

  if (!m_currentDebugPath)
  {
    startNewDebugFile(nullptr);
  }

  m_debugBuffer[m_debugBufferPos] = '\0';
  appendToFile(m_currentDebugPath, m_debugBuffer);
  m_debugBufferPos = 0;
  m_lastFlushTime = millis();
}


/**
 * @brief Creates new data file with timestamp name
 * @param dataFilePath Optional pointer to store new file path
 * @return true if file created successfully, false if error
 *
 * Creates data file with name format: DD-MM-YY_HH-MM-SS_data.txt
 * Updates both internal path and optional external pointer
 * Resets initial time when creating new file
 */
bool SDCardManager::startNewDataFile(char **dataFilePath)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || !m_pRTC)
    return false;

  static char filename[64];
  DateTime now = m_pRTC->now();

  snprintf(filename, sizeof(filename), "/DATA/%02d-%02d-%02d_%02d-%02d-%02d_data.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  TimeManager::getInstance().resetInitialTime();

  if (SD.exists(filename))
  {
    logStatus("Data file already exists: %s\n", filename);
    return false;
  }

  File file = SD.open(filename, FILE_WRITE);
  if (!file)
  {
    logStatus("Failed to create new data file");
    return false;
  }

  // Write configuration header to the new file
  char timeStr[32];
  TimeManager::getInstance().getFormattedTimestamp(timeStr, sizeof(timeStr));

  // Create header buffer
  char header[1024];
  int pos = 0;

  // Build header string
  pos += snprintf(header + pos, sizeof(header) - pos, "First Data File Since Power On: %s\n",
                  m_linesSaved == 0 ? "True" : "False");
  pos += snprintf(header + pos, sizeof(header) - pos, "Data File: %s\n", filename);
  pos += snprintf(header + pos, sizeof(header) - pos, "Start Time: %s\n", timeStr);
  pos += snprintf(header + pos, sizeof(header) - pos, "Location: %s %s\n",
                  m_currentConfig.latitude, m_currentConfig.longitude);
  pos += snprintf(header + pos, sizeof(header) - pos, "Elevation: %s\n",
                  m_currentConfig.elevation);
  pos += snprintf(header + pos, sizeof(header) - pos, "Start of range: %.2f m\n",
                  m_currentConfig.start_m);
  pos += snprintf(header + pos, sizeof(header) - pos, "End of range: %.2f m\n",
                  m_currentConfig.end_m);
  pos += snprintf(header + pos, sizeof(header) - pos, "Update rate: %.1f Hz\n",
                  m_currentConfig.update_rate);
  pos += snprintf(header + pos, sizeof(header) - pos, "Maximum step length: %d (%.1f mm)\n",
                  m_currentConfig.max_step_length,
                  (float)m_currentConfig.max_step_length * 2.5f);
  pos += snprintf(header + pos, sizeof(header) - pos, "Maximum profile: %d\n",
                  m_currentConfig.max_profile);
  pos += snprintf(header + pos, sizeof(header) - pos, "Signal quality: %.1f\n",
                  m_currentConfig.signal_quality);
  pos += snprintf(header + pos, sizeof(header) - pos, "Reflector shape: %d (0: generic, 1: planar)\n",
                  m_currentConfig.reflector_shape);
  pos += snprintf(header + pos, sizeof(header) - pos, "Threshold sensitivity: %.2f\n",
                  m_currentConfig.threshold_sensitivity);
  pos += snprintf(header + pos, sizeof(header) - pos, "True update rate: %.1f Hz\n",
                  m_currentConfig.true_update_rate);
  pos += snprintf(header + pos, sizeof(header) - pos, "Text width: %d characters\n",
                  m_currentConfig.text_width);
  pos += snprintf(header + pos, sizeof(header) - pos, "---\n"); // Add separator line

  // Write header to file
  file.write((uint8_t *)header, strlen(header));
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

  logStatus("New data file created: %s", filename);

  m_linesSaved = 0;
  return true;
}


/**
 * @brief Appends data message to current data file
 * @param format Treat this function like a wrapper for printf
 * @return none
 *
 * Messages are buffered and written in batches. New file is created
 * automatically when MAX_LINES_PER_FILE is reached
 */
void SDCardManager::appendData(const char *format, ...)
{
  OperationGuard guard(m_operationInProgress);
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


/**
 * @brief Flushes data buffer to SD card
 * @return none
 *
 * Forces writing of buffered data messages to SD card
 * Called automatically when buffer is full or by timer
 */
void SDCardManager::flushDataBuffer()
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || m_dataBufferPos == 0)
    return;

  if (!m_currentDataPath)
  {
    startNewDataFile(nullptr);
  }

  // Already null-terminated by appendData
  appendToFile(m_currentDataPath, m_dataBuffer);

  // Clear buffer
  m_linesSaved += m_dataBufferPos;
  m_dataBuffer[0] = '\0';
  m_dataBufferPos = 0;
  m_lastFlushTime = millis();

  // Check if we need to start a new file
  if (m_linesSaved >= MAX_LINES_PER_FILE)
  {
    startNewDataFile(nullptr); // Only update internal path
  }
}


/**
 * @brief Reads configuration from SD card
 * @param config Pointer to store loaded configuration
 * @return true if config loaded successfully, false if error
 *
 * If config file doesn't exist, creates new one with default values
 * Verifies all config values are within valid ranges
 */
bool SDCardManager::readConfig(ConfigSettings *config)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || !config)
    return false;

  File file = SD.open(CONFIG_FILE_PATH, FILE_READ);
  if (!file)
  {
    // File doesn't exist, create with defaults
    *config = getDefaultConfig(); 
    return saveConfig(config);
  }

  char buf[192]; // Increased buffer for GPS coordinates
  size_t len = file.readBytesUntil('\n', buf, sizeof(buf) - 1);
  file.close();

  if (len == 0)
  {
    logStatus("Error: Empty config file");
    deleteFile(CONFIG_FILE_PATH);
    return false;
  }

  buf[len] = '\0';

  char lat_buf[32], lon_buf[32], elev_buf[16];
  int parsed = sscanf(buf, "%f,%f,%f,%hhu,%hhu,%f,%hhu,%f,%hhu,%f,%hhu,%[^,],%[^,],%s",
                      &config->start_m,
                      &config->end_m,
                      &config->update_rate,
                      &config->max_step_length,
                      &config->max_profile,
                      &config->signal_quality,
                      &config->reflector_shape,
                      &config->threshold_sensitivity,
                      &config->testing_update_rate,
                      &config->true_update_rate,
                      &config->text_width,
                      lat_buf,
                      lon_buf,
                      elev_buf);

  if (parsed != 14)
  {
    logStatus("Error: Failed to parse config file");
    deleteFile(CONFIG_FILE_PATH);
    return false;
  }

  strncpy(config->latitude, lat_buf, sizeof(config->latitude) - 1);
  strncpy(config->longitude, lon_buf, sizeof(config->longitude) - 1);
  strncpy(config->elevation, elev_buf, sizeof(config->elevation) - 1);
  config->latitude[sizeof(config->latitude) - 1] = '\0';
  config->longitude[sizeof(config->longitude) - 1] = '\0';
  config->elevation[sizeof(config->elevation) - 1] = '\0';

  return verifyConfig(config);
}


/**
 * @brief Saves configuration to SD card
 * @param config Pointer to configuration to save
 * @return true if config saved successfully, false if error
 *
 * Verifies config values before saving
 * Overwrites existing config file if present
 */
bool SDCardManager::saveConfig(const ConfigSettings *config)
{
  OperationGuard guard(m_operationInProgress);
  if (!m_isInitialized || !config)
    return false;

  if (!verifyConfig(config))
    return false;

  // Delete existing file if present
  if (SD.exists(CONFIG_FILE_PATH))
  {
    deleteFile(CONFIG_FILE_PATH);
  }

  char config_string[192]; // Increased size for GPS
  snprintf(config_string, sizeof(config_string),
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f,%d,%s,%s,%s\n",
           config->start_m,
           config->end_m,
           config->update_rate,
           config->max_step_length,
           config->max_profile,
           config->signal_quality,
           config->reflector_shape,
           config->threshold_sensitivity,
           config->testing_update_rate,
           config->true_update_rate,
           config->text_width,
           config->latitude,
           config->longitude,
           config->elevation);

  // Write new config
  return appendToFile(CONFIG_FILE_PATH, config_string);
}


/**
 * @brief Verifies configuration values are within valid ranges
 * @param config Pointer to configuration to verify
 * @return true if all values valid, false if any invalid
 */
bool SDCardManager::verifyConfig(const ConfigSettings *config)
{
  OperationGuard guard(m_operationInProgress);
  if (!config)
    return false;

  // Verify ranges for all numeric values
  if (config->start_m < 0.1f || config->start_m > 20.0f ||
      config->end_m < 0.1f || config->end_m > 20.0f ||
      config->update_rate < 0.1f || config->update_rate > 10.5f ||
      config->max_step_length < 1 || config->max_step_length > 99 ||
      config->max_profile < 1 || config->max_profile > 5 ||
      config->signal_quality < 0.0f || config->signal_quality > 35.0f ||
      config->reflector_shape > 1 ||
      config->threshold_sensitivity < 0.0f || config->threshold_sensitivity > 1.0f ||
      config->testing_update_rate > 1 ||
      config->true_update_rate < 0.0f || config->true_update_rate > 10.5f ||
      config->text_width > 140)
  {
    return false;
  }

  // Verify strings aren't empty and have reasonable lengths
  if (strlen(config->latitude) == 0 || strlen(config->latitude) >= sizeof(config->latitude) ||
      strlen(config->longitude) == 0 || strlen(config->longitude) >= sizeof(config->longitude) ||
      strlen(config->elevation) == 0 || strlen(config->elevation) >= sizeof(config->elevation))
  {
    return false;
  }

  return true;
}


/**
 * @brief Logs any input/output messages or debug statements to the debug log
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return none
 * 
 * Adds "SDCard:" prefix to all messages
 * Prints to Serial and queues for SD card if available
 */
void SDCardManager::logStatus(const char *format, ...)
{
  OperationGuard guard(m_operationInProgress);
  // Buffer for formatting the message
  char messageBuffer[MAX_LINE_LENGTH];

  // Handle variable arguments
  va_list args;
  va_start(args, format);
  vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
  va_end(args);

  // Print to Serial
  Serial.println(messageBuffer);

  // If we have a valid debug file, write directly
  if (m_isInitialized && m_currentDebugPath)
  {
    // Format with prefix
    char buffer[MAX_LINE_LENGTH];
    snprintf(buffer, sizeof(buffer), "SDCard: %s", messageBuffer);

    // Write directly to debug buffer/file
    appendDebug(buffer);
  }
}
