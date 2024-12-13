// src/communication/RadarManager.cpp
#include "communication/RadarManager.h"
#include "communication/BluetoothManager.h"
#include "storage/SDCardManager.h"
#include "storage/TimeManager.h"
#include <Arduino.h>
#include <stdarg.h>

bool RadarManager::initialize(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
{
  m_rxPin = rxPin;
  m_txPin = txPin;

  m_serial.begin(baudRate, SERIAL_8E1, rxPin, txPin);
  m_isActive = false;
  m_isTesting = false;
  m_noiseBlocking = false;
  m_testStartTime = 0;
  m_lastCommandTime = 0;

  // Load initial configuration
  m_currentConfig = SDCardManager::getInstance().getConfig();

  logStatus("Loaded configuration:");
  logStatus("Start distance: %.2f m", m_currentConfig.start_m);
  logStatus("End distance: %.2f m", m_currentConfig.end_m);
  logStatus("Update rate: %.1f Hz", m_currentConfig.update_rate);
  logStatus("Max step length: %d", m_currentConfig.max_step_length);
  logStatus("Max profile: %d", m_currentConfig.max_profile);
  logStatus("Signal quality: %.1f", m_currentConfig.signal_quality);
  logStatus("Reflector shape: %d", m_currentConfig.reflector_shape);
  logStatus("Threshold sensitivity: %.2f", m_currentConfig.threshold_sensitivity);
  logStatus("Testing update rate: %d", m_currentConfig.testing_update_rate);
  logStatus("True update rate: %.1f Hz", m_currentConfig.true_update_rate);

  // Clear any stale data
  while (m_serial.available())
  {
    m_serial.read();
  }

  logStatus("\nRadar initialized\n");
  return true;
}

void RadarManager::radarTask()
{
  while (true)  // Add infinite loop
  {
    if (m_serial.available())
    {
      processRadarData();
    }

    // Prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(5));  // Small delay between checks
  }
}

bool RadarManager::sendConfig(const ConfigSettings &config)
{
  m_currentConfig = config;

  // Format config string
  char configStr[64];
  snprintf(configStr, sizeof(configStr),
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f",
           config.start_m,
           config.end_m,
           config.update_rate,
           config.max_step_length,
           config.max_profile,
           config.signal_quality,
           config.reflector_shape,
           config.threshold_sensitivity,
           config.testing_update_rate,
           config.true_update_rate);

  logStatus("Sending config to STM...");

  if (!sendCommandWithData(RADAR_CMD_CONFIG_STRING, (uint8_t *)configStr, strlen(configStr)))
  {
    logStatus("Failed to send config string");
    return false;
  }

  // Wait for echo and validation
  uint32_t startTime = millis();
  while ((millis() - startTime) < CONFIG_TIMEOUT_MS)
  {
    if (m_serial.available() >= 4)
    {
      uint8_t response[MAX_DATA_SIZE];
      size_t len = m_serial.readBytesUntil(RADAR_NULL, response, sizeof(response));

      if (len > 0)
      {
        logStatus("Received from STM32: %s", response);

        // First validate the echo
        if (!validateConfigEcho(response, len))
        {
          logStatus("Config echo validation failed");
          return false;
        }

        // Then wait for success/fail response
        while ((millis() - startTime) < CONFIG_TIMEOUT_MS)
        {
          if (m_serial.available() >= 4)
          {
            uint8_t header[4];
            m_serial.readBytes(header, 4);

            if (header[0] == RADAR_HEADER_BYTE1 &&
                header[1] == RADAR_HEADER_BYTE2 &&
                header[3] == RADAR_NULL)
            {
              if (header[2] == RADAR_CMD_CONFIG_GOOD)
              {
                logStatus("Configuration accepted by STM32");
                return true;
              }
              else if (header[2] == RADAR_CMD_CONFIG_BAD)
              {
                logStatus("Configuration rejected by STM32");
                return false;
              }
            }
          }
          delay(1);
        }
      }
    }
    delay(1);
  }

  logStatus("Config timeout waiting for STM32 response");
  return false;
}

bool RadarManager::stopDataCollection()
{
  // Calculate delay based on update rate
  uint32_t delay_ms = (uint32_t)(3000.0f / m_currentConfig.update_rate) + 1000;

  if (!performStopSequence(delay_ms, STOP_TIMEOUT_MS))
  {
    return false;
  }

  m_isActive = false;
  logStatus("Stopped radar data collection");
  return true;
}

bool RadarManager::sendCommand(uint8_t cmd)
{
  uint8_t buf[4] = {RADAR_HEADER_BYTE1, RADAR_HEADER_BYTE2, cmd, RADAR_NULL};
  if (m_serial.write(buf, 4) != 4)
  {
    logStatus("Failed to send command: 0x%02X", cmd);
    return false;
  }
  m_lastCommandTime = millis();
  return true;
}

bool RadarManager::sendCommandWithData(uint8_t cmd, const uint8_t *data, size_t len)
{
  if (len > MAX_DATA_SIZE)
  {
    logStatus("Data too large to send");
    return false;
  }

  uint8_t buf[MAX_DATA_SIZE + 4];
  buf[0] = RADAR_HEADER_BYTE1;
  buf[1] = RADAR_HEADER_BYTE2;
  buf[2] = cmd;
  memcpy(&buf[3], data, len);
  buf[len + 3] = RADAR_NULL; // Put null terminator after the data

  if (m_serial.write(buf, len + 4) != (len + 4))
  {
    logStatus("Failed to send command with data: 0x%02X", cmd);
    return false;
  }
  m_lastCommandTime = millis();
  return true;
}

bool RadarManager::processRadarData()
{
  if (m_serial.available() < 3)
  {
    return false;
  }

  uint8_t header[3];
  m_serial.readBytes(header, 3);
  uint8_t nullByte;
  // if (!m_noiseBlocking)
  // {
  //   Serial.printf("Header received: %02X %02X %02X\n", header[0], header[1], header[2]);
  // }
  // Serial.printf("Header received: %02X %02X %02X\n", header[0], header[1], header[2]);

  if (header[0] != RADAR_HEADER_BYTE1 || header[1] != RADAR_HEADER_BYTE2)
  {
    if (!m_noiseBlocking)
    {
      logStatus("Invalid header received");
    }
    // Clear any stale data
    while (m_serial.available())
    {
      m_serial.read();
    }
    return false;
  }

  switch (header[2])
  {

  case RADAR_CMD_NEW_DATA:
  {
    uint8_t data[MAX_DATA_SIZE];
    size_t len = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < DEFAULT_TIMEOUT_MS && len < MAX_DATA_SIZE)
    {
      if (m_serial.available())
      {
        data[len] = m_serial.read();
        if (data[len] == RADAR_NULL)
        {
          handleDistanceData(data, len);
          return true;
        }
        len++;
      }
    }
    logStatus("Timeout or buffer overflow reading distance data");
    break;
  }

  case RADAR_CMD_REQUEST_CONFIG:
  {
    // Read the null terminator separately
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      logStatus("Config requested by STM32");
      return sendConfig(m_currentConfig);
    }
    break;
  }

  case RADAR_CMD_START_DATA:
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      logStatus("Starting data collection");
      SDCardManager::getInstance().requestNewDataFile();
      m_isActive = true;
      return true;
    }
    break;

  case RADAR_CMD_START_TEST:
  {
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      m_testStartTime = millis();
      logStatus("Update rate test started");
      return true;
    }
    break;
  }
  case RADAR_CMD_END_TEST:
  {
    uint32_t startTime = millis();
    char numStr[4] = {0}; // Buffer for up to 3 digits plus null
    size_t pos = 0;

    // Read digits until non-digit or timeout
    while (pos < 3 && (millis() - startTime) < DEFAULT_TIMEOUT_MS)
    {
      if (m_serial.available())
      {
        char c = m_serial.read();
        if (!isdigit(c))
        {
          break;
        }
        numStr[pos++] = c;
      }
    }

    if (pos > 0)
    {
      uint8_t count = atoi(numStr);
      uint32_t elapsed = millis() - m_testStartTime;
      float actualRate = calculateUpdateRate(count, elapsed);
      m_isTesting = false;
      logStatus("Update rate test complete: %d samples, %.1f Hz", count, actualRate);
      return true;
    }
    break;
  }

  case RADAR_CMD_STOP_REQUEST:
  {
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      logStatus("Received stop request from STM32, sending confirmation");

      // Send confirmation back to STM32
      if (sendCommand(RADAR_CMD_STOP_CONFIRM))
      {
        m_isActive = false;
        return true;
      }
      else
      {
        logStatus("Failed to send stop confirmation");
        return false;
      }
    }
    break;
  }

  case RADAR_CMD_NOISE_ON:
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      m_noiseBlocking = true;
      return true;
    }
    break;

  case RADAR_CMD_NOISE_OFF:
    if (!m_serial.available())
    {
      return false;
    }
    nullByte = m_serial.read();

    if (nullByte == RADAR_NULL)
    {
      m_noiseBlocking = false;
      m_measurementInProgress = true;
      return true;
    }
    break;

  case RADAR_CMD_DEBUG_MSG:
  {
    // Buffer for debug message including null terminator
    uint8_t data[MAX_DATA_SIZE];
    size_t len = 0;
    uint32_t startTime = millis();

    // Read until null terminator or timeout
    while ((millis() - startTime) < DEFAULT_TIMEOUT_MS && len < MAX_DATA_SIZE - 1)
    {
      if (m_serial.available())
      {
        data[len] = m_serial.read();
        if (data[len] == RADAR_NULL) // Found null terminator
        {
          // Explicitly null terminate the string
          data[len] = '\0';
          // Log the debug message from STM32
          logStatus("STM32 Debug: %s", data);
          return true;
        }
        len++;
      }
    }
    logStatus("Timeout or buffer overflow reading debug message");
    break;
  }

  default:
    if (!m_noiseBlocking)
    {
      logStatus("Unknown command received: 0x%02X", header[2]);
    }
    // Clear any stale data
    while (m_serial.available())
    {
      m_serial.read();
    }
    break;
  }

  return false;
}

void RadarManager::handleDistanceData(const uint8_t *data, size_t len)
{
  // Get timestamp
  char timestamp[32];
  TimeManager::getInstance().getFormattedTimestamp(timestamp, sizeof(timestamp));

  // Make a null-terminated copy of the data
  char dataStr[MAX_DATA_SIZE];
  memcpy(dataStr, data, len);
  dataStr[len] = '\0';

  if (len == 0 || dataStr[0] == '\0')
  {
    // For empty data, print "no_dists" with timestamp to Serial and BT only
    char noDistStr[64];
    snprintf(noDistStr, sizeof(noDistStr), "%s no_dists", timestamp);

    SDCardManager::getInstance().queueData("%s", noDistStr);

    // Check if enough time has passed to print again
    uint32_t currentTime = millis();
    if ((currentTime - m_lastPrintTime) >= MIN_PRINT_INTERVAL_MS)
    {
      Serial.println(noDistStr);
      BluetoothManager::getInstance().sendWithWrapping("", noDistStr, false);
      m_lastPrintTime = currentTime;
    }

    m_measurementInProgress = false;
    return;
  }

  // For valid data, include timestamp and save to all outputs
  char fullStr[MAX_DATA_SIZE + 32]; // Add space for timestamp
  snprintf(fullStr, sizeof(fullStr), "%s %s", timestamp, dataStr);

  // Always log the data
  SDCardManager::getInstance().queueData("%s", fullStr);

  // Check if enough time has passed to print again
  uint32_t currentTime = millis();
  if ((currentTime - m_lastPrintTime) >= MIN_PRINT_INTERVAL_MS)
  {
    Serial.println(fullStr);
    BluetoothManager::getInstance().sendWithWrapping("", fullStr, false);
    m_lastPrintTime = currentTime;
  }
  
  m_measurementInProgress = false;
}

void RadarManager::logStatus(const char *format, ...)
{
  char messageBuffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
  va_end(args);

  // Print to Serial for debugging
  Serial.println(messageBuffer);

  // Queue for SD card logging
  SDCardManager::getInstance().queueDebug("Radar: %s", messageBuffer);
}

bool RadarManager::performStopSequence(uint32_t delay_ms, uint32_t timeout_ms)
{
  // Log to both status and Bluetooth
  logStatus("Stopping radar... (%.1f second delay)", delay_ms / 1000.0f);
  BluetoothManager::getInstance().sendMessageESP32("Stopping radar... (%.1f second delay)", delay_ms / 1000.0f);

  // Disable UART
  uart_driver_delete(UART_NUM_2);

  // Configure TX pin as output and set LOW
  gpio_set_direction((gpio_num_t)m_txPin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)m_txPin, 0);

  // Wait for calculated delay
  uint32_t startTime = millis();
  while ((millis() - startTime) < delay_ms)
    ;

  // Restart UART
  m_serial.begin(921600, SERIAL_8E1, m_rxPin, m_txPin);

  // Wait for stop acknowledgment
  startTime = millis();

  while ((millis() - startTime) < timeout_ms)
  {
    if (m_serial.available() >= 4)
    {
      uint8_t response[4];
      m_serial.readBytes(response, 4);

      if (response[0] == RADAR_HEADER_BYTE1 &&
          response[1] == RADAR_HEADER_BYTE2 &&
          response[2] == RADAR_CMD_STOP_REQUEST &&
          response[3] == RADAR_NULL)
      {

        // Send confirmation
        return sendCommand(RADAR_CMD_STOP_CONFIRM);
      }
    }
    delay(1);
  }

  logStatus("Stop sequence timeout");
  return false;
}

float RadarManager::calculateUpdateRate(uint8_t count, uint32_t elapsed_ms)
{
  if (count >= 0 && count <= 251)
  {
    return (float)count * 1000.0f / (float)elapsed_ms;
  }
  logStatus("Invalid sample count for update rate calculation");
  return 0.0f;
}

bool RadarManager::validateConfigEcho(const uint8_t *received, size_t len)
{
  // Check minimum length for doubled header
  if (len < 7) // "O:O:$" is 5 chars plus at least 1 data char plus null
  {
    logStatus("Echo too short");
    return false;
  }

  // Check for doubled header "O:O:$"
  if (received[0] != RADAR_HEADER_BYTE1 ||
      received[1] != RADAR_HEADER_BYTE2 ||
      received[2] != RADAR_HEADER_BYTE1 ||
      received[3] != RADAR_HEADER_BYTE2 ||
      received[4] != RADAR_CMD_CONFIG_STRING)
  {
    logStatus("Invalid echo header");
    return false;
  }

  // Format the expected config string
  char expectedStr[64];
  snprintf(expectedStr, sizeof(expectedStr),
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f",
           m_currentConfig.start_m,
           m_currentConfig.end_m,
           m_currentConfig.update_rate,
           m_currentConfig.max_step_length,
           m_currentConfig.max_profile,
           m_currentConfig.signal_quality,
           m_currentConfig.reflector_shape,
           m_currentConfig.threshold_sensitivity,
           m_currentConfig.testing_update_rate,
           m_currentConfig.true_update_rate);

  // Compare the config strings (skip doubled header + cmd bytes)
  if (len < (strlen(expectedStr) + 5))
  {
    logStatus("Config echo too short");
    return false;
  }

  // Compare starting after the doubled header (O:O:$)
  if (memcmp(received + 5, expectedStr, strlen(expectedStr)) != 0)
  {
    logStatus("Config echo content mismatch");
    logStatus("Expected: %s", expectedStr);
    logStatus("Received: %.*s", len - 5, received + 5);
    return false;
  }

  logStatus("Config echo validated successfully");
  return true;
}
