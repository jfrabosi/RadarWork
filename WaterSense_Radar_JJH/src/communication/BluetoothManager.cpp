// src/communication/BluetoothManager.cpp
#include "communication/BluetoothManager.h"
#include "storage/SDCardManager.h"
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>


/**
 * @brief Initializes the Bluetooth Manager
 * @param deviceName const char* like "ESP32-HUZZAH32", BT access name
 * @param ledPin Pin number for BT status LED (usually LED_BUILTIN)
 * @return true for successful initialization, false if error occurred
 */
bool BluetoothManager::initialize(const char *deviceName, uint8_t ledPin)
{
  // keep LED on while BT is on - initialize here
  m_ledPin = ledPin;
  pinMode(m_ledPin, OUTPUT);
  digitalWrite(m_ledPin, HIGH);

  // start BT serial
  if (!m_serialBT.begin(deviceName))
  {
    logStatus("DEBUG: Failed to initialize Bluetooth");
    return false;
  }
  
  m_isEnabled = true;
  m_lastActivityTime = millis();
  logStatus("Bluetooth initialized\n");

  // create message queue for received messages
  return createMessageQueue();
}


/**
 * @brief Main task for Bluetooth Manager
 * @return none
 *
 * BTTask has two main functions:
 * - Check if timeout has been exceeded (power-saving)
 * - Check if any messages have been received from the phone/laptop BT connection
 *
 * Sending messages is handled by each task, using sendMessage or sendMessageSTM32
 * 
 * Any messages received by BT or sent through BT are saved to the debug log automatically
 */
void BluetoothManager::bluetoothTask()
{
  char receiveBuffer[MAX_MESSAGE_SIZE];

  while (true)
  {
    if (m_isEnabled)
    {
      // check for Bluetooth timeout
      if ((millis() - m_lastActivityTime) > BT_TIMEOUT)
      {
        powerOff();
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      // handle incoming messages
      if (m_serialBT.available())
      {
        m_lastActivityTime = millis();
        size_t bytesRead = m_serialBT.readBytesUntil('\n', receiveBuffer, MAX_MESSAGE_SIZE - 1);
        if (bytesRead > 0)
        {
          receiveBuffer[bytesRead] = '\0';
          xQueueSend(m_messageQueue, receiveBuffer, pdMS_TO_TICKS(100));

          // print the received message to the terminal
          sendMessageESP32("%s", receiveBuffer);
        }
      }
    }

    // prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


/**
 * @brief Sends a message to the BT terminal with prefix "STM32: "
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return true for successful print, false if error occurred
 * 
 * Use this function for any messages coming from the STM32/radar
 */
bool BluetoothManager::sendMessageSTM32(const char *format, ...)
{
  if (!m_isEnabled)
  {
    return false;
  }

  char buffer[MAX_MESSAGE_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  return sendFormattedMessage("STM32: ", buffer);
}


/**
 * @brief Sends a message to the BT terminal with prefix "ESP32: "
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return true for successful print, false if error occurred
 *
 * Use this function for the majority of messages.
 */
bool BluetoothManager::sendMessageESP32(const char *format, ...)
{
  if (!m_isEnabled)
  {
    return false;
  }

  char buffer[MAX_MESSAGE_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  return sendFormattedMessage("ESP32: ", buffer);
}


/**
 * @brief Disables / turns off the Bluetooth module
 * @return none
 * 
 * Turns off automatically after timeout is exceeded - configure in BluetoothManager.h
 */
void BluetoothManager::powerOff() {
  if (m_isEnabled) {
    m_serialBT.flush();
    m_serialBT.end();
    m_isEnabled = false;
    logStatus("DEBUG: Bluetooth powered down.");
    digitalWrite(m_ledPin, LOW);
  }
}


/**
 * @brief Enables / turns on the Bluetooth module
 * @return none
 */
void BluetoothManager::powerOn() {
  if (!m_isEnabled) {
    m_serialBT.begin("ESP32_Device");
    m_isEnabled = true;
    m_lastActivityTime = millis();
    logStatus("DEBUG: Bluetooth powered up.");
    digitalWrite(m_ledPin, HIGH);
  }
}


/**
 * @brief Destructor, destroys the Bluetooth Manager
 * @return none
 */
BluetoothManager::~BluetoothManager() {
  if (m_messageQueue) {
    vQueueDelete(m_messageQueue);
  }
  powerOff();
}


/**
 * @brief Flushes / clears the Bluetooth UART buffer
 * @return none
 */
void BluetoothManager::flushReceiveBuffer() {
  while (m_serialBT.available()) {
    m_serialBT.read();
  }
}


/**
 * @brief Creates a message queue for received messages
 * @return none
 */
bool BluetoothManager::createMessageQueue() {
  if (m_messageQueue) {
    vQueueDelete(m_messageQueue);
  }
  
  m_messageQueue = xQueueCreate(QUEUE_SIZE, MAX_MESSAGE_SIZE);
  return m_messageQueue != nullptr;
}


/**
 * @brief Sends a message to the BT terminal with prefix "Input: "
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return true for successful print, false if error occurred
 *
 * Use this function for any user inputs
 */
bool BluetoothManager::sendMessageInput(const char *format, ...)
{
  if (!m_isEnabled)
  {
    return false;
  }

  char buffer[MAX_MESSAGE_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  return sendFormattedMessage("Input:", buffer);
}


/**
 * @brief Wrapper for sendWithWrapping, with status logging
 * @param prefix const char* like "ESP32:", sends before message in same line
 * @param message const char* like "Blessed are the cheesemakers."
 * @param useDelay true if lines should be send with delay - looks better this way
 * @return true for successful print, false if error occurred
 */
bool BluetoothManager::sendFormattedMessage(const char *prefix, const char *message, bool useDelay)
{
  if (!m_isEnabled)
  {
    return false;
  }

  // create a buffer for the combined message
  char logBuffer[MAX_MESSAGE_SIZE];
  snprintf(logBuffer, sizeof(logBuffer), "%s %s", prefix, message);

  // log the full message
  logStatus(logBuffer);

  sendWithWrapping(prefix, message, useDelay);
  return true;
}


/**
 * @brief Puts received messages into a queue
 * @param buffer const char* like "s01.20"
 * @return true if message is put in queue correctly, false otherwise
 */
bool BluetoothManager::receiveMessage(char *buffer, size_t bufferSize, uint32_t timeout)
{
  if (!m_isEnabled || !buffer || bufferSize == 0)
  {
    return false;
  }

  return xQueueReceive(m_messageQueue, buffer, pdMS_TO_TICKS(timeout)) == pdTRUE;
}


/**
 * @brief Sends a message over BT serial with wrapping (fit to device screen / window)
 * @param prefix const char* like "ESP32:", sends before message in same line
 * @param text const char* like "Blessed are the cheesemakers."
 * @param useDelay true if lines should be send with delay - looks better this way
 * @return none
 */
void BluetoothManager::sendWithWrapping(const char* prefix, const char* text, bool useDelay) {
  if (m_textWidth == 0) {
    // no text wrapping
    m_serialBT.print(prefix);
    m_serialBT.println(text);
    if (useDelay) {
        delay(MESSAGE_DELAY);
    }
    return;
  }

  size_t prefixLen = strlen(prefix);
  size_t textLen = strlen(text);
  size_t start = 0;
  bool firstLine = true;

  while (start < textLen) {
    if (firstLine) {
      m_serialBT.print(prefix);
    } else {
      // for continuation lines, add spaces to align with first line
      // two more spaces for "indentation"
      m_serialBT.print(prefix);
      m_serialBT.print("  ");
    }

    // calculate available width for text
    size_t availWidth = firstLine ? 
      m_textWidth - prefixLen : // 
      m_textWidth - prefixLen - 2;  // -2 for the extra spacing on continuation

    // find where to break the line
    size_t end = start + availWidth;
    if (end > textLen) {
      end = textLen;
    }

    // write the line segment
    m_serialBT.write(reinterpret_cast<const uint8_t*>(text + start), end - start);
    m_serialBT.println();

    if (useDelay) {
      delay(MESSAGE_DELAY);
    }

    start = end;
    firstLine = false;
  }

  // if we didn't output anything (empty string), still print the prefix
  if (textLen == 0) {
    m_serialBT.print(prefix);
    m_serialBT.println();
    if (useDelay) {
      delay(MESSAGE_DELAY);
    }
  }
}


/**
 * @brief Logs any input/output messages or debug statements to the debug log
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return none
 *
 * Adds "BT:" prefix to all messages
 * Prints to Serial and queues for SD card if available
 */
void BluetoothManager::logStatus(const char *format, ...)
{
  // Buffer for formatting the message
  char messageBuffer[MAX_MESSAGE_SIZE];

  // Handle variable arguments
  va_list args;
  va_start(args, format);
  vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
  va_end(args);

  // Print to Serial
  Serial.println(messageBuffer);

  // Queue for SD card if available
  SDCardManager::getInstance().queueDebug("BT: %s", messageBuffer);
}
