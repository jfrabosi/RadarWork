// src/communication/BluetoothManager.cpp
#include "communication/BluetoothManager.h"
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

bool BluetoothManager::initialize(const char *deviceName, uint8_t ledPin)
{
  // turn LED on while BT is on
  m_ledPin = ledPin;
  pinMode(m_ledPin, OUTPUT);
  digitalWrite(m_ledPin, HIGH);

  if (!m_serialBT.begin(deviceName))
  {
    logStatus("Failed to initialize Bluetooth");
    return false;
  }
  
  m_isEnabled = true;
  m_lastActivityTime = millis();
  logStatus("Bluetooth initialized successfully");
  return createMessageQueue();
}

void BluetoothManager::bluetoothTask()
{
  char receiveBuffer[MAX_MESSAGE_SIZE];
  static bool set = false;

  while (true)
  {
    if (m_isEnabled)
    {
      // Check for Bluetooth timeout
      if ((millis() - m_lastActivityTime) > BT_TIMEOUT)
      {
        disable();
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      // Handle incoming messages
      if (m_serialBT.available())
      {
        m_lastActivityTime = millis();
        size_t bytesRead = m_serialBT.readBytesUntil('\n', receiveBuffer, MAX_MESSAGE_SIZE - 1);
        if (bytesRead > 0)
        {
          receiveBuffer[bytesRead] = '\0';
          xQueueSend(m_messageQueue, receiveBuffer, pdMS_TO_TICKS(100));

          // Process the message immediately
          Serial.printf("Received: %s\n", receiveBuffer);
          sendMessage("Received: %s", receiveBuffer);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool BluetoothManager::sendMessage(const char* format, ...) {
  if (!m_isEnabled) {
    return false;
  }

  char buffer[MAX_MESSAGE_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // Use the new formatted message sender with "ESP32:" prefix
  return sendFormattedMessage("ESP32:", buffer);
}

bool BluetoothManager::sendFormattedMessage(const char* prefix, const char* message, bool useDelay) {
  if (!m_isEnabled) {
    return false;
  }

  m_lastActivityTime = millis();
  sendWithWrapping(prefix, message, useDelay);
  return true;
}

bool BluetoothManager::receiveMessage(char* buffer, size_t bufferSize, uint32_t timeout) {
  if (!m_isEnabled || !buffer || bufferSize == 0) {
    return false;
  }

  return xQueueReceive(m_messageQueue, buffer, pdMS_TO_TICKS(timeout)) == pdTRUE;
}

void BluetoothManager::disable() {
  if (m_isEnabled) {
    m_serialBT.flush();
    m_serialBT.end();
    m_isEnabled = false;
    logStatus("Bluetooth powering down.");
    digitalWrite(m_ledPin, LOW);
  }
}

void BluetoothManager::enable() {
  if (!m_isEnabled) {
    m_serialBT.begin("ESP32_Device");
    m_isEnabled = true;
    m_lastActivityTime = millis();
    digitalWrite(m_ledPin, HIGH);
  }
}

BluetoothManager::~BluetoothManager() {
  if (m_messageQueue) {
    vQueueDelete(m_messageQueue);
  }
  disable();
}

void BluetoothManager::flushReceiveBuffer() {
  while (m_serialBT.available()) {
    m_serialBT.read();
  }
}

bool BluetoothManager::createMessageQueue() {
  if (m_messageQueue) {
    vQueueDelete(m_messageQueue);
  }
  
  m_messageQueue = xQueueCreate(QUEUE_SIZE, MAX_MESSAGE_SIZE);
  return m_messageQueue != nullptr;
}

void BluetoothManager::sendWithWrapping(const char* prefix, const char* text, bool useDelay) {
  if (m_textWidth == 0) {
    // No text wrapping
    m_serialBT.print(prefix);
    m_serialBT.print(" ");
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
      m_serialBT.print(" ");
    } else {
      // For continuation lines, add spaces to align with first line
      m_serialBT.print(prefix);
      m_serialBT.print("    ");
    }

    // Calculate available width for text
    size_t availWidth = firstLine ? 
      m_textWidth - prefixLen - 1 : // -1 for the space after prefix
      m_textWidth - prefixLen - 4;  // -4 for the extra spacing on continuation

    // Find where to break the line
    size_t end = start + availWidth;
    if (end > textLen) {
      end = textLen;
    }

    // Write the line segment
    m_serialBT.write(reinterpret_cast<const uint8_t*>(text + start), end - start);
    m_serialBT.println();

    if (useDelay) {
      delay(MESSAGE_DELAY);
    }

    start = end;
    firstLine = false;
  }

  // If we didn't output anything (empty string), still print the prefix
  if (textLen == 0) {
    m_serialBT.print(prefix);
    m_serialBT.println();
    if (useDelay) {
      delay(MESSAGE_DELAY);
    }
  }
}

void BluetoothManager::logStatus(const char *message)
{
  Serial.println(message);
}