// include/communication/BluetoothManager.h
#pragma once

#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string>

class BluetoothManager
{
public:
  static BluetoothManager &getInstance()
  {
    static BluetoothManager instance;
    return instance;
  }

  bool initialize(const char *deviceName, uint8_t ledPin);
  void bluetoothTask();
  bool sendMessage(const char *format, ...);
  bool sendFormattedMessage(const char *prefix, const char *message, bool useDelay = true);
  bool receiveMessage(char *buffer, size_t bufferSize, uint32_t timeout = 0);
  void disable();
  void enable();
  bool isEnabled() const { return m_isEnabled; }
  void setTextWidth(uint8_t width) { m_textWidth = width; }
  uint8_t getTextWidth() const { return m_textWidth; }

private:
  BluetoothManager() : m_isEnabled(false),
                       m_messageQueue(nullptr),
                       m_textWidth(40),
                       m_taskHandle(nullptr) {}
  ~BluetoothManager();

  BluetoothManager(const BluetoothManager &) = delete;
  BluetoothManager &operator=(const BluetoothManager &) = delete;

  // Internal methods
  void flushReceiveBuffer();
  bool createMessageQueue();
  void sendWithWrapping(const char *prefix, const char *text, bool useDelay);
  void logStatus(const char *message);

  BluetoothSerial m_serialBT;
  bool m_isEnabled;
  uint8_t m_ledPin;
  QueueHandle_t m_messageQueue;
  uint8_t m_textWidth;
  TaskHandle_t m_taskHandle;

  static constexpr size_t QUEUE_SIZE = 10;
  static constexpr size_t MAX_MESSAGE_SIZE = 256;
  static constexpr uint32_t BT_TIMEOUT = 1 * 60 * 1000;
  static constexpr uint32_t MESSAGE_DELAY = 100;
  static constexpr uint32_t TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t TASK_PRIORITY = 2;
  static constexpr BaseType_t CORE_ID = 0;

  uint32_t m_lastActivityTime;
};