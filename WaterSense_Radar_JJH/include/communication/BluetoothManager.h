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
  bool sendMessageSTM32(const char *format, ...);
  void powerOff();
  void powerOn();
  bool isEnabled() const { return m_isEnabled; }
  void setTextWidth(uint8_t width) { m_textWidth = width; }
  uint8_t getTextWidth() const { return m_textWidth; }

private:
  BluetoothManager() : m_isEnabled(false),
                       m_messageQueue(nullptr),
                       m_textWidth(40) {}
  ~BluetoothManager();

  BluetoothManager(const BluetoothManager &) = delete;
  BluetoothManager &operator=(const BluetoothManager &) = delete;

  void flushReceiveBuffer();
  bool createMessageQueue();
  bool sendMessageInput(const char *format, ...);
  bool sendFormattedMessage(const char *prefix, const char *message, bool useDelay = true);
  bool receiveMessage(char *buffer, size_t bufferSize, uint32_t timeout = 0);
  void sendWithWrapping(const char *prefix, const char *text, bool useDelay);
  void logStatus(const char *message);

  // member variables
  BluetoothSerial m_serialBT;   // serial UART object
  bool m_isEnabled;             // is BT running?
  uint8_t m_ledPin;             // pin for BT status LED
  QueueHandle_t m_messageQueue; // queue for incoming (from user) messages
  uint8_t m_textWidth;          // width for text wrapping, character count

  // parameters
  static constexpr size_t QUEUE_SIZE = 10;                // max num of messages in queue
  static constexpr size_t MAX_MESSAGE_SIZE = 256;         // max size of individual messages
  static constexpr uint32_t BT_TIMEOUT = 100 * 60 * 1000;  // how long before BT turns off, in ms
  static constexpr uint32_t MESSAGE_DELAY = 100;          // delay in ms for each line printed to BT

  uint32_t m_lastActivityTime;
};