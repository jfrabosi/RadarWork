// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "communication/BluetoothManager.h"
#include "communication/GPSManager.h"
#include "storage/SDCardManager.h"
#include "storage/TimeManager.h"
#include "communication/RadarManager.h"
#include "RTClib.h"
#include "driver/uart.h"

#define DEVICE_NAME "ESP32_HUZZAH32"
#define LED_PIN LED_BUILTIN
#define GPS_RX_PIN 32
#define GPS_TX_PIN 14
#define GPS_POWER_PIN 15
#define CHIP_SELECT_PIN 33
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

// Function declarations

// Global file path pointers
char *debugFilePath = nullptr;
char *dataFilePath = nullptr;

RTC_PCF8523 rtc; // Create RTC object

void setup()
{
  Serial.begin(115200);

  // set up RTC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }

  // initialize managers
  if (!TimeManager::getInstance().initialize(rtc))
  {
    Serial.println("Time Manager initialization failed!");
    while (1)
      delay(10);
  }

  if (!SDCardManager::getInstance().initialize(CHIP_SELECT_PIN, rtc))
  {
    Serial.println("SD Card initialization failed!");
    while (1)
      delay(10);
  }

  // Get current configuration
  ConfigSettings currentConfig = SDCardManager::getInstance().getConfig();

  // Update the update rate
  currentConfig.update_rate = 20.0f;
  currentConfig.start_m = 1.0f;
  currentConfig.end_m = 1.10f;
  currentConfig.max_step_length = 2;

  // Save the modified configuration
  SDCardManager::getInstance().updateConfig(currentConfig);

  BluetoothManager::getInstance().initialize(DEVICE_NAME, LED_PIN);

  GPSManager::getInstance().initialize(GPS_RX_PIN, GPS_TX_PIN, GPS_POWER_PIN, rtc);

  // Initialize RadarManager
  if (!RadarManager::getInstance().initialize(RADAR_RX_PIN, RADAR_TX_PIN))
  {
    Serial.println("Radar initialization failed!");
    while (1)
      delay(10);
  }

  // Configure sleep parameters
  esp_sleep_enable_uart_wakeup(UART_NUM_2); // Wake on STM32 UART
  esp_sleep_enable_timer_wakeup(100000);    // Wake every 100ms to check tasks

  // Configure UART for wake capability
  uart_set_wakeup_threshold(UART_NUM_2, 3); // Wake after 3 bytes received

  // // Set initial date/time - for December 11, 2024
  // TimeManager::getInstance().setDateTime(
  //     2024, // year
  //     12,   // month (December)
  //     11,   // day
  //     1,    // hour (midnight UTC)
  //     0,    // minute
  //     0     // second
  // );
  // Serial.println("Time Manager initialized and date set");

  // Create SD Card task
  xTaskCreatePinnedToCore(
      [](void *parameter)
      {
        SDCardManager::getInstance().sdTask();
      },
      "sd_card_task",
      4096,    // Stack size
      nullptr, // Parameters
      3,       // Priority
      nullptr, // Task handle
      0        // Core ID (same as GPS)
  );

  // Create Bluetooth task
  xTaskCreatePinnedToCore(
      [](void *parameter)
      {
        BluetoothManager::getInstance().bluetoothTask();
      },
      "bluetooth_task",
      4096,    // Stack size
      nullptr, // Task parameters
      2,       // Priority
      nullptr, // Task handle
      0        // Core ID
  );

  // Create GPS task
  xTaskCreatePinnedToCore(
      [](void *parameter)
      {
        GPSManager::getInstance().gpsTask();
      },
      "gps_task",
      4096,    // Stack size
      nullptr, // Parameters
      2,       // Priority
      nullptr, // Task handle
      1        // Core ID
  );

  // Create Radar task - high priority since timing is critical
  xTaskCreatePinnedToCore(
      [](void *parameter)
      {
        RadarManager::getInstance().radarTask();
      },
      "radar_task",
      4096,    // Stack size
      nullptr, // Parameters
      3,       // Priority (highest)
      nullptr, // Task handle
      1        // Core ID (same core as GPS)
  );
}

void loop()
{
  // // Check if we can sleep
  // bool canSleep = true;

  // // Don't sleep if BT is active
  // if (BluetoothManager::getInstance().isEnabled())
  // {
  //   canSleep = false;
  // }

  // // Don't sleep if GPS is active
  // if (GPSManager::getInstance().isEnabled())
  // {
  //   canSleep = false;
  // }

  // // Don't sleep if radar is testing or in middle of measurement
  // if (RadarManager::getInstance().isTesting() ||
  //     RadarManager::getInstance().isMeasuring())
  // {
  //   canSleep = false;
  // }

  // // Don't sleep if SD card has pending operations
  // if (SDCardManager::getInstance().hasPendingOperations())
  // {
  //   canSleep = false;
  // }

  // if (canSleep)
  // {
  //   // Enter light sleep
  //   esp_light_sleep_start();
  // }

  // Regular task processing
  vTaskDelay(pdMS_TO_TICKS(1000));
}