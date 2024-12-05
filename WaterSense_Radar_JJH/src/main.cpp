// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "communication/BluetoothManager.h"
#include "communication/GPSManager.h"
#include "storage/SDCardManager.h"
#include "RTClib.h"

#define DEVICE_NAME "ESP32_HUZZAH32"
#define LED_PIN LED_BUILTIN
#define GPS_RX_PIN 32
#define GPS_TX_PIN 14
#define GPS_POWER_PIN 15
#define CHIP_SELECT_PIN 33

// Function declarations
void monitorGPSData();
void monitorBluetoothData();
void createInitialFiles();
void logTimestampedData();

// Global file path pointers
char *debugFilePath = nullptr;
char *dataFilePath = nullptr;

RTC_PCF8523 rtc; // Create RTC object

void setup()
{
  Serial.begin(115200);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1)
      delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower())
  {
    Serial.println("RTC is NOT initialized, setting time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize managers
  GPSManager::getInstance().initialize(GPS_RX_PIN, GPS_TX_PIN, GPS_POWER_PIN, rtc);
  GPSManager::getInstance().setDebugMode(false);
  Serial.println("GPS initialized and powered on");

  BluetoothManager::getInstance().initialize(DEVICE_NAME, LED_PIN);
  Serial.println("Bluetooth initialized");

  if (!SDCardManager::getInstance().initialize(CHIP_SELECT_PIN, rtc))
  {
    Serial.println("SD Card initialization failed!");
    while (1)
      delay(10);
  }
  Serial.println("SD Card initialized");

  // Create initial log files
  createInitialFiles();

  // Create GPS task
  xTaskCreatePinnedToCore(
      [](void *parameter)
      {
        GPSManager::getInstance().gpsTask();
      },
      "gps_task",
      4096,    // Stack size
      nullptr, // Parameters
      1,       // Priority
      nullptr, // Task handle
      1        // Core ID
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

  Serial.println("Setup complete");
}

void loop()
{
  monitorGPSData();
  monitorBluetoothData();
  logTimestampedData();
  vTaskDelay(pdMS_TO_TICKS(100));
}

void createInitialFiles()
{
  // Create new debug file
  if (!SDCardManager::getInstance().startNewDebugFile(&debugFilePath))
  {
    Serial.println("Failed to create debug file!");
    return;
  }
  Serial.printf("Created debug file: %s\n", debugFilePath);

  // Create new data file
  if (!SDCardManager::getInstance().startNewDataFile(&dataFilePath))
  {
    Serial.println("Failed to create data file!");
    return;
  }
  Serial.printf("Created data file: %s\n", dataFilePath);

  // Write initial header to data file
  DateTime now = rtc.now();
  SDCardManager::getInstance().appendData("First Data File Since Powering On: True");
  SDCardManager::getInstance().appendData("Data File: %s", dataFilePath);
  SDCardManager::getInstance().appendData("Start Time: [%02d/%02d/%02d %02d:%02d:%02d.%03u]",
                                          now.year() % 100, now.month(), now.day(),
                                          now.hour(), now.minute(), now.second(),
                                          (millis() % 1000));
  SDCardManager::getInstance().appendData("Location: Not Set Not Set");
  SDCardManager::getInstance().appendData("Elevation: Not Set");
  SDCardManager::getInstance().appendData("---");
  SDCardManager::getInstance().flushDataBuffer(); // Force write to SD card
}

void monitorBluetoothData()
{
  static char buffer[256];

  if (BluetoothManager::getInstance().receiveMessage(buffer, sizeof(buffer), 0))
  {
    // Log received message to debug file
    SDCardManager::getInstance().appendDebug("BT Received: %s", buffer);
    Serial.println("Appended to debug!");
  }
}

void logTimestampedData()
{
  static uint32_t lastLogTime = 0;
  const uint32_t LOG_INTERVAL = 1000; // Log every second

  if (millis() - lastLogTime >= LOG_INTERVAL)
  {
    DateTime now = rtc.now();
    SDCardManager::getInstance().appendData("[%02d/%02d/%02d %02d:%02d:%02d.%03u] 0.0m",
                                            now.year() % 100, now.month(), now.day(),
                                            now.hour(), now.minute(), now.second(),
                                            (millis() % 1000));
    lastLogTime = millis();
    Serial.println("Appended to data!");
  }
}

void monitorGPSData()
{
  static bool fixReported = false;

  if (GPSManager::getInstance().hasValidFix())
  {
    if (!fixReported)
    {
      Serial.println("GPS Fix obtained!");
      BluetoothManager::getInstance().sendMessage("GPS Fix obtained!");
      SDCardManager::getInstance().appendDebug("GPS Fix obtained!");
      fixReported = true;
    }

    const GPSData &gpsData = GPSManager::getInstance().getCurrentLocation();

    // Format time as HH:MM:SS
    char timeStr[9];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             gpsData.hour, gpsData.minute, gpsData.second);

    // Log GPS data
    SDCardManager::getInstance().appendDebug("GPS Time: %s PST", timeStr);
    SDCardManager::getInstance().appendDebug("GPS Location: %s %s",
                                             gpsData.latitude, gpsData.longitude);
    SDCardManager::getInstance().appendDebug("GPS Elevation: %s", gpsData.elevation);

    // Send to Bluetooth
    BluetoothManager::getInstance().sendMessage("Time: %s PST", timeStr);
    BluetoothManager::getInstance().sendMessage("Location: %s %s",
                                                gpsData.latitude, gpsData.longitude);
    BluetoothManager::getInstance().sendMessage("Elevation: %s",
                                                gpsData.elevation);

    // Synchronize RTC if needed
    GPSManager::getInstance().syncRTCWithGPS();
    GPSManager::getInstance().powerOff();
  }
}