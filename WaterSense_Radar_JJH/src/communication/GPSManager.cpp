// src/communication/GPSManager.cpp
#include "communication/GPSManager.h"
#include "communication/GPSManager.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


/**
 * @brief Initializes the GPS Manager
 * @param rxPin Pin number for RX pin on ESP32 (connects to GPS TX)
 * @param txPin Pin number for TX pin on ESP32 (connects to GPS RX)
 * @param powerPin Pin number for MOSFET pin on ESP32 (MOSFET connects GPS GND to ESP32 GND)
 * @param rtcRef Pointer to RTC object
 * @return true for successful initialization, false if error occurred
 */
bool GPSManager::initialize(uint8_t rxPin, uint8_t txPin, uint8_t powerPin, RTC_PCF8523& rtcRef) {
  m_powerPin = powerPin;
  m_pRTC = &rtcRef;  // store reference to RTC
  m_pBT = &BluetoothManager::getInstance(); // store reference to BT manager

  pinMode(m_powerPin, OUTPUT);
  digitalWrite(m_powerPin, HIGH);  // start with GPS powered on

  m_gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  m_isEnabled = true;
  m_lastGPSTime = millis();

  return true;
}


/**
 * @brief Main task for GPS Manager
 * @return none
 *
 * GPSTask has three main functions:
 * - Check if timeout has been exceeded (power-saving)
 * - Check if any NMEA messages are received
 * - Check if position/time fix has been acquired
 *
 * The GPS will automatically find its position/time once every GPS_TIMEOUT interval.
 * The function parseGPGGA explains what a "good fix" looks like, and how the readings
 * are averaged.
 * 
 * The GPS will automatically save the new "good fix" to the debug log and radar config.
 * 
 * Time is always saved in UTC. It is up to the user to convert it to the time zone based
 * on the current date and GPS position (usually PST/PDT)
 */
void GPSManager::gpsTask() {
  while (true) {
    
    // check if timeout exceeded OR millis overflowed (every 51 days)
    bool timeout_exceeded = ((millis() - m_lastGPSTime) > GPS_TIMEOUT) ||
                            ((millis() - m_lastGPSTime) < 0);

    // turn on GPS after timeout exceeded
    if (!m_isEnabled && timeout_exceeded)
    {
      powerOn();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // if on, check for (and process) any GPS NMEA messages
    if (m_isEnabled) {
      processIncomingData();
    }

    // if position/time located, save, then reset timeout and power off
    if (m_hasFix) {

      // send GPS data over Bluetooth
      m_pBT->sendMessage("=== New GPS Fix ===");
      m_pBT->sendMessage("Time: %02d:%02d:%02d UTC",
                         m_currentData.hour,
                         m_currentData.minute,
                         m_currentData.second);
      m_pBT->sendMessage("Location: %s %s",
                         m_currentData.latitude,
                         m_currentData.longitude);
      m_pBT->sendMessage("Elevation: %s",
                         m_currentData.elevation);
      m_pBT->sendMessage("==================");

      // TODO: save GPS position data to config
      m_lastGPSTime = millis();
      powerOff();
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


/**
 * @brief Enable / powers on the GPS receiver
 * @return none
 * 
 * MOSFET acts like a switch, connecting GPS GND to ESP32 GND. If FET is high, then
 * switch is closed and power is on. If FET is low, then switch is open and no power.
 */
void GPSManager::powerOn()
{
  digitalWrite(m_powerPin, HIGH);
  delay(100); // give GPS time to wake up

  // send wakeup command
  m_gpsSerial.write(0xFF);
  delay(100);

  m_isEnabled = true;
}


/**
 * @brief Disables / powers off the GPS receiver
 * @return none
 *
 * MOSFET acts like a switch, connecting GPS GND to ESP32 GND. If FET is high, then
 * switch is closed and power is on. If FET is low, then switch is open and no power.
 */
void GPSManager::powerOff()
{
  digitalWrite(m_powerPin, LOW);
  m_isEnabled = false;
  m_hasFix = false;
}


/**
 * @brief Adjusts the RTC time using the GPS readings
 * @return none
 *
 * Can't just overwrite RTC time with GPS time because of day/night boundary!
 * Need to be careful to preserve current date if close to midnight turnover
 * (23:59:59 -> 00:00:00)
 */
void GPSManager::syncRTCWithGPS()
{
  if (!m_hasFix || !m_pRTC)
    return;

  DateTime now = m_pRTC->now();

  // convert times to seconds for comparison
  int32_t rtc_seconds = ((int32_t)now.hour() * 60 + now.minute()) * 60 + now.second();
  int32_t gps_seconds = ((int32_t)m_currentData.hour * 60 + m_currentData.minute) * 60 + m_currentData.second;

  // calculate difference
  int32_t second_diff = gps_seconds - rtc_seconds;

  // handle day boundary cases
  // if difference is more than 12 hours, we're likely crossing midnight
  if (second_diff > 12 * 3600)
  {
    // GPS time is ahead, but we're actually crossing backwards
    // e.g., RTC: 00:01:00, GPS: 23:10:00 (previous day)
    second_diff -= 24 * 3600;
  }
  else if (second_diff < -12 * 3600)
  {
    // GPS time is behind, but we're actually crossing forwards
    // e.g., RTC: 23:55:00, GPS: 00:05:00 (next day)
    second_diff += 24 * 3600;
  }

  // update if there's a difference
  if (second_diff != 0)
  {
    DateTime adjustment = now;

    if (second_diff >= 0)
    {
      adjustment = now + TimeSpan(0, 0, 0, second_diff);
    }
    else
    {
      adjustment = now - TimeSpan(0, 0, 0, -second_diff);
    }

    m_pRTC->adjust(DateTime(adjustment.year(),
                            adjustment.month(),
                            adjustment.day(),
                            m_currentData.hour,
                            m_currentData.minute,
                            m_currentData.second));
  }
}


/**
 * @brief Processes incoming data from GPS, searching for GPGGA messages
 * @return none
 * 
 * First, checks if any new data from GPS UART. If so, checks if it's a GPGGA
 * message. If so, parse the message using parseGPGGA.
 */
void GPSManager::processIncomingData()
{
  static char buffer[MAX_SENTENCE_LENGTH];
  static size_t bufferIndex = 0;

  while (m_gpsSerial.available())
  {
    char c = m_gpsSerial.read();

    if (c == '$')
    { // start of new NMEA sentence
      bufferIndex = 0;
    }

    if (bufferIndex < MAX_SENTENCE_LENGTH - 1)
    {
      buffer[bufferIndex++] = c;

      if (c == '\n' || c == '\r')
      { // end of sentence
        buffer[bufferIndex] = '\0';

        if (strncmp(buffer, "$GPGGA", 6) == 0)
        {
          m_hasFix = parseGPGGA(buffer);
        }

        bufferIndex = 0;
      }
    }
    else
    {
      bufferIndex = 0; // buffer overflow protection
    }
  }
}


/**
 * @brief Parses GPGGA messages and calculates average position
 * @return true if fix is acquired (average position determined), false otherwise
 *
 * GPGGA messages contain relevant info for this implementation (lat/long/elev, time,
 * number of satellites, etc.). The first part of this function parses incoming
 * messages and rejects them if they fail to fit the format (or if not enough satellites
 * are in view).
 * 
 * If the message is valid, the function increments a counter validCount. The first
 * NUM_GPS_WARMUPS messages are discarded. After this, the lat/long/elev of NUM_GPS_AVERAGES 
 * messages are averaged to get a better estimate of position. Once all averages are taken,
 * the current time is adjusted to the current timezone, the lat/long are converted to the
 * proper format, and the time/lat/long/elev are saved.
 */
bool GPSManager::parseGPGGA(const char* sentence) {
  static int8_t validCount = -NUM_GPS_WARMUPS;
  static float latBin = 0.0f;
  static float longBin = 0.0f;
  static float elevBin = 0.0f;
  char* ptr = const_cast<char*>(sentence);
  
  // skip to time field
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;

  // parse time
  int time = atoi(ptr);
  m_currentData.hour = time / 10000;
  m_currentData.minute = (time / 100) % 100;
  m_currentData.second = time % 100;

  // skip to latitude
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // parse latitude
  float rawLat = atof(ptr);
  if (rawLat == 0.0f) return false;
  
  float latDegrees = (int)(rawLat / 100);
  float latMinutes = fmod(rawLat, 100.0);
  float latitude = latDegrees + (latMinutes / 60.0);
  
  // skip to N/S indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  char latDirection = *ptr;
  
  // skip to longitude
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // parse longitude
  float rawLon = atof(ptr);
  if (rawLon == 0.0f) return false;
  
  float lonDegrees = (int)(rawLon / 100);
  float lonMinutes = fmod(rawLon, 100.0);
  float longitude = lonDegrees + (lonMinutes / 60.0);
  
  // skip to E/W indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  char lonDirection = *ptr;

  // skip to fix quality
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int fixQuality = atoi(ptr);
  if (fixQuality == 0) return false;

  // skip to number of satellites
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int satellites = atoi(ptr);
  if (satellites < 4) return false;

  // skip to HDOP
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float hdop = atof(ptr);
  if (hdop > 5.0) return false;

  // parse elevation
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float elevation = atof(ptr);
  
  // increment valid count
  validCount++;
  if (validCount <= 0) return false;
  if (validCount <= NUM_GPS_AVERAGES)
  {
    latBin += latitude;
    longBin += longitude;
    elevBin += elevation;
    return false;
  }

  // if we get here, we've recorded enough to average
  latitude = latBin / validCount;
  longitude = longBin / validCount;
  elevation = elevBin / validCount;

  validCount = -NUM_GPS_WARMUPS;
  latBin = 0.0f;
  longBin = 0.0f;
  elevBin = 0.0f;

  // convert coordinates to DMS format
  convertDDtoDMS(latitude, m_currentData.latitude, sizeof(m_currentData.latitude), true, latDirection);
  convertDDtoDMS(longitude, m_currentData.longitude, sizeof(m_currentData.longitude), false, lonDirection);

  // format elevation
  snprintf(m_currentData.elevation, sizeof(m_currentData.elevation), "%.1f m", elevation);

  // sync new time with RTC
  syncRTCWithGPS();

  return true;
}


/**
 * @brief Converts lat/long from DD to DMS format
 * @param decimal_degrees current lat/long in decimal degrees format
 * @param result char* to place string result in
 * @param size size of char* result
 * @param isLat true if lat input, false if long input
 * @param direction "N"/"S" for lat, "E"/"W" for long
 * @return none
 * 
 * Decimal degrees (DD) are the format used by GPGGA messages. These are fine, but
 * degrees-minutes-seconds (DMS) matches Google Maps' format, which can simplify
 * data processing. This was a personal choice - feel free to change if needed.
 */
void GPSManager::convertDDtoDMS(float decimal_degrees, char* result, size_t size, bool isLat, char direction) {
  float abs_degrees = fabs(decimal_degrees);
  int degrees = (int)abs_degrees;
  float minutes = (abs_degrees - degrees) * 60;
  int whole_minutes = (int)minutes;
  float seconds = (minutes - whole_minutes) * 60;

  snprintf(result, size, "%d°%02d'%05.2f\"%c",
           degrees, whole_minutes, seconds, direction);
}


// TODO: MAKE THIS LOG TO THE DEBUG BUFFER AND PRINT TO SERIAL BT
/**
 * @brief Logs any input/output messages or debug statements
 * @param message const char* like "Error: GPS exploded :("
 * @return none
 */
void GPSManager::logStatus(const char *message)
{
  Serial.println(message);
}