#include "communication/GPSManager.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

bool GPSManager::initialize(uint8_t rxPin, uint8_t txPin, uint8_t powerPin, RTC_PCF8523& rtcRef) {
  m_powerPin = powerPin;
  m_pRTC = &rtcRef;  // Store reference to RTC
  
  pinMode(m_powerPin, OUTPUT);
  digitalWrite(m_powerPin, HIGH);  // Start with GPS powered on

  m_gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  m_isEnabled = true;
  m_lastGPSTime = millis();

  return true;
}

void GPSManager::gpsTask() {
  while (true) {
    // Check for Bluetooth timeout
    if (!m_isEnabled && ((millis() - m_lastGPSTime) > GPS_TIMEOUT))
    {
      powerOn();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (m_isEnabled) {
      processIncomingData();
    }
    if (m_hasFix) {
      // todo: print/save GPS data
      Serial.println("\n=== GPS Location Saved ===");
      m_lastGPSTime = millis();
      // powerOff();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay to prevent task starvation
  }
}

void GPSManager::powerOn()
{
  digitalWrite(m_powerPin, HIGH);
  delay(100); // Give GPS time to wake up

  // Send wakeup command
  m_gpsSerial.write(0xFF);
  delay(100);

  m_isEnabled = true;
}

void GPSManager::powerOff()
{
  digitalWrite(m_powerPin, LOW);
  m_isEnabled = false;
  m_hasFix = false;
}

void GPSManager::syncRTCWithGPS()
{
  if (!m_hasFix || !m_pRTC)
    return;

  DateTime now = m_pRTC->now();

  // Convert times to seconds for comparison
  int32_t rtc_seconds = ((int32_t)now.hour() * 60 + now.minute()) * 60 + now.second();
  int32_t gps_seconds = ((int32_t)m_currentData.hour * 60 + m_currentData.minute) * 60 + m_currentData.second;

  // Calculate difference
  int32_t second_diff = gps_seconds - rtc_seconds;

  // Handle day boundary cases
  if (second_diff > 12 * 3600)
  {
    second_diff -= 24 * 3600;
  }
  else if (second_diff < -12 * 3600)
  {
    second_diff += 24 * 3600;
  }

  // Update if there's a difference
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

bool GPSManager::parseGPGGA(const char* sentence) {
  static int8_t validCount = -NUM_GPS_WARMUPS;
  static float latBin = 0.0f;
  static float longBin = 0.0f;
  static float elevBin = 0.0f;
  char* ptr = const_cast<char*>(sentence);
  
  // Skip to time field
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;

  // Parse time
  int time = atoi(ptr);
  m_currentData.hour = time / 10000;
  m_currentData.minute = (time / 100) % 100;
  m_currentData.second = time % 100;

  // Skip to latitude
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Parse latitude
  float rawLat = atof(ptr);
  if (rawLat == 0.0f) return false;
  
  float latDegrees = (int)(rawLat / 100);
  float latMinutes = fmod(rawLat, 100.0);
  float latitude = latDegrees + (latMinutes / 60.0);
  
  // Skip to N/S indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  char latDirection = *ptr;
  
  // Skip to longitude
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Parse longitude
  float rawLon = atof(ptr);
  if (rawLon == 0.0f) return false;
  
  float lonDegrees = (int)(rawLon / 100);
  float lonMinutes = fmod(rawLon, 100.0);
  float longitude = lonDegrees + (lonMinutes / 60.0);
  
  // Skip to E/W indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  char lonDirection = *ptr;

  // Skip to fix quality
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int fixQuality = atoi(ptr);
  if (fixQuality == 0) return false;

  // Skip to number of satellites
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int satellites = atoi(ptr);
  if (satellites < 4) return false;

  // Skip to HDOP
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float hdop = atof(ptr);
  if (hdop > 5.0) return false;

  // Parse elevation
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float elevation = atof(ptr);
  
  // Increment valid count
  validCount++;
  if (validCount <= 0) return false;
  if (validCount <= NUM_GPS_AVERAGES)
  {
    latBin += latitude;
    longBin += longitude;
    elevBin += elevation;
    return false;
  }

  // If we get here, we've recorded enough to average
  latitude = latBin / validCount;
  longitude = longBin / validCount;
  elevation = elevBin / validCount;

  validCount = -NUM_GPS_WARMUPS;
  latBin = 0.0f;
  longBin = 0.0f;
  elevBin = 0.0f;

  // Apply timezone correction
  int8_t tzOffset = getUTCOffset();
  m_currentData.hour += tzOffset;
  if (m_currentData.hour < 0) {
    m_currentData.hour += 24;
  } else if (m_currentData.hour >= 24) {
    m_currentData.hour -= 24;
  }

  // Convert coordinates to DMS format
  convertDMtoDMS(latitude, m_currentData.latitude, sizeof(m_currentData.latitude), true, latDirection);
  convertDMtoDMS(longitude, m_currentData.longitude, sizeof(m_currentData.longitude), false, lonDirection);
  
  // Format elevation
  snprintf(m_currentData.elevation, sizeof(m_currentData.elevation), "%.1f m", elevation);

  return true;
}

void GPSManager::convertDMtoDMS(float decimal_degrees, char* result, size_t size, bool isLat, char direction) {
  float abs_degrees = fabs(decimal_degrees);
  int degrees = (int)abs_degrees;
  float minutes = (abs_degrees - degrees) * 60;
  int whole_minutes = (int)minutes;
  float seconds = (minutes - whole_minutes) * 60;

  snprintf(result, size, "%d°%02d'%05.2f\"%c",
           degrees, whole_minutes, seconds, direction);
}

int8_t GPSManager::getUTCOffset() {
  if (!m_pRTC) return -8;  // Default to PST if no RTC
  
  DateTime now = m_pRTC->now();
  return isDST(now.year(), now.month(), now.day(), now.hour()) ? -7 : -8;
}

bool GPSManager::isDST(uint16_t year, uint8_t month, uint8_t day, uint8_t hour) {
  // DST rules for Pacific Time Zone:
  // Starts second Sunday in March at 2 AM
  // Ends first Sunday in November at 2 AM
  
  if (month < 3 || month > 11) {
    return false;  // Jan, Feb, Dec are always Standard Time
  }
  
  if (month > 3 && month < 11) {
    return true;   // Apr to Oct are always DST
  }

  int targetSunday;
  if (month == 3) {
    // Second Sunday in March
    targetSunday = 14 - ((year + year/4 + 2) % 7);
    return (day > targetSunday || (day == targetSunday && hour >= 2));
  } else {  // month == 11
    // First Sunday in November
    targetSunday = 7 - ((year + year/4 + 2) % 7);
    return (day < targetSunday || (day == targetSunday && hour < 2));
  }
}

void GPSManager::processIncomingData()
{
  static char buffer[MAX_SENTENCE_LENGTH];
  static size_t bufferIndex = 0;
  static uint32_t sentenceCount = 0;

  while (m_gpsSerial.available())
  {
    char c = m_gpsSerial.read();

    if (c == '$')
    { // Start of new NMEA sentence
      bufferIndex = 0;
    }

    if (bufferIndex < MAX_SENTENCE_LENGTH - 1)
    {
      buffer[bufferIndex++] = c;

      if (c == '\n' || c == '\r')
      { // End of sentence
        buffer[bufferIndex] = '\0';

        if (strncmp(buffer, "$GPGGA", 6) == 0)
        {
          // Debug print if enabled
          if (m_debugMode)
          {
            printDebugInfo(buffer);
          }
          m_hasFix = parseGPGGA(buffer);
        }

        bufferIndex = 0;
        sentenceCount++;
      }
    }
    else
    {
      bufferIndex = 0; // Buffer overflow protection
    }
  }
}

void GPSManager::printDebugInfo(const char *sentence)
{
  uint32_t currentTime = millis();

  // Only print debug info once per DEBUG_PRINT_INTERVAL
  if (currentTime - m_lastDebugPrint >= DEBUG_PRINT_INTERVAL)
  {
    // Extract sentence type (characters between $ and first comma)
    char sentenceType[6] = {0};
    const char *commaPos = strchr(sentence, ',');
    if (commaPos)
    {
      size_t typeLen = commaPos - (sentence + 1);
      if (typeLen < 6)
      {
        strncpy(sentenceType, sentence + 1, typeLen);
        sentenceType[typeLen] = '\0';
      }
    }

    Serial.println("\n=== GPS Debug Info ===");
    Serial.printf("Received NMEA Sentence Type: %s\n", sentenceType);
    Serial.printf("Raw NMEA Data: %s\n", sentence);
    Serial.printf("Fix Status: %s\n", m_hasFix ? "Valid Fix" : "No Fix");
    if (m_hasFix)
    {
      Serial.printf("Current Position: %s %s\n",
                    m_currentData.latitude, m_currentData.longitude);
    }

    m_lastDebugPrint = currentTime;
  }
}