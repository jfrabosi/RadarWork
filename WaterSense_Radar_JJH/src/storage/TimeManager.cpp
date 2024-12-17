// src/storage/TimeManager.cpp
#include "storage/TimeManager.h"
#include "storage/SDCardManager.h"
#include "communication/BluetoothManager.h"
#include <Arduino.h>
#include <stdarg.h>


/**
 * @brief Initializes the Time Manager
 * @param rtc Reference to RTC object
 * @return true for successful initialization, false if error occurred
 *
 * Initializes RTC connection, checks for power loss, and sets initial time.
 * If RTC lost power, time is set from compile time as fallback.
 */
bool TimeManager::initialize(RTC_PCF8523 &rtc)
{
  std::lock_guard<std::mutex> lock(m_timeMutex);

  m_pRTC = &rtc;

  if (!m_pRTC->begin())
  {
    logStatus("Couldn't find RTC");
    return false;
  }

  if (!m_pRTC->initialized() || m_pRTC->lostPower())
  {
    logStatus("RTC lost power, setting time from compile time!");
    m_pRTC->adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  m_pRTC->start();

  // Set initial time
  DateTime now = m_pRTC->now();
  m_initTime.year = now.year();
  m_initTime.month = now.month();
  m_initTime.day = now.day();
  m_initTime.hour = now.hour();
  m_initTime.minute = now.minute();
  m_initTime.second = now.second();
  m_initTime.millisecond = millis() % 1000;
  m_initMillis = millis();

  m_currentTime = m_initTime;
  m_lastUpdateTime = m_initMillis;
  m_overflowCount = 0;

  m_isInitialized = true;

  logStatus("\nRTC initialized\n");

  return true;
}


/**
 * @brief Gets current time formatted as timestamp string
 * @param buffer Buffer to store formatted timestamp
 * @param size Size of buffer (must be >= 25 bytes)
 * @return none
 *
 * Formats current time as [DD/MM/YY HH:MM:SS.mmm]
 * Buffer must be at least 25 bytes to hold full timestamp
 */
void TimeManager::getFormattedTimestamp(char *buffer, size_t size)
{
  if (!buffer || size < 25)
  { // Minimum size for timestamp
    if (buffer && size > 0)
    {
      buffer[0] = '\0';
    }
    return;
  }

  DateTimeMS current = getCurrentTimeMS();

  snprintf(buffer, size, "[%02d/%02d/%02d %02d:%02d:%02d.%03d]",
           current.year % 100, current.month, current.day,
           current.hour, current.minute, current.second,
           current.millisecond);
}


/**
 * @brief Gets current time with millisecond precision
 * @return DateTimeMS struct with current time
 *
 * Returns current time combining RTC time with internal millisecond counter.
 * Updates internal time if update interval exceeded.
 */
DateTimeMS TimeManager::getCurrentTimeMS()
{
  if (!m_isInitialized)
  {
    return DateTimeMS{0};
  }

  std::lock_guard<std::mutex> lock(m_timeMutex);

  updateCurrentTime();

  return m_currentTime;
}


/**
 * @brief Resets initial time reference
 * @return none
 *
 * Updates initial time reference to current RTC time.
 * Used when starting new data files or after long operations.
 */
void TimeManager::resetInitialTime()
{
  std::lock_guard<std::mutex> lock(m_timeMutex);

  // Get current RTC time
  DateTime now = m_pRTC->now();

  // Update initial time
  m_initTime.year = now.year();
  m_initTime.month = now.month();
  m_initTime.day = now.day();
  m_initTime.hour = now.hour();
  m_initTime.minute = now.minute();
  m_initTime.second = now.second();
  m_initTime.millisecond = millis() % 1000;

  // Reset other counters
  m_initMillis = millis();
  m_overflowCount = 0;
  m_lastUpdateTime = m_initMillis;

  // Force an immediate update of current time
  updateCurrentTime();
}


/**
 * @brief Sets current date and time
 * @param year Four-digit year
 * @param month Month (1-12)
 * @param day Day (1-31)
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 * @return none
 *
 * Updates both RTC and internal time reference
 */
void TimeManager::setDateTime(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second)
{
  std::lock_guard<std::mutex> lock(m_timeMutex);

  if (!m_pRTC)
    return;

  // Update RTC
  m_pRTC->adjust(DateTime(year, month, day, hour, minute, second));

  // Reset our timing variables to match
  DateTime now = m_pRTC->now();
  m_initTime.year = now.year();
  m_initTime.month = now.month();
  m_initTime.day = now.day();
  m_initTime.hour = now.hour();
  m_initTime.minute = now.minute();
  m_initTime.second = now.second();
  m_initTime.millisecond = millis() % 1000;
  m_initMillis = millis();
  m_overflowCount = 0;

  // Force immediate update of current time
  updateCurrentTime();
}


/**
 * @brief Updates internal current time
 * @return none
 *
 * Calculates current time based on initial time and elapsed milliseconds.
 * Handles millisecond counter overflow and date rollovers.
 */
void TimeManager::updateCurrentTime()
{
  checkMillisOverflow();

  uint32_t currentMillis = millis();
  uint64_t elapsedMS;

  if (currentMillis >= m_initMillis)
  {
    elapsedMS = currentMillis - m_initMillis;
  }
  else
  {
    elapsedMS = (MILLIS_OVERFLOW - m_initMillis) + currentMillis + 1UL;
  }
  elapsedMS += (uint64_t)MILLIS_OVERFLOW * m_overflowCount;

  // Start with initial time
  m_currentTime = m_initTime;

  // Add milliseconds and handle rollovers
  m_currentTime.millisecond = (elapsedMS % 1000);
  uint64_t totalSeconds = elapsedMS / 1000;

  // Add seconds and handle rollovers
  m_currentTime.second += totalSeconds % 60;
  uint64_t totalMinutes = totalSeconds / 60;
  if (m_currentTime.second >= 60)
  {
    m_currentTime.second -= 60;
    totalMinutes++;
  }

  // Add minutes and handle rollovers
  m_currentTime.minute += totalMinutes % 60;
  uint64_t totalHours = totalMinutes / 60;
  if (m_currentTime.minute >= 60)
  {
    m_currentTime.minute -= 60;
    totalHours++;
  }

  // Add hours and handle rollovers
  m_currentTime.hour += totalHours % 24;
  uint64_t totalDays = totalHours / 24;
  if (m_currentTime.hour >= 24)
  {
    m_currentTime.hour -= 24;
    totalDays++;
  }

  // Add remaining days, handling month and year transitions
  while (totalDays > 0)
  {
    uint8_t daysThisMonth = daysInMonth(m_currentTime.month, m_currentTime.year);
    if (m_currentTime.day + totalDays > daysThisMonth)
    {
      totalDays -= (daysThisMonth - m_currentTime.day + 1);
      m_currentTime.day = 1;
      if (++m_currentTime.month > 12)
      {
        m_currentTime.month = 1;
        m_currentTime.year++;
      }
    }
    else
    {
      m_currentTime.day += totalDays;
      break;
    }
  }

  m_lastUpdateTime = currentMillis;
}


/**
 * @brief Checks for millisecond counter overflow
 * @return none
 *
 * Updates overflow counter and re-syncs with RTC when overflow occurs.
 * Prevents time drift over long periods.
 */
void TimeManager::checkMillisOverflow()
{
  uint32_t currentMillis = millis();
  if (currentMillis < m_lastUpdateTime)
  {
    m_overflowCount++;

    // Re-sync with RTC on overflow to prevent drift
    if (m_pRTC)
    {
      DateTime now = m_pRTC->now();
      m_initTime.year = now.year();
      m_initTime.month = now.month();
      m_initTime.day = now.day();
      m_initTime.hour = now.hour();
      m_initTime.minute = now.minute();
      m_initTime.second = now.second();
      m_initTime.millisecond = currentMillis % 1000;
      m_initMillis = currentMillis;
      m_overflowCount = 0;
    }
  }
}


/**
 * @brief Gets number of days in specified month
 * @param month Month (1-12)
 * @param year Four-digit year
 * @return Number of days in month, accounting for leap years
 */
uint8_t TimeManager::daysInMonth(uint8_t month, uint16_t year) const
{
  static const uint8_t daysPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  if (month == 0 || month > 12)
  {
    return 0;
  }

  if (month == 2 && isLeapYear(year))
  {
    return 29;
  }

  return daysPerMonth[month - 1];
}


/**
 * @brief Checks if specified year is a leap year
 * @param year Four-digit year to check
 * @return true if leap year, false otherwise
 */
bool TimeManager::isLeapYear(uint16_t year) const
{
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}


/**
 * @brief Logs any input/output messages or debug statements to the debug log
 * @param format --- Treat this function like a wrapper for printf! ---
 * @return none
 *
 * Adds "Time:" prefix to all messages
 * Prints to Serial and queues for SD card if available
 */
void TimeManager::logStatus(const char *format, ...)
{
  // Let's use reasonable buffer size, similar to others
  static constexpr size_t MAX_MESSAGE_SIZE = 256;
  char messageBuffer[MAX_MESSAGE_SIZE];

  // Handle variable arguments
  va_list args;
  va_start(args, format);
  vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);
  va_end(args);

  // Print to Serial
  Serial.println(messageBuffer);

  // Queue for SD card if available
  SDCardManager::getInstance().queueDebug("Time: %s", messageBuffer);
}


void TimeManager::performLightSleep()
{
  // First check if SD card is busy
  if (SDCardManager::getInstance().hasActiveOperations())
  {
    logStatus("Skipping deep sleep - SD card operations in progress");
    return;
  }

  // Calculate processing time
  uint32_t processingTime = m_processingEndTime - m_processingStartTime;

  // Calculate sleep time (0.99 * period - processing - delay)
  float sleepTime = (0.99f * m_targetSamplePeriod) - processingTime - 10.0f;
  // Serial.println(sleepTime);

  if (sleepTime > 0)
  {
    uint32_t sleepStart = millis();
    uint32_t targetWake = sleepStart + (uint32_t)sleepTime;

    // Split into smaller sleep intervals to maintain system stability
    while (millis() < targetWake)
    {
      // Sleep in 10ms chunks
      uint32_t remainingTime = targetWake - millis();
      uint32_t sleepChunk = remainingTime < 10UL ? remainingTime : 10UL;

      if (sleepChunk > 0)
      {
        esp_sleep_enable_timer_wakeup(sleepChunk * 1000); // convert to microseconds
        esp_light_sleep_start();
      }

      // Brief task delay to allow system processing
      vTaskDelay(1);
    }
  }
  // Serial.printf("wakeup: %lu\n", millis());
}