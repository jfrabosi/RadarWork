// include/storage/TimeManager.h
#pragma once

#include "RTClib.h"
#include <mutex>
#include <string>

struct DateTimeMS
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
};

class TimeManager
{
public:
  // singleton pattern
  static TimeManager &getInstance()
  {
    static TimeManager instance;
    return instance;
  }

  bool initialize(RTC_PCF8523 &rtc);

  void getFormattedTimestamp(char *buffer, size_t size); // For data logging
  DateTimeMS getCurrentTimeMS();                         // For direct timestamp access if needed
  void resetInitialTime();
  void setDateTime(uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute, uint8_t second);

private:
  TimeManager() : m_isInitialized(false), m_pRTC(nullptr),
                  m_initTime{0}, m_initMillis(0), m_overflowCount(0),
                  m_lastUpdateTime(0) {}

  // prevent copying
  TimeManager(const TimeManager &) = delete;
  TimeManager &operator=(const TimeManager &) = delete;

  void updateCurrentTime();
  uint8_t daysInMonth(uint8_t month, uint16_t year) const;
  bool isLeapYear(uint16_t year) const;
  void checkMillisOverflow();
  void logStatus(const char *format, ...);

  bool m_isInitialized;       // is time manager set up?
  RTC_PCF8523 *m_pRTC;        // pointer to RTC object
  DateTimeMS m_initTime;      // RTC time at startup
  uint32_t m_initMillis;      // initial millis() at startup
  uint32_t m_overflowCount;   // how many times has millis() overflowed? (51 days)
  uint32_t m_lastUpdateTime;  // when was the time updated last? 
  DateTimeMS m_currentTime;   // what is the time right now?

  mutable std::mutex m_timeMutex;   

  static constexpr uint32_t MILLIS_OVERFLOW = 0xFFFFFFFF;
};