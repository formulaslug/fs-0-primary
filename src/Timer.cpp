// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include "Timer.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

Timer::Timer(uint32_t timeout) : m_timeout(timeout) {
  m_count = timeout;
  m_previousMillis = millis();
}

void Timer::update() {
  if (millis() - m_previousMillis >= m_intervalMillis) {
    // As suggested by benjamin.soelberg@gmail.com, the following line
    // m_previousMillis = millis();
    // was changed to
    // m_previousMillis += m_intervalMillis;

    // If the interval is set to 0 we revert to the original behavior
    if (m_intervalMillis <= 0 || m_autoreset) {
      m_previousMillis = millis();
    } else {
      m_previousMillis += m_intervalMillis;
    }

    if (m_count != 0) {
      m_count--;
    }
  }
}

bool Timer::isExpired() {
  if (m_count == 0) {
    m_count = m_timeout;
    return true;
  } else {
    return false;
  }
}
