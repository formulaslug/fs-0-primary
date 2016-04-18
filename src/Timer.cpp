#include "Timer.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

Timer::Timer(uint32_t timeout) : m_timeout(timeout) {
  m_count = timeout;
  m_previous_millis = millis();
}

void Timer::update() {
  if (millis() - m_previous_millis >= m_interval_millis) {
    // As suggested by benjamin.soelberg@gmail.com, the following line
    // m_previous_millis = millis();
    // was changed to
    // m_previous_millis += m_interval_millis;

    // If the interval is set to 0 we revert to the original behavior
    if (m_interval_millis <= 0 || m_autoreset) {
      m_previous_millis = millis();
    } else {
      m_previous_millis += m_interval_millis;
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
