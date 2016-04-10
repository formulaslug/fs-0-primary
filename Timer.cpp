#include "Timer.h"

Timer::Timer(uint32_t timeout) : m_timeout(timeout) {
  m_count = timeout;

  m_sysTimer.reset();
}

void Timer::update() {
  if (m_sysTimer.check() && m_count != 0) {
    m_count--;
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
