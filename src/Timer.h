#ifndef TIMER_H
#define TIMER_H

#include <cstdint>

class Timer {
 public:
  // @param timeout expiration value in milliseconds
  explicit Timer(uint32_t timeout);

  void update();
  bool isExpired();

 private:
  const uint32_t m_timeout;
  uint32_t m_count;

  uint8_t m_autoreset = 0;
  unsigned long m_previous_millis, m_interval_millis = 1;
};

#endif // TIMER_H
