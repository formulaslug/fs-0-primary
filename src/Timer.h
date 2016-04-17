#ifndef TIMER_H
#define TIMER_H

#include <Metro.h>

class Timer {
 public:
  // @param timeout expiration value in milliseconds
  Timer(uint32_t timeout);

  void update();
  bool isExpired();

 private:
  const uint32_t m_timeout;
  uint32_t m_count;

  Metro m_sysTimer{1}; // ms
};

#endif // TIMER_H
