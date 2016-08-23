// Copyright (c) Formula Slug 2016. All Rights Reserved.

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

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
  uint32_t m_previous_millis, m_interval_millis = 1;
};

#endif  // TIMER_H
