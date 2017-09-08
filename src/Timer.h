// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

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
  uint32_t m_previousMillis, m_intervalMillis = 1;
};
